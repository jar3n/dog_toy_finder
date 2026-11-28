r"""Offline batch evaluation of saved rectified stereo shots.

Captures are written first by ``vision_capture``; this tool replays every saved
shot through every vision pipeline, then writes one trial CSV per pipeline that
already contains the operator's ground truth. Every object in a scene is scored:
the primary target plus each entry in ``ground_truth.additional_objects``. A
scene captured with ``--shots 3`` has three independently scored shots that
share the same operator-measured ground truth for each object.

By default (or with ``--pipeline all``), one invocation runs ``triangulation`` and
``stereo-net`` in sequence against the same captures and the same shuffle seed.
Before each pipeline, the evaluator tells the already-running host services which
detector/depth models to load from the config. After each pipeline, it unloads
those models before the next pipeline loads its own; the HTTP services stay
running. Use ``--pipeline triangulation`` or ``--pipeline stereo-net`` to evaluate
only one pipeline. ``stereo-net`` also needs the host depth service running.

No live camera topics and no ROS graph are needed. Only the shared detector
config, the saved JPEGs, and the host services (detector, plus the depth service
for ``stereo-net``) are used.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_eval \
        --captures-dir /workspace/data/vision/captures

Scenes are shuffled first, then the shots within each scene are shuffled. Use
``--seed`` to reproduce an order; the generated seed, scene order, and shot
order are recorded in each summary manifest. Both multi-shot scenes and legacy
flat captures are discovered. ``--pilot`` selects
``/workspace/data/vision/pilot_captures`` for both replay input and default
outputs, keeping trial data out of the normal captures tree; explicit paths win.

Default all-pipeline outputs (next to the captures directory, or inside the
pilot root with ``--pilot``)::

    vision_eval_triangulation_trials.csv      per-object scored rows
    vision_eval_triangulation_results.jsonl   full /vision/result-style entries
    vision_eval_triangulation_summary.json    aggregate metrics and shuffle order
    vision_eval_stereo-net_*.{csv,jsonl,json}
    vision_eval_all_trials.csv             combined rows from both pipelines
    vision_eval_all_summary.json           index of the per-pipeline outputs

With an explicit output path, each pipeline's name is inserted before the file
extension (``trials.csv`` becomes ``trials_triangulation.csv``). A single-pipeline
run with ``--pipeline NAME`` keeps the generic ``vision_eval_*`` names.

If interrupted with Ctrl+C, the evaluator unloads the host models and removes
any output files created by that run; files that already existed before the run
are restored instead of deleted.
"""

import argparse
import json
import os
import random
import re
import secrets
import signal
import sys
from datetime import datetime

from . import depth_client, vision_config, vision_pipelines, yolo_client

# Both packages are installed together in the ROS workspace. The fallback lets
# the tool run from a source checkout in a plain Python environment, where the
# outer ROS package directory is a namespace package rather than importable.
try:
    from dog_toy_finder_data import llm_trials
except ImportError:  # pragma: no cover - only used outside an installed workspace
    _SRC_ROOT = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    sys.path.insert(0, _SRC_ROOT)
    sys.path.insert(0, os.path.join(
        _SRC_ROOT, "dog_toy_finder_data", "dog_toy_finder_data"))
    try:
        from dog_toy_finder_data import llm_trials
    except ImportError:
        import llm_trials


def _is_shot_dir(path):
    """Return True when a directory holds one complete rectified stereo shot."""
    return all(os.path.isfile(os.path.join(path, name))
               for name in ("left.jpg", "right.jpg", "meta.json"))


def _read_meta(path):
    with open(os.path.join(path, "meta.json")) as handle:
        return json.load(handle)


def _meta_run_id(meta, path):
    """Use the recorded run id, falling back to the parent run directory."""
    value = meta.get("run_id")
    if value:
        return str(value)
    if path and re.match(r"^shot_\d+$", os.path.basename(path)):
        return os.path.basename(os.path.dirname(os.path.dirname(path)))
    return os.path.basename(os.path.dirname(path)) if path else "captures"


def _scene_id(meta, path=None):
    """Return the scene key shared by all shots of one capture scene."""
    value = meta.get("scene_id") or meta.get("capture_id")
    if value:
        return str(value)
    if path:
        base = os.path.basename(path)
        if re.match(r"^shot_\d+$", base):
            return os.path.basename(os.path.dirname(path))
        return base
    return "captures"


def _object_label(value):
    """Return a label from an additional-object entry or a plain string."""
    if isinstance(value, dict):
        value = (value.get("label") or value.get("name")
                 or value.get("target"))
    return str(value or "").strip()


def scene_objects(meta):
    """Return every labelled object to score in one shot's metadata.

    The primary target uses the scene-level ground truth. Additional objects
    come from ``ground_truth.additional_objects``; legacy captures may store
    bare labels instead of per-object dictionaries. A stable ``object_id``
    keeps repeated labels in one shot separate without changing the trial CSV
    schema: it suffixes each additional object's ``ask_id`` (and ``trial_id``),
    while the primary object keeps the historic shot ID.
    """
    ground_truth = meta.get("ground_truth")
    if not isinstance(ground_truth, dict):
        ground_truth = {}
    objects = []
    primary = _object_label(meta.get("target"))
    if primary:
        objects.append({
            "object_id": "primary",
            "role": "primary",
            "label": primary,
            "target": primary,
            "ground_truth": dict(ground_truth),
        })

    extras = ground_truth.get("additional_objects") or []
    if isinstance(extras, (str, dict)):
        extras = [extras]
    for index, item in enumerate(extras, 1):
        label = _object_label(item)
        if not label:
            continue
        object_ground_truth = dict(item) if isinstance(item, dict) else {}
        # Additional objects are captured as present; older bare-label entries
        # have no numeric truth, but can still be checked for visibility.
        object_ground_truth.setdefault("target_visible", True)
        for key in ("obstacle_ahead", "notes", "occlusion_percent",
                    "visible_side"):
            if key in ground_truth and key not in object_ground_truth:
                object_ground_truth[key] = ground_truth[key]
        objects.append({
            "object_id": "additional_%02d" % index,
            "role": "additional",
            "label": label,
            "target": label,
            "ground_truth": object_ground_truth,
        })
    return objects


def _shuffle_scenes_and_shots(selected, rng):
    """Order shots by shuffled scene, then shuffled shots within each scene."""
    groups = {}
    for path, meta in selected:
        key = (_meta_run_id(meta, path), _scene_id(meta, path))
        groups.setdefault(key, []).append(path)
    keys = list(groups)
    rng.shuffle(keys)
    ordered = []
    for key in keys:
        shots = groups[key]
        rng.shuffle(shots)
        ordered.extend(shots)
    return ordered


def _as_int(value, default=1):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def iter_captures(root, run_id=None, capture=None, rng=None):
    """Yield complete shot directories, optionally shuffled by ``rng``.

    ``vision_capture`` writes multi-shot scenes as ``<capture_id>/shot_NN/``;
    older single-shot captures keep ``left.jpg`` and ``meta.json`` directly in
    ``<capture_id>/``. Both layouts are discovered here. ``--capture`` may point
    at either one shot directory or a whole scene directory.

    Discovery is sorted so the set of shots is deterministic; ``main`` supplies
    a seeded random generator so scenes and the shots inside each scene are
    shuffled reproducibly. Callers that omit ``rng`` keep the sorted order
    (useful for tests and tooling that needs a stable listing).
    """
    if capture:
        start = os.path.abspath(os.path.expanduser(capture))
        if _is_shot_dir(start):
            paths = [start]
        else:
            paths = []
            for current, directories, files in os.walk(start):
                directories.sort()
                if _is_shot_dir(current):
                    paths.append(current)
    else:
        root = os.path.abspath(os.path.expanduser(root))
        paths = []
        for current, directories, files in os.walk(root):
            directories.sort()
            if _is_shot_dir(current):
                paths.append(current)
    paths = sorted(paths)

    selected = []
    for path in paths:
        try:
            meta = _read_meta(path)
        except (OSError, ValueError):
            continue
        if run_id and _meta_run_id(meta, path) != run_id:
            continue
        selected.append((path, meta))
    if rng is not None:
        ordered = _shuffle_scenes_and_shots(selected, rng)
    else:
        ordered = [path for path, _ in selected]
    for path in ordered:
        yield path


def shot_id(meta, path=None):
    """Return the pairing key for one shot.

    Flat legacy captures keep their ``capture_id`` as the ID. Multi-shot scenes
    append ``_shot_NN`` so every shot is paired across models without two
    shots of the same scene colliding in the trial CSV.
    """
    fallback = os.path.basename(os.path.dirname(path)) if path else "capture"
    capture_id = str(meta.get("capture_id") or meta.get("scene_id") or fallback)
    index = meta.get("shot_index")
    count = _as_int(meta.get("shot_count"), 1)
    if count > 1 and index not in (None, ""):
        return "%s_shot_%02d" % (capture_id, _as_int(index))
    return capture_id


def _image_relative_path(meta, path, eye):
    """Return the saved JPEG path relative to the captures root."""
    fallback = os.path.basename(os.path.dirname(path)) if path else "capture"
    capture_id = str(meta.get("capture_id") or meta.get("scene_id") or fallback)
    run_id = _meta_run_id(meta, path)
    index = meta.get("shot_index")
    count = _as_int(meta.get("shot_count"), 1)
    if count > 1 and index not in (None, ""):
        shot = "shot_%02d" % _as_int(index)
        return os.path.join(run_id, capture_id, shot, eye + ".jpg")
    return os.path.join(run_id, capture_id, eye + ".jpg")


def load_capture(path):
    """Load a capture's metadata and the two rectified JPEG byte strings."""
    with open(os.path.join(path, "meta.json")) as handle:
        meta = json.load(handle)
    images = {}
    for eye in ("left", "right"):
        with open(os.path.join(path, "%s.jpg" % eye), "rb") as handle:
            images[eye + "_jpeg"] = handle.read()
    return meta, images


def evaluate_capture(path, config, pipeline=None, client=None,
                     depth_client=None):
    """Replay one shot for every object and return a /vision/result-style dict.

    Top-level fields describe the shot; ``result["objects"]`` holds one entry
    per scored object with its own ``id``, target, ground truth, and pipeline
    answers. The flat ``results`` list keeps the older single-shot shape for
    inspection tools.
    """
    meta, images = load_capture(path)
    name = vision_pipelines.pipeline_name(pipeline, config)
    capture_id = str(meta.get("capture_id") or os.path.basename(path))
    scene_id = str(meta.get("scene_id") or capture_id)
    base_id = shot_id(meta, path)
    result = {
        "id": base_id,
        "scene_id": scene_id,
        "shot_index": _as_int(meta.get("shot_index"), 1),
        "shot_count": _as_int(meta.get("shot_count"), 1),
        "target": str(meta.get("target") or ""),
        "pipeline": name,
        "models": [],
        "objects": [],
        "results": [],
        "stereo": True,
        "image_source": "rectified_color",
        "image": _image_relative_path(meta, path, "left"),
        "capture_dir": path,
    }
    for obj in scene_objects(meta):
        entries = vision_pipelines.run_pipeline(
            name, obj["target"], images["left_jpeg"], images["right_jpeg"],
            config, client=client, depth_client=depth_client)
        # The primary object keeps the historic shot ID so existing CSVs and
        # paired analyses remain comparable; additional objects get a suffix.
        suffix = "" if obj["role"] == "primary" else "__%s" % obj["object_id"]
        object_result = dict(result)
        object_result.update({
            "id": base_id + suffix,
            "shot_id": base_id,
            "object_id": obj["object_id"],
            "object_role": obj["role"],
            "label": obj["label"],
            "target": obj["target"],
            "ground_truth": obj["ground_truth"],
            "objects": [],
            "results": entries,
            "models": [entry.get("model", "unknown") for entry in entries],
        })
        result["objects"].append(object_result)
        result["results"].extend(entries)
    result["models"] = list(dict.fromkeys(
        entry.get("model", "unknown") for entry in result["results"]))
    return result, meta


def _default_output(directory, name):
    return os.path.join(directory, name)


def _requested_pipelines(explicit=None):
    """Return the pipelines one evaluator invocation should run.

    Omitting ``--pipeline`` (or passing ``all``) runs every known pipeline in the
    canonical order. An explicit pipeline name keeps the older one-pipeline
    behaviour. Environment/YAML pipeline selection governs the live node; the
    offline evaluator is intentionally all-pipeline by default.
    """
    value = "" if explicit is None else str(explicit).strip().lower()
    if value in ("", "all", "*"):
        return list(vision_pipelines.PIPELINES)
    return [vision_pipelines.pipeline_name(value)]


def _with_pipeline_suffix(path, pipeline):
    """Return ``path`` with ``_pipeline`` inserted before its extension."""
    if not path:
        return path
    base, extension = os.path.splitext(path)
    return "%s_%s%s" % (base, pipeline, extension)


def _log(message):
    """Print progress immediately so long model calls do not look stalled."""
    print(message, flush=True)


def _pipeline_status(pipeline, config, detector=None, depth=None):
    """Describe the active pipeline configuration for progress messages."""
    detector = (detector
                or vision_config.get(config, "open_vocab", "name")
                or "unknown")
    parts = ["pipeline=%s" % pipeline, "detector=%s" % detector]
    if pipeline == vision_pipelines.STEREO_NET:
        depth = (depth
                 or vision_config.get(config, "stereo-net", "name")
                 or "unknown")
        parts.append("depth=%s" % depth)
    return " ".join(parts)


def _reported_models(result, pipeline):
    """Collect model names actually reported by one shot's pipeline answers."""
    models = {}
    for entry in result.get("results", []):
        if pipeline == vision_pipelines.STEREO_NET:
            models["depth"] = models.get("depth") or entry.get("model")
            models["detector"] = (models.get("detector")
                                  or entry.get("detector_model"))
        else:
            models["detector"] = models.get("detector") or entry.get("model")
    return models


def _configure_pipeline(pipeline, config):
    """Tell the host services which models ``pipeline`` should use.

    The HTTP processes are started without a model selection, so this must run
    before the first shot of each pipeline.
    """
    if pipeline == vision_pipelines.STEREO_NET:
        depth_client.configure(config)
    yolo_client.configure(config)


def _unload_pipeline(pipeline, config):
    """Release the host models used by ``pipeline`` after its evaluation.

    The host HTTP services stay alive, but their loaded detector/depth models and
    caches are dropped. The next pipeline therefore starts with fresh model
    loads instead of sharing weights with the previous configuration.
    ``stereo-net`` also uses the detector, so both host services are unloaded.
    """
    status = {}
    if pipeline == vision_pipelines.STEREO_NET:
        status["depth"] = depth_client.unload(config)
    status["detector"] = yolo_client.unload(config)
    return status


def _pipeline_outputs(args, output_root, pipeline, all_pipelines):
    """Return (trial CSV, results JSONL, summary JSON) for one pipeline."""
    if not all_pipelines:
        return (
            args.output_csv or _default_output(
                output_root, "vision_eval_trials.csv"),
            args.results_json or _default_output(
                output_root, "vision_eval_results.jsonl"),
            args.summary_json or _default_output(
                output_root, "vision_eval_summary.json"))
    return (
        _with_pipeline_suffix(args.output_csv, pipeline) if args.output_csv
        else _default_output(output_root,
                             "vision_eval_%s_trials.csv" % pipeline),
        _with_pipeline_suffix(args.results_json, pipeline) if args.results_json
        else _default_output(output_root,
                             "vision_eval_%s_results.jsonl" % pipeline),
        _with_pipeline_suffix(args.summary_json, pipeline) if args.summary_json
        else _default_output(output_root,
                             "vision_eval_%s_summary.json" % pipeline))


def _all_summary_output(args, output_root):
    """Return the manifest path tying together an all-pipeline run."""
    if args.summary_json:
        return _with_pipeline_suffix(args.summary_json, "all")
    return _default_output(output_root, "vision_eval_all_summary.json")


def _all_trials_output(args, output_root):
    """Return the combined CSV path for an all-pipeline run."""
    if args.output_csv:
        return _with_pipeline_suffix(args.output_csv, "all")
    return _default_output(output_root, "vision_eval_all_trials.csv")


def _capture_root(path):
    """Infer the captures root from an explicit shot or scene directory."""
    path = os.path.abspath(os.path.expanduser(path))
    if re.match(r"^shot_\d+$", os.path.basename(path)):
        return os.path.dirname(os.path.dirname(os.path.dirname(path)))
    return os.path.dirname(os.path.dirname(path))


def _build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--captures-dir", default=None,
                        help="capture root (default: data/vision/captures, "
                             "or data/vision/pilot_captures with --pilot)")
    parser.add_argument("--pilot", action="store_true",
                        help="read pilot captures and write default outputs "
                             "inside the pilot root (explicit paths win)")
    parser.add_argument("--capture", default=None,
                        help="evaluate one shot directory or a whole scene "
                             "directory with all of its shots")
    parser.add_argument("--run-id", default=None,
                        help="only evaluate captures from this run id")
    parser.add_argument("--config", default=None, help="detector YAML path")
    parser.add_argument("--pipeline", default=None,
                        help="vision pipeline: triangulation|stereo-net|all "
                             "(default: all; NAME evaluates one pipeline only)")
    parser.add_argument("--tolerance", type=float, default=0.25,
                        help="distance tolerance in metres")
    parser.add_argument("--bearing-tolerance", type=float, default=5.0,
                        help="lateral bearing tolerance in degrees")
    parser.add_argument("--limit", type=int, default=0,
                        help="stop after this many shots (0 = all)")
    parser.add_argument("--seed", type=int, default=None,
                        help="scene/shot shuffle seed (default: random; "
                             "recorded in the summary manifest)")
    parser.add_argument("--output-csv", default=None, help="trial CSV path")
    parser.add_argument("--results-json", default=None, help="full results JSONL")
    parser.add_argument("--summary-json", default=None, help="aggregate JSON path")
    parser.add_argument("--json", action="store_true",
                        help="print the per-model summary as JSON")
    return parser


def _evaluate_pipeline(shots, root, config, args, pipeline, outputs):
    """Evaluate one pipeline over the shared shot order and write its outputs."""
    output_csv, results_json, summary_json = outputs
    rows = []
    results = []
    scene_order = []
    shots_per_scene = {}
    objects_per_scene = {}
    objects_evaluated = 0
    detector_model = None
    depth_model = None
    _log("[eval] starting %s tolerance=%.3f bearing_tolerance=%.3f"
         % (_pipeline_status(pipeline, config), args.tolerance,
            args.bearing_tolerance))
    for path in shots:
        try:
            shot_meta = _read_meta(path)
        except (OSError, ValueError):
            shot_meta = {}
        scene_id = _scene_id(shot_meta, path)
        shot_label = shot_id(shot_meta, path)
        _log("[eval] processing %s scene=%s shot=%s"
             % (_pipeline_status(pipeline, config, detector_model,
                                 depth_model),
                scene_id, shot_label))
        try:
            result, meta = evaluate_capture(path, config, pipeline)
        except Exception as error:
            print("Failed to replay %s with %s: %s"
                  % (path, pipeline, error), file=sys.stderr)
            continue
        reported = _reported_models(result, pipeline)
        detector_model = reported.get("detector") or detector_model
        depth_model = reported.get("depth") or depth_model
        results.append(result)
        scene_id = result.get("scene_id") or "captures"
        if scene_id not in shots_per_scene:
            scene_order.append(scene_id)
            shots_per_scene[scene_id] = 0
        shots_per_scene[scene_id] += 1
        ground_truth = meta.get("ground_truth") or {}
        object_results = result.get("objects")
        if not object_results:
            # Older replay helpers may still return the flat shot-level shape.
            object_results = [{
                "id": result.get("id", ""),
                "target": result.get("target", ""),
                "ground_truth": ground_truth,
                "results": result.get("results", []),
            }]
        objects_evaluated += len(object_results)
        objects_per_scene[scene_id] = (
            objects_per_scene.get(scene_id, 0) + len(object_results))
        for object_result in object_results:
            object_ground_truth = object_result.get("ground_truth", ground_truth)
            for entry in object_result.get("results", []):
                model = entry.get("model") or "unknown"
                rows.append(llm_trials.vision_row(
                    object_result, entry, model,
                    meta.get("run_id") or "captures", args.tolerance,
                    ground_truth=object_ground_truth,
                    bearing_tolerance=args.bearing_tolerance))

    if not rows:
        _log("[eval] finished %s shots=%d scenes=%d objects=%d rows=0"
             % (_pipeline_status(pipeline, config, detector_model,
                                 depth_model),
                len(shots), len(scene_order), objects_evaluated))
        print("No model rows produced for %s" % pipeline, file=sys.stderr)
        return None
    llm_trials.write_csv(output_csv, rows, llm_trials.VISION_FIELDS)
    os.makedirs(os.path.dirname(os.path.abspath(results_json)), exist_ok=True)
    with open(results_json, "w") as handle:
        for result in results:
            handle.write(json.dumps(result) + "\n")

    summaries = llm_trials.aggregate(
        rows, args.tolerance, args.bearing_tolerance)
    manifest = {
        "kind": "vision_eval",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "captures_dir": root,
        # `captures` and `capture_order` are retained for older tooling; each
        # entry is now one shot, which may be one of several in a scene.
        "captures": len(shots),
        "capture_order": [os.path.relpath(path, root) for path in shots],
        "shots": len(shots),
        "shot_order": [os.path.relpath(path, root) for path in shots],
        "scenes": len(scene_order),
        "scene_order": scene_order,
        "shots_per_scene": shots_per_scene,
        "objects": objects_evaluated,
        "objects_per_scene": objects_per_scene,
        "shuffle_seed": args.seed,
        "rows": len(rows),
        "pipeline": pipeline,
        "models": sorted({row["model"] for row in rows}),
        "distance_tolerance": args.tolerance,
        "bearing_tolerance": args.bearing_tolerance,
        "config_path": config.get("config_path"),
        "trial_csv": output_csv,
        "results_json": results_json,
        "summary_json": summary_json,
        "summary": summaries,
    }
    os.makedirs(os.path.dirname(os.path.abspath(summary_json)), exist_ok=True)
    with open(summary_json, "w") as handle:
        json.dump(manifest, handle, indent=2)

    _log("[eval] finished %s shots=%d scenes=%d objects=%d rows=%d csv=%s"
         % (_pipeline_status(pipeline, config, detector_model, depth_model),
            len(shots), len(scene_order), objects_evaluated, len(rows),
            output_csv))
    return manifest


class _SigintInterrupted(BaseException):
    """Raised by the SIGINT handler so the run can clean up before returning.

    This deliberately derives from BaseException, not Exception: per-shot and
    per-pipeline error handlers catch Exception so one bad capture cannot stop
    the batch, and SIGINT must bypass those handlers to reach main().
    """


def _sigint_handler(signum, frame):
    """Turn Ctrl+C into a normal cleanup path instead of an abrupt exit."""
    raise _SigintInterrupted()


class _ArtifactTracker:
    """Snapshot output paths so an interrupted run leaves no new artifacts."""

    def __init__(self):
        self._snapshots = {}

    def track(self, *paths):
        """Remember whether each output file existed before this run."""
        for path in paths:
            absolute = os.path.abspath(path)
            if absolute in self._snapshots:
                continue
            try:
                with open(absolute, "rb") as handle:
                    self._snapshots[absolute] = (True, handle.read())
            except FileNotFoundError:
                self._snapshots[absolute] = (False, None)
            except OSError:
                # A directory or unreadable path is not ours to remove/restore.
                self._snapshots[absolute] = (True, None)

    def cleanup(self):
        """Delete new artifacts and restore files that existed before the run."""
        removed = 0
        restored = 0
        for path, (existed, data) in self._snapshots.items():
            try:
                if existed:
                    if data is not None:
                        directory = os.path.dirname(path)
                        if directory:
                            os.makedirs(directory, exist_ok=True)
                        with open(path, "wb") as handle:
                            handle.write(data)
                        restored += 1
                elif os.path.exists(path):
                    os.remove(path)
                    removed += 1
            except OSError as error:
                print("Could not clean partial artifact %s: %s"
                      % (path, error), file=sys.stderr)
        self._snapshots.clear()
        return removed, restored

    def commit(self):
        """Forget snapshots after a successful run; keep the written outputs."""
        self._snapshots.clear()


def _run(args, artifacts):
    """Evaluate the requested pipelines and return the process exit code."""
    config = vision_config.load_config(args.config)
    pipelines = _requested_pipelines(args.pipeline)
    all_pipelines = len(pipelines) > 1
    if args.captures_dir:
        root = os.path.abspath(os.path.expanduser(args.captures_dir))
    elif args.capture:
        root = _capture_root(args.capture)
    else:
        root = vision_config.default_capture_dir(pilot=args.pilot)
    # Pilot outputs stay inside the pilot tree; normal outputs stay beside the
    # captures tree so they share one data/vision directory.
    output_root = root if args.pilot else os.path.dirname(root.rstrip(os.sep))

    seed = args.seed if args.seed is not None else secrets.randbelow(2 ** 32)
    args.seed = seed
    rng = random.Random(seed)
    shots = list(iter_captures(root, args.run_id, args.capture, rng=rng))
    if args.limit:
        shots = shots[:args.limit]
    if not shots:
        print("No shots found under %s" % root, file=sys.stderr)
        return 1
    print("Processing %d shot(s) in randomized scene/shot order (seed %d)"
          % (len(shots), seed))

    manifests = []
    failed = []
    unloads = {}
    for pipeline in pipelines:
        outputs = _pipeline_outputs(args, output_root, pipeline, all_pipelines)
        artifacts.track(*outputs)
        configured = False
        run_error = None
        try:
            _configure_pipeline(pipeline, config)
            configured = True
            manifest = _evaluate_pipeline(
                shots, root, config, args, pipeline, outputs)
        except BaseException as error:
            run_error = error
            raise
        finally:
            # Tear this pipeline's host models down before the next pipeline
            # loads its own, even if evaluation of this one failed. Do not
            # attempt an unload when configuration never reached the service,
            # and do not let an unload error hide the original failure (or a
            # SIGINT that main() still needs to clean up for).
            if configured:
                try:
                    unloads[pipeline] = _unload_pipeline(pipeline, config)
                except Exception as error:
                    print("Failed to unload host model(s) for %s: %s"
                          % (pipeline, error), file=sys.stderr)
                    if run_error is None:
                        raise
                else:
                    print("Unloaded host model(s) for %s" % pipeline)
        if manifest is None:
            failed.append(pipeline)
            continue
        manifests.append(manifest)

    if not manifests:
        return 1

    if all_pipelines:
        all_rows = []
        fields = None
        for manifest in manifests:
            rows, row_fields = llm_trials.read_csv(manifest["trial_csv"])
            fields = fields or row_fields
            all_rows.extend(rows)
        combined_csv = _all_trials_output(args, output_root)
        all_summary = _all_summary_output(args, output_root)
        artifacts.track(combined_csv, all_summary)
        llm_trials.write_csv(
            combined_csv, all_rows, fields or llm_trials.VISION_FIELDS)
        combined = {
            "kind": "vision_eval_all",
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "captures_dir": root,
            "captures": len(shots),
            "shots": len(shots),
            "shot_order": manifests[0]["shot_order"],
            "scenes": manifests[0]["scenes"],
            "scene_order": manifests[0]["scene_order"],
            "shuffle_seed": seed,
            "pipelines": [manifest["pipeline"] for manifest in manifests],
            "failed_pipelines": failed,
            "unloads": unloads,
            "trial_csv": combined_csv,
            "rows": len(all_rows),
            "runs": {
                manifest["pipeline"]: {
                    "trial_csv": manifest["trial_csv"],
                    "results_json": manifest["results_json"],
                    "summary_json": manifest["summary_json"],
                    "rows": manifest["rows"],
                    "objects": manifest["objects"],
                    "models": manifest["models"],
                }
                for manifest in manifests
            },
        }
        os.makedirs(os.path.dirname(os.path.abspath(all_summary)), exist_ok=True)
        with open(all_summary, "w") as handle:
            json.dump(combined, handle, indent=2)
        print("All-pipeline CSV and manifest written to %s and %s"
              % (combined_csv, all_summary))

    # The run has finished writing everything; Ctrl+C during summary printing
    # should not remove completed artifacts.
    artifacts.commit()
    if args.json:
        if all_pipelines:
            print(json.dumps({
                manifest["pipeline"]: manifest["summary"]
                for manifest in manifests
            }, indent=2))
        else:
            llm_trials.print_summaries(manifests[0]["summary"], as_json=True)
    else:
        for manifest in manifests:
            if all_pipelines:
                print("\n=== pipeline: %s ===" % manifest["pipeline"])
            llm_trials.print_summaries(manifest["summary"])
    return 1 if failed else 0


def main(argv=None):
    """Replay every saved shot through one or all vision pipelines."""
    args = _build_parser().parse_args(argv)
    artifacts = _ArtifactTracker()
    previous_handler = None
    try:
        previous_handler = signal.signal(signal.SIGINT, _sigint_handler)
    except (OSError, ValueError):  # not running in the main thread
        pass
    try:
        try:
            return _run(args, artifacts)
        except (KeyboardInterrupt, _SigintInterrupted):
            removed, restored = artifacts.cleanup()
            print("SIGINT received; removed %d new artifact file(s), "
                  "restored %d existing file(s)"
                  % (removed, restored), file=sys.stderr)
            return 130
    finally:
        if previous_handler is not None:
            try:
                signal.signal(signal.SIGINT, previous_handler)
            except (OSError, ValueError):
                pass


if __name__ == "__main__":
    sys.exit(main())
