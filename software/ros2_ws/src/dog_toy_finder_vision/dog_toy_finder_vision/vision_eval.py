"""Offline batch evaluation of saved rectified stereo captures.

Captures are written first by ``vision_capture``; this tool replays every saved
pair through the selected vision pipeline, then writes a trial CSV that already
contains the operator's ground truth.

No live camera topics and no ROS graph are needed. Only the shared detector
config, the saved JPEGs, and the host services (detector, plus the depth service
for ``stereo-net``) are used.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_eval \
        --captures-dir /workspace/data/vision/captures \
        --pipeline open_vocab

Captures are processed in shuffled order. Use ``--seed`` to reproduce an
order; the generated seed and the resulting capture order are recorded in the
summary manifest.

Outputs (defaults next to the captures directory)::

    vision_eval_trials.csv     one scored row per capture and model name
    vision_eval_results.jsonl  full /vision/result-style entries for inspection
    vision_eval_summary.json   aggregate metrics, pipeline, shuffle seed/order
"""

import argparse
import json
import os
import random
import secrets
import sys
from datetime import datetime

from . import vision_config, vision_pipelines

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


def iter_captures(root, run_id=None, capture=None, rng=None):
    """Yield complete capture directories, optionally shuffled by ``rng``.

    Discovery is sorted so the set of captures is deterministic; ``main``
    supplies a seeded random generator so the *processing order* is random but
    reproducible. Callers that omit ``rng`` keep the sorted order (useful for
    tests and tooling that needs a stable listing).
    """
    if capture:
        paths = [os.path.abspath(os.path.expanduser(capture))]
    else:
        paths = []
        for current, directories, files in os.walk(root):
            directories.sort()
            if "meta.json" in files:
                paths.append(current)
    paths = sorted(paths)
    if rng is not None and len(paths) > 1:
        rng.shuffle(paths)
    for path in paths:
        if not all(os.path.isfile(os.path.join(path, name))
                   for name in ("left.jpg", "right.jpg", "meta.json")):
            continue
        if run_id and os.path.basename(os.path.dirname(path)) != run_id:
            continue
        yield path


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
    """Replay one capture and return a /vision/result-style dict."""
    meta, images = load_capture(path)
    name = vision_pipelines.pipeline_name(pipeline, config)
    run_id = str(meta.get("run_id") or "captures")
    capture_id = str(meta.get("capture_id") or os.path.basename(path))
    target = str(meta.get("target") or "")
    result = {
        "id": capture_id,
        "target": target,
        "pipeline": name,
        "models": [],
        "stereo": True,
        "image_source": "rectified_color",
        "image": os.path.join(run_id, capture_id, "left.jpg"),
        "capture_dir": path,
    }
    entries = vision_pipelines.run_pipeline(
        name, target, images["left_jpeg"], images["right_jpeg"],
        config, client=client, depth_client=depth_client)
    result["results"] = entries
    result["models"] = [entry.get("model", "unknown") for entry in entries]
    return result, meta


def _default_output(directory, name):
    return os.path.join(directory, name)


def _build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures-dir", default=None,
                        help="capture root (default: data/vision/captures)")
    parser.add_argument("--capture", default=None,
                        help="evaluate a single capture directory")
    parser.add_argument("--run-id", default=None,
                        help="only evaluate captures from this run id")
    parser.add_argument("--config", default=None, help="detector YAML path")
    parser.add_argument("--pipeline", default=None,
                        help="vision pipeline: open_vocab|stereo-net (default: "
                             "$VISION_PIPELINE / detector YAML / open_vocab)")
    parser.add_argument("--tolerance", type=float, default=0.25,
                        help="distance tolerance in metres")
    parser.add_argument("--bearing-tolerance", type=float, default=5.0,
                        help="lateral bearing tolerance in degrees")
    parser.add_argument("--limit", type=int, default=0,
                        help="stop after this many captures (0 = all)")
    parser.add_argument("--seed", type=int, default=None,
                        help="capture shuffle seed (default: random; recorded "
                             "in the summary manifest)")
    parser.add_argument("--output-csv", default=None, help="trial CSV path")
    parser.add_argument("--results-json", default=None, help="full results JSONL")
    parser.add_argument("--summary-json", default=None, help="aggregate JSON path")
    parser.add_argument("--json", action="store_true",
                        help="print the per-model summary as JSON")
    return parser


def main(argv=None):
    """Replay all saved captures and write scored comparison trials."""
    args = _build_parser().parse_args(argv)
    config = vision_config.load_config(args.config)
    pipeline = vision_pipelines.pipeline_name(args.pipeline, config)
    if args.captures_dir:
        root = os.path.abspath(os.path.expanduser(args.captures_dir))
    elif args.capture:
        root = os.path.dirname(os.path.dirname(
            os.path.abspath(os.path.expanduser(args.capture))))
    else:
        root = vision_config.default_capture_dir()
    output_root = os.path.dirname(root.rstrip(os.sep))
    output_csv = args.output_csv or _default_output(
        output_root, "vision_eval_trials.csv")
    results_json = args.results_json or _default_output(
        output_root, "vision_eval_results.jsonl")
    summary_json = args.summary_json or _default_output(
        output_root, "vision_eval_summary.json")

    seed = args.seed if args.seed is not None else secrets.randbelow(2 ** 32)
    rng = random.Random(seed)
    captures = list(iter_captures(root, args.run_id, args.capture, rng=rng))
    if args.limit:
        captures = captures[:args.limit]
    if not captures:
        print("No captures found under %s" % root, file=sys.stderr)
        return 1
    capture_order = [os.path.relpath(path, root) for path in captures]
    print("Processing %d capture(s) in shuffled order (seed %d)"
          % (len(captures), seed))

    rows = []
    results = []
    for path in captures:
        try:
            result, meta = evaluate_capture(path, config, pipeline)
        except Exception as error:
            print("Failed to replay %s: %s" % (path, error), file=sys.stderr)
            continue
        results.append(result)
        ground_truth = meta.get("ground_truth") or {}
        for entry in result["results"]:
            model = entry.get("model") or "unknown"
            rows.append(llm_trials.vision_row(
                result, entry, model, meta.get("run_id") or "captures",
                args.tolerance, ground_truth=ground_truth,
                bearing_tolerance=args.bearing_tolerance))

    if not rows:
        print("No model rows produced", file=sys.stderr)
        return 1
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
        "captures": len(captures),
        "capture_order": capture_order,
        "shuffle_seed": seed,
        "rows": len(rows),
        "pipeline": pipeline,
        "models": sorted({row["model"] for row in rows}),
        "distance_tolerance": args.tolerance,
        "bearing_tolerance": args.bearing_tolerance,
        "config_path": config.get("config_path"),
        "trial_csv": output_csv,
        "results_json": results_json,
        "summary": summaries,
    }
    os.makedirs(os.path.dirname(os.path.abspath(summary_json)), exist_ok=True)
    with open(summary_json, "w") as handle:
        json.dump(manifest, handle, indent=2)

    print("Evaluated %d capture(s) with %s, wrote %d row(s) to %s"
          % (len(captures), pipeline, len(rows), output_csv))
    llm_trials.print_summaries(summaries, as_json=args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
