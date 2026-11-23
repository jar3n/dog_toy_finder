"""Offline batch evaluation of saved rectified stereo captures.

Captures are written first by ``vision_capture``; this tool replays every saved
pair through the classical detector and the configured Ollama vision models,
then writes a trial CSV that already contains the operator's ground truth.

No live camera topics and no ROS graph are needed. Only the shared detector
config, the saved JPEGs, and (for ML rows) a reachable Ollama server are used.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_eval \
        --captures-dir /workspace/data/vision/captures \
        --models gemma4,minicpm-v4.5

Outputs (defaults next to the captures directory)::

    vision_eval_trials.csv     one scored row per capture and detector/model
    vision_eval_results.jsonl  full /vision/result-style entries for inspection
    vision_eval_summary.json   per-model aggregate metrics
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
from PIL import Image as PilImage

from . import classical_detector, ollama_vlm, vision_answer, vision_config

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


def iter_captures(root, run_id=None, capture=None):
    """Yield capture directories that contain left.jpg, right.jpg, meta.json."""
    if capture:
        paths = [os.path.abspath(os.path.expanduser(capture))]
    else:
        paths = []
        for current, directories, files in os.walk(root):
            directories.sort()
            if "meta.json" in files:
                paths.append(current)
    for path in sorted(paths):
        if not all(os.path.isfile(os.path.join(path, name))
                   for name in ("left.jpg", "right.jpg", "meta.json")):
            continue
        if run_id and os.path.basename(os.path.dirname(path)) != run_id:
            continue
        yield path


def load_capture(path):
    """Load a capture's metadata, RGB arrays, and JPEG bytes."""
    with open(os.path.join(path, "meta.json")) as handle:
        meta = json.load(handle)
    images = {}
    for eye in ("left", "right"):
        jpeg_path = os.path.join(path, "%s.jpg" % eye)
        with open(jpeg_path, "rb") as handle:
            images[eye + "_jpeg"] = handle.read()
        with PilImage.open(jpeg_path) as image:
            images[eye + "_rgb"] = np.asarray(image.convert("RGB"))
    return meta, images


def evaluate_capture(path, config, models, classical, chat_url,
                     compare_models, timeout, include_ml=True,
                     generate_url=None):
    """Replay one capture and return a /vision/result-style dict."""
    meta, images = load_capture(path)
    run_id = str(meta.get("run_id") or "captures")
    capture_id = str(meta.get("capture_id") or os.path.basename(path))
    target = str(meta.get("target") or "")
    result = {
        "id": capture_id,
        "target": target,
        "models": [],
        "stereo": True,
        "image_source": "rectified_color",
        "image": os.path.join(run_id, capture_id, "left.jpg"),
        "capture_dir": path,
    }
    entries = []
    if classical:
        start = time.time()
        detection = classical_detector.detect(
            images["left_rgb"], images["right_rgb"], target, config)
        detection["model"] = "classical"
        detection["seconds"] = round(time.time() - start, 3)
        entries.append(vision_answer.tidy(detection, config))

    if include_ml:
        prompt = vision_answer.prompt_for(target, config)
        jpegs = [images["left_jpeg"], images["right_jpeg"]]
        for model in models:
            try:
                entries.append(ollama_vlm.ask_model(
                    model, jpegs, target, prompt, chat_url,
                    compare_models=compare_models, timeout=timeout,
                    config=config))
            finally:
                if compare_models:
                    ollama_vlm.unload_model(
                        model, generate_url or (
                            chat_url.rstrip("/").rsplit("/", 1)[0]
                            + "/api/generate"), timeout=15)
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
    parser.add_argument("--models", default=None,
                        help="comma-separated Ollama models; empty skips ML")
    parser.add_argument("--ollama-url", default=None,
                        help="Ollama endpoint (default: $OLLAMA_URL or localhost)")
    parser.add_argument("--no-classical", action="store_true",
                        help="skip the deterministic detector")
    parser.add_argument("--no-ml", action="store_true",
                        help="skip Ollama models (classical-only replay)")
    parser.add_argument("--compare-models", action="store_true",
                        help="unload each model between requests, as live compare mode")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="Ollama request timeout in seconds")
    parser.add_argument("--tolerance", type=float, default=0.25,
                        help="distance tolerance in metres")
    parser.add_argument("--bearing-tolerance", type=float, default=5.0,
                        help="lateral bearing tolerance in degrees")
    parser.add_argument("--limit", type=int, default=0,
                        help="stop after this many captures (0 = all)")
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

    if args.models is None:
        models_text = os.environ.get("VISION_MODELS") \
            or os.environ.get("VISION_MODEL") or "gemma4"
    else:
        models_text = args.models
    models = [m.strip() for m in models_text.split(",") if m.strip()]
    base_url = (args.ollama_url or os.environ.get(
        "OLLAMA_URL", "http://localhost:11434")).rstrip("/")

    # Multi-model comparisons unload each model after its answer, matching the
    # live node's cold-load policy; a single model stays warm unless forced.
    compare_models = args.compare_models or len(models) > 1

    captures = list(iter_captures(root, args.run_id, args.capture))
    if args.limit:
        captures = captures[:args.limit]
    if not captures:
        print("No captures found under %s" % root, file=sys.stderr)
        return 1

    rows = []
    results = []
    for path in captures:
        try:
            result, meta = evaluate_capture(
                path, config, models, not args.no_classical,
                base_url + "/api/chat", compare_models,
                args.timeout, include_ml=not args.no_ml and bool(models),
                generate_url=base_url + "/api/generate")
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
        print("No detector/model rows produced", file=sys.stderr)
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
        "rows": len(rows),
        "models": models if not args.no_ml else [],
        "classical": not args.no_classical,
        "compare_models": compare_models if not args.no_ml else False,
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

    print("Evaluated %d capture(s), wrote %d row(s) to %s"
          % (len(captures), len(rows), output_csv))
    llm_trials.print_summaries(summaries, as_json=args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
