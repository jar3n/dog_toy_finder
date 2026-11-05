"""Dataset utilities: summarize a collected dataset directory.

Usage:
    python3 -m dog_toy_finder_data.dataset_tools /tmp/dog_toy_dataset/run1
"""

import csv
import json
import os
import sys


def summarize(run_dir):
    summary = {"run_dir": run_dir}
    # Poses
    poses_path = os.path.join(run_dir, "poses.csv")
    if os.path.exists(poses_path):
        with open(poses_path) as f:
            rows = list(csv.DictReader(f))
        summary["pose_samples"] = len(rows)
        if rows:
            xs = [float(r["x"]) for r in rows]
            ys = [float(r["y"]) for r in rows]
            summary["x_range"] = (min(xs), max(xs))
            summary["y_range"] = (min(ys), max(ys))
            path = 0.0
            for i in range(1, len(rows)):
                dx = float(rows[i]["x"]) - float(rows[i - 1]["x"])
                dy = float(rows[i]["y"]) - float(rows[i - 1]["y"])
                path += (dx * dx + dy * dy) ** 0.5
            summary["path_m"] = round(path, 3)
    # Vision
    vision_path = os.path.join(run_dir, "vision.jsonl")
    if os.path.exists(vision_path):
        with open(vision_path) as f:
            vrows = [json.loads(l) for l in f if l.strip()]
        summary["vision_calls"] = len(vrows)
        visible = sum(1 for v in vrows if any(
            r.get("target_visible") for r in v.get("results", [v])))
        summary["target_visible_calls"] = visible
    # Images
    img_dir = os.path.join(run_dir, "images")
    if os.path.isdir(img_dir):
        files = os.listdir(img_dir)
        summary["image_files"] = len(files)
        summary["left_frames"] = sum(1 for f in files if f.startswith("left_"))
        summary["right_frames"] = sum(1 for f in files if f.startswith("right_"))
    # Manifest
    manifest_path = os.path.join(run_dir, "manifest.json")
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            m = json.load(f)
        summary["started"] = m.get("started")
        summary["finished"] = m.get("finished")
        summary["recorded_frames"] = m.get("frames")
    return summary


def main(argv=None):
    args = argv if argv is not None else sys.argv[1:]
    if not args:
        print("Usage: dataset_summary <run_dir> [run_dir ...]")
        return 1
    for d in args:
        s = summarize(d)
        print(json.dumps(s, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())