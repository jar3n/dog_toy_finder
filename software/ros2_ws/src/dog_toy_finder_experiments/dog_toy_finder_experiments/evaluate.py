"""Evaluate one or more toy-search CSV logs and print performance metrics.

Usage:
    python3 -m dog_toy_finder_experiments.evaluate /tmp/run1.csv
    python3 -m dog_toy_finder_experiments.evaluate /tmp/run1.csv /tmp/run2.csv
"""

import csv
import glob
import os
import sys


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def metrics(rows):
    found_rows = [r for r in rows if r.get("found") == "True"]
    found = len(found_rows) > 0
    time_to_find = float(found_rows[0]["t_sec"]) if found else None
    steps = len(rows)
    steps_to_find = int(found_rows[0]["step"]) if found else None
    path = 0.0
    for i in range(1, len(rows)):
        dx = float(rows[i]["x"]) - float(rows[i - 1]["x"])
        dy = float(rows[i]["y"]) - float(rows[i - 1]["y"])
        path += (dx * dx + dy * dy) ** 0.5
    vision_calls = len(rows)
    vision_time = sum(float(r["vision_seconds"]) for r in rows
                      if r.get("vision_seconds") not in ("", None))
    avg_vision = vision_time / vision_calls if vision_calls else 0.0
    visible_count = sum(1 for r in rows if r.get("target_visible") == "True")
    return {
        "found": found,
        "time_to_find_s": time_to_find,
        "steps": steps,
        "steps_to_find": steps_to_find,
        "path_m": round(path, 3),
        "vision_calls": vision_calls,
        "vision_time_s": round(vision_time, 2),
        "avg_vision_s": round(avg_vision, 2),
        "target_visible_rate": round(visible_count / steps, 3) if steps else 0.0,
        "final_x": float(rows[-1]["x"]) if rows else 0.0,
        "final_y": float(rows[-1]["y"]) if rows else 0.0,
    }


def print_metrics(path, m):
    print(f"\n--- {os.path.basename(path)} ---")
    for key, value in m.items():
        print(f"  {key}: {value}")


def main(argv=None):
    args = argv if argv is not None else sys.argv[1:]
    paths = []
    for arg in args:
        paths.extend(glob.glob(arg))
    if not paths:
        print("Usage: evaluate_runs <csv> [csv ...]")
        return 1
    all_metrics = []
    for path in paths:
        rows = load(path)
        m = metrics(rows)
        all_metrics.append(m)
        print_metrics(path, m)
    if len(all_metrics) > 1:
        found_rate = sum(1 for m in all_metrics if m["found"]) / len(all_metrics)
        times = [m["time_to_find_s"] for m in all_metrics if m["time_to_find_s"] is not None]
        avg_time = sum(times) / len(times) if times else float("nan")
        print("\n===== AGGREGATE =====")
        print(f"  runs:            {len(all_metrics)}")
        print(f"  success rate:    {found_rate:.2f}")
        print(f"  mean time (s):   {avg_time:.2f}")
        print(f"  mean path (m):   {sum(m['path_m'] for m in all_metrics)/len(all_metrics):.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
