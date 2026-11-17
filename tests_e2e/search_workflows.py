"""End-to-end search workflow tests (fake robot + scripted vision).

Covers the autonomous (single-target) and find-all demo workflows:

  SCENARIO=size        search stops when the toy looks medium (homing path)
  SCENARIO=distance    search stops on VLM distance (<= found_distance)
  SCENARIO=notfound    toy never appears: full coverage, found=False
  SCENARIO=findall     two distinct toys found, search keeps sweeping
  SCENARIO=multitarget two target kinds; only one is ever visible

Run one scenario per process (each builds a fresh ROS world):

    SCENARIO=size python3 tests_e2e/search_workflows.py

or all of them (see tests_e2e/README.md). Exit code 0 means every check
passed; each check prints PASS/FAIL.
"""

import csv
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.executors import SingleThreadedExecutor

from fakes import FakeRobot, FakeVision, FakeVisionPerTarget


def check(name, cond):
    print(("PASS" if cond else "FAIL"), name, flush=True)
    if not cond:
        raise SystemExit(1)


SCENARIOS = {
    "size": {
        "answers": [
            {"target_visible": False},
            {"target_visible": True, "position": "center", "size": "small",
             "distance_m": 1.5},
            {"target_visible": True, "position": "center", "size": "medium",
             "distance_m": 0.5},
        ],
        "find_all": False,
    },
    "distance": {
        "answers": [
            {"target_visible": False},
            {"target_visible": True, "position": "left", "size": "small",
             "distance_m": 0.3},
        ],
        "find_all": False,
    },
    "notfound": {
        "answers": [{"target_visible": False}],
        "find_all": False,
    },
    "findall": {
        "answers": [
            {"target_visible": False},
            {"target_visible": True, "position": "left", "size": "small",
             "distance_m": 0.6},
            {"target_visible": True, "position": "left", "size": "small",
             "distance_m": 0.3},
            {"target_visible": False},
            {"target_visible": True, "position": "right", "size": "medium",
             "distance_m": 1.0},
            {"target_visible": False},
            {"target_visible": False},
            {"target_visible": False},
            {"target_visible": False},
        ],
        "find_all": True,
    },
    "multitarget": {
        # Blue bone is visible on its second ask only, then never again
        # (the last script entry repeats).
        "per_target": {
            "red ball": [{"target_visible": False}],
            "blue bone": [
                {"target_visible": False},
                {"target_visible": True, "position": "center",
                 "size": "medium", "distance_m": 0.5},
                {"target_visible": False},
            ],
        },
        "find_all": True,
        "target": "red ball,blue bone",
    },
}


def run_scenario(name):
    spec = SCENARIOS[name]
    csv_path = "/tmp/search_e2e_%s.csv" % name
    if os.path.exists(csv_path):
        os.remove(csv_path)

    from dog_toy_finder_planning.autonomous_search_node import AutonomousSearch

    rclpy.init()
    robot = FakeRobot()
    if "per_target" in spec:
        vision = FakeVisionPerTarget(spec["per_target"])
    else:
        vision = FakeVision(spec["answers"])
    node = AutonomousSearch()
    node.targets = [t.strip() for t in str(
        spec.get("target", node.target)).split(",") if t.strip()]
    node.target = spec.get("target", node.target)
    node.side = 0.2
    node.side_increment = 0.0
    node.loops = 1
    node.max_time = 90.0
    node.vision_timeout = 3.0
    node.output_csv = csv_path
    node.run_id = "e2e_%s" % name
    node.found_distance = 0.4
    node.find_all = spec["find_all"]
    node.toy_side, node.toy_forward, node.toy_merge = 0.25, 0.6, 0.15
    node.escape_distance = 0.3

    executor = SingleThreadedExecutor()
    for n in (robot, vision, node):
        executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.5)

    t0 = time.time()
    node.run()
    elapsed = time.time() - t0

    rows = list(csv.DictReader(open(csv_path)))
    check("CSV written with header", len(rows) >= 1)
    check("run finished quickly (%.1fs < 90s)" % elapsed, elapsed < 90.0)
    found_rows = sum(1 for r in rows if r["found"] == "True")
    if name in ("size", "distance"):
        check("found=True", node.found is True)
        check("found_step set", node.found_step is not None)
        check("exactly one CSV found row", found_rows == 1)
        check("search stopped early (rows=%d <= 3)" % len(rows), len(rows) <= 3)
    elif name == "notfound":
        check("found=False", node.found is False)
        check("completed all 4 sides", len(rows) == 4)
        check("CSV all found=False", found_rows == 0)
    elif name == "findall":
        check("completed all 4 sides (no early exit)", len(rows) >= 4)
        check("found 2 distinct toys", len(node.toys) == 2)
        check("single-target found stays False in find_all", node.found is False)
        check("two CSV rows marked found", found_rows == 2)
        check("toys at distinct positions",
              math.hypot(node.toys[0][0] - node.toys[1][0],
                         node.toys[0][1] - node.toys[1][1]) > 0.15)
    elif name == "multitarget":
        check("targets parsed",
              node.targets == ["red ball", "blue bone"])
        check("completed all 4 sides", len(rows) >= 4)
        check("exactly 1 toy found", len(node.toys) == 1)
        check("toy labelled blue bone", node.toys[0][2] == "blue bone")
        check("one CSV found row", found_rows == 1)
    check("first row near origin (after one side)",
          abs(float(rows[0]["x"])) <= 0.45 and abs(float(rows[0]["y"])) <= 0.45)
    print("PASS scenario %s" % name, flush=True)
    executor.shutdown()
    os._exit(0)


if __name__ == "__main__":
    scenario = os.environ.get("SCENARIO", "")
    if scenario not in SCENARIOS:
        print("Set SCENARIO to one of: %s" % ", ".join(SCENARIOS), flush=True)
        raise SystemExit(2)
    run_scenario(scenario)
