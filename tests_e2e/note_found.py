"""Fast checks for the found/toy-counting logic (no driving involved).

Covers AutonomousSearch and ToySearchTest _note_found: single-target
latching, find-all counting, position dedupe, and the invisible case.

    python3 tests_e2e/note_found.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy


def check(name, cond):
    print(("PASS" if cond else "FAIL"), name, flush=True)
    if not cond:
        raise SystemExit(1)


FOUND = {"target_visible": True, "position": "center", "size": "medium",
         "distance_m": 0.5}

rclpy.init()

from dog_toy_finder_experiments.toy_search_test import ToySearchTest
from dog_toy_finder_planning.autonomous_search_node import AutonomousSearch

for cls, name in ((AutonomousSearch, "auto"), (ToySearchTest, "toy")):
    node = cls()
    node.pose = lambda key="odom": (0.0, 0.0, 0.0)
    node.found_distance = 0.4
    node.toy_side, node.toy_forward, node.toy_merge = 0.25, 0.6, 0.15

    node.find_all = False
    check(f"{name} single: _note_found True", node._note_found(FOUND) is True)
    check(f"{name} single: found latched", node.found is True)
    check(f"{name} single: toy recorded", len(node.toys) == 1)
    check(f"{name} single: labelled with target",
          node.toys[0][2] == node.target)
    check(f"{name} single: re-report is not new",
          node._note_found(FOUND) is False)

    node2 = cls()
    node2.pose = lambda key="odom": (0.0, 0.0, 0.0)
    node2.found_distance = 0.4
    node2.toy_side, node2.toy_forward, node2.toy_merge = 0.25, 0.6, 0.15
    node2.find_all = True
    check(f"{name} find_all: _note_found True", node2._note_found(FOUND) is True)
    check(f"{name} find_all: found stays False", node2.found is False)
    check(f"{name} find_all: toy recorded", len(node2.toys) == 1)
    check(f"{name} find_all: estimate 0.5 m ahead",
          abs(node2.toys[0][0] - 0.5) < 1e-6
          and abs(node2.toys[0][1]) < 1e-6)
    check(f"{name}: invisible is not found",
          node2._note_found({"target_visible": False}) is False)
    node.destroy_node()
    node2.destroy_node()

print("PASS all _note_found checks", flush=True)
rclpy.try_shutdown()
