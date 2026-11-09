"""ROSA run_search tool tests (fake robot + scripted vision).

Stubs the langchain_core `tool` decorator so the tool functions import
without the full LangChain/Rosa stack, then drives the real RobotSession
and the real autonomous search node through the tool.

  MODE=one    single toy, stops after the first side (~1 min incl. startup)
  MODE=all    two scripted toys, keeps sweeping (~2-3 min)
  MODE=multi  two target kinds; only one is ever visible (~2-3 min)

    MODE=one ROSA_REQUIRE_APPROVAL=0 python3 tests_e2e/rosa_search.py
"""

import os
import sys
import threading
import time
import types

# Stub langchain_core.tools.tool before importing the rosa tools.
lc = types.ModuleType("langchain_core")
tools_mod = types.ModuleType("langchain_core.tools")
tools_mod.tool = lambda fn: fn
lc.tools = tools_mod
sys.modules["langchain_core"] = lc
sys.modules["langchain_core.tools"] = tools_mod

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.executors import SingleThreadedExecutor

from fakes import FakeRobot, FakeVision, FakeVisionPerTarget

MODE = os.environ.get("MODE", "one")

if MODE == "one":
    ANSWERS = [
        {"target_visible": False},
        {"target_visible": True, "position": "center", "size": "medium",
         "distance_m": 0.5},
    ]
elif MODE == "all":
    ANSWERS = [
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
    ]
elif MODE == "multi":
    ANSWERS = []
else:
    ANSWERS = []


def check(name, cond):
    print(("PASS" if cond else "FAIL"), name, flush=True)
    if not cond:
        raise SystemExit(1)


sys.path.insert(0, "/workspace/software/rosa_agent")
os.environ["ROSA_REQUIRE_APPROVAL"] = "0"

from rosa_agent.robot import get_robot
from rosa_agent import tools

rclpy.init()
robot_driver = FakeRobot()
if MODE == "multi":
    vision = FakeVisionPerTarget({
        "red ball": [{"target_visible": False}],
        "blue bone": [
            {"target_visible": False},
            {"target_visible": True, "position": "center", "size": "medium",
             "distance_m": 0.5},
        ],
    })
else:
    vision = FakeVision(ANSWERS)
ex = SingleThreadedExecutor()
ex.add_node(robot_driver)
ex.add_node(vision)
threading.Thread(target=ex.spin, daemon=True).start()
time.sleep(0.5)

session = get_robot()
check("robot session ready", session.ready(timeout=8.0))

# Invalid requests never move the robot.
check("invalid mode rejected",
      "Invalid search" in tools.run_search("red ball", "bogus"))
check("empty target rejected",
      "Invalid search" in tools.run_search("", "one"))

target = "red ball,blue bone" if MODE == "multi" else "red ball"
t0 = time.time()
out = tools.run_search(target, MODE)
elapsed = time.time() - t0
print("run_search %s ->" % MODE, out[:240], flush=True)

if MODE == "one":
    check("one mode reported 1 toy", "Toys found: 1" in out)
    check("one mode stopped fast < 60s (%.1f)" % elapsed, elapsed < 60.0)
elif MODE == "all":
    check("find_all reported 2 toys", "Toys found: 2" in out)
    check("find_all finished < 180s (%.1f)" % elapsed, elapsed < 180.0)
elif MODE == "multi":
    check("multi-target found 1 toy", "Toys found: 1" in out)
    check("toy labelled blue bone", "(blue bone)" in out)
    check("multi finished < 180s (%.1f)" % elapsed, elapsed < 180.0)

print("PASS ROSA run_search scenario (%s)" % MODE, flush=True)
ex.shutdown()
os._exit(0)
