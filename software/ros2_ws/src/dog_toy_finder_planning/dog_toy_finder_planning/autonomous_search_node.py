"""Autonomous (non-LLM) search planner for comparison with ROSA-driven planning.

Strategy ("smart search"):
  1. Drive an expanding square (coverage) using closed-loop moves.
  2. After every side, ask the vision node about the target.
  3. If the target is visible, home in: turn toward it and drive, re-checking.
  4. If the path ahead is blocked, avoid by turning and continuing.
  5. Stop when the target is found or the time budget is exhausted.

This deterministic planner is the baseline against which the LLM planner
(ROSA) is compared. Both write the same CSV schema so evaluate_runs can
compare them.

Subscribes  /odometry/filtered   nav_msgs/Odometry
            /vision/result       std_msgs/String (JSON)
Publishes   /vision/ask         std_msgs/String (JSON)
            /cmd_vel            (via Mover)
"""

import csv
import json
import math
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from dog_toy_finder_motion.mover import Mover
from dog_toy_finder_planning.path_planner import expanding_square_waypoints


class AutonomousSearch(Mover):
    def __init__(self):
        super().__init__(node_name="autonomous_search")
        self.declare_parameter("target", "red ball")
        self.declare_parameter("side", 0.5)
        self.declare_parameter("side_increment", 0.5)
        self.declare_parameter("loops", 3)
        self.declare_parameter("max_time", 120.0)
        self.declare_parameter("vision_timeout", 10.0)
        self.declare_parameter("output_csv", "/tmp/autonomous_search.csv")
        self.declare_parameter("run_id", "auto1")
        self.declare_parameter("home_on_visible", True)

        self.target = self.get_parameter("target").value
        self.side = float(self.get_parameter("side").value)
        self.side_increment = float(self.get_parameter("side_increment").value)
        self.loops = int(self.get_parameter("loops").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout").value)
        self.output_csv = self.get_parameter("output_csv").value
        self.run_id = self.get_parameter("run_id").value
        self.home_on_visible = bool(self.get_parameter("home_on_visible").value)

        self.ask_pub = self.create_publisher(String, "/vision/ask", 10)
        self.create_subscription(String, "/vision/result", self.on_vision_result,
                                 qos_profile_sensor_data)

        self._lock = threading.Lock()
        self._vision_result = None
        self._vision_event = threading.Event()

        self.rows = []
        self.found = False
        self.found_step = None
        self.step = 0

    def on_vision_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._vision_result = data
            self._vision_event.set()

    def ask_vision(self):
        self._vision_event.clear()
        with self._lock:
            self._vision_result = None
        req_id = str(int(time.time() * 1000))
        self.ask_pub.publish(String(data=json.dumps(
            {"id": req_id, "target": self.target})))
        if not self._vision_event.wait(timeout=self.vision_timeout):
            return None
        with self._lock:
            return self._vision_result

    def current_pose(self):
        pose = self.pose("odom")
        if pose is None:
            return 0.0, 0.0, 0.0
        return pose

    def log_step(self, action, result, found_now, t0):
        x, y, theta = self.current_pose()
        target_visible = False
        position = "none"
        size = "none"
        obstacle = "none"
        path_clear = True
        vision_seconds = None
        if result is not None:
            target_visible = bool(result.get("target_visible", False))
            position = result.get("position", "none")
            size = result.get("size", "none")
            obstacle = result.get("obstacle_ahead", "none")
            path_clear = bool(result.get("path_clear", True))
            vision_seconds = result.get("seconds")
        self.rows.append({
            "run_id": self.run_id,
            "step": self.step,
            "t_sec": round(time.time() - t0, 3),
            "x": round(x, 4),
            "y": round(y, 4),
            "theta_deg": round(math.degrees(theta), 2),
            "target": self.target,
            "target_visible": target_visible,
            "position": position,
            "size": size,
            "obstacle_ahead": obstacle,
            "path_clear": path_clear,
            "vision_seconds": vision_seconds,
            "found": found_now,
            "action": action,
        })
        self.get_logger().info(
            f"step {self.step} [{action}]: found={found_now} visible={target_visible} "
            f"pos={position} size={size} obstacle={obstacle}")

    def home_on_target(self, result, t0):
        """Turn toward a visible target and drive a short step."""
        if not result or not result.get("target_visible"):
            return False
        pos = result.get("position", "center")
        turn_angle = 0.0
        if pos == "left":
            turn_angle = math.radians(15)
        elif pos == "right":
            turn_angle = math.radians(-15)
        if turn_angle != 0.0:
            self.turn(turn_angle)
        # Drive a short step toward the target.
        size = result.get("size", "none")
        step_dist = 0.2 if size in ("small", "none") else 0.1
        self.straight(step_dist)
        self.step += 1
        # Re-check.
        new_result = self.ask_vision()
        found_now = False
        if new_result is not None:
            tv = new_result.get("target_visible", False)
            sz = new_result.get("size", "none")
            if tv and sz in ("medium", "large"):
                found_now = True
                self.found = True
                self.found_step = self.step
        self.log_step("home", new_result, found_now, t0)
        return found_now

    def avoid_obstacle(self, result):
        """Turn away from a central obstacle."""
        obstacle = (result or {}).get("obstacle_ahead", "none")
        if obstacle and obstacle.lower() not in ("", "none", "nothing", "no", "n/a"):
            # Turn left to skirt the obstacle.
            self.turn(math.radians(30))

    def run(self):
        t0 = time.time()
        self.wait_for_odom()
        side = self.side

        for loop in range(self.loops):
            if time.time() - t0 > self.max_time or self.found:
                break
            for _ in range(4):
                if time.time() - t0 > self.max_time or self.found:
                    break
                # Obstacle check before driving.
                pre = self.ask_vision()
                if pre is not None:
                    self.avoid_obstacle(pre)

                self.straight(side)
                self.step += 1
                if self.failsafe_hit():
                    self.get_logger().warn(
                        f"Pico failsafe interrupted side {self.step}: IR "
                        "obstacle ahead, move aborted early. Continuing search "
                        "from the stopped pose.")
                result = self.ask_vision()
                found_now = False
                if result is not None:
                    tv = result.get("target_visible", False)
                    sz = result.get("size", "none")
                    if tv and sz in ("medium", "large"):
                        found_now = True
                        self.found = True
                        self.found_step = self.step
                    elif tv and self.home_on_visible:
                        # Try to home in on the visible target.
                        if self.home_on_target(result, t0):
                            found_now = True
                self.log_step("side", result, found_now, t0)
                if self.found:
                    self.stop()
                    self._write_csv()
                    self._print_summary(t0)
                    return
                self.turn(math.pi / 2.0)
            side += self.side_increment

        self.stop()
        self._write_csv()
        self._print_summary(t0)

    def _write_csv(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        fieldnames = ["run_id", "step", "t_sec", "x", "y", "theta_deg", "target",
                      "target_visible", "position", "size", "obstacle_ahead",
                      "path_clear", "vision_seconds", "found", "action"]
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)
        self.get_logger().info(f"Wrote {len(self.rows)} rows to {self.output_csv}")

    def _print_summary(self, t0):
        elapsed = time.time() - t0
        print("\n===== AUTONOMOUS SEARCH SUMMARY =====")
        print(f"run_id:        {self.run_id}")
        print(f"target:        {self.target}")
        print(f"found:         {self.found}")
        print(f"found_step:    {self.found_step}")
        print(f"time (s):     {elapsed:.2f}")
        print(f"steps:         {len(self.rows)}")
        if self.rows:
            path = 0.0
            for i in range(1, len(self.rows)):
                path += math.hypot(self.rows[i]["x"] - self.rows[i - 1]["x"],
                                   self.rows[i]["y"] - self.rows[i - 1]["y"])
            print(f"path (m):     {path:.3f}")
            print(f"final pose:   ({self.rows[-1]['x']:.3f}, {self.rows[-1]['y']:.3f})")
        print(f"csv:           {self.output_csv}")
        print("=====================================\n")


def main(args=None):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutonomousSearch()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
        print("\nInterrupted.")
    finally:
        node.stop()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
