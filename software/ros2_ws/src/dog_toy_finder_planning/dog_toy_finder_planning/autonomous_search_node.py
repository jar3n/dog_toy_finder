"""Autonomous (non-LLM) search planner for comparison with ROSA-driven planning.

Strategy ("smart search"):
  1. Drive an expanding square (coverage) using closed-loop moves.
  2. After every side, ask the vision node about the target.
  3. If the target is visible, home in: turn toward it and drive, re-checking.
  4. If the path ahead is blocked, avoid by turning and continuing.
  5. Stop when the target is found or the time budget is exhausted.

With find_all=true (the demo mode), a found toy is recorded and the robot
backs away and keeps searching instead of stopping, until the planned
coverage is complete. Found toys are estimated in the odom frame from the
robot pose plus the VLM position/distance, and deduplicated within
toy_merge_radius metres, so the summary reports every distinct toy found.

This deterministic planner is the baseline against which the LLM planner
(ROSA) is compared. Both write the same CSV schema so evaluate_runs can
compare them.

Subscribes  /odometry/filtered   nav_msgs/Odometry
            /vision/result       std_msgs/String (JSON)
Publishes   /vision/ask         std_msgs/String (JSON)
            /cmd_vel_auto       (via Mover; cmd_vel_mux forwards to the Pico)
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
        self.declare_parameter("found_distance", 0.4)  # m, stop when this close
        self.declare_parameter("find_all", False)      # demo: keep searching
        self.declare_parameter("toy_side_offset", 0.25)  # m left/right estimate
        self.declare_parameter("toy_forward_distance", 0.6)  # fallback m ahead
        self.declare_parameter("toy_merge_radius", 0.15)  # m, same-toy radius
        self.declare_parameter("toy_max_count", 500)
        self.declare_parameter("escape_distance", 0.3)  # m backed up per find

        self.target = self.get_parameter("target").value
        # The target is a comma-separated list of toy descriptions; the
        # search asks the vision node about each one in turn. A single name
        # (the default) searches for every instance of that one kind.
        self.targets = [t.strip() for t in str(self.target).split(",")
                        if t.strip()] or [str(self.target)]
        self.side = float(self.get_parameter("side").value)
        self.side_increment = float(self.get_parameter("side_increment").value)
        self.loops = int(self.get_parameter("loops").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout").value)
        self.output_csv = self.get_parameter("output_csv").value
        self.run_id = self.get_parameter("run_id").value
        self.home_on_visible = bool(self.get_parameter("home_on_visible").value)
        self.found_distance = float(self.get_parameter("found_distance").value)
        self.find_all = bool(self.get_parameter("find_all").value)
        self.toy_side = float(self.get_parameter("toy_side_offset").value)
        self.toy_forward = float(self.get_parameter("toy_forward_distance").value)
        self.toy_merge = float(self.get_parameter("toy_merge_radius").value)
        self.toy_max_count = int(self.get_parameter("toy_max_count").value)
        self.escape_distance = float(self.get_parameter("escape_distance").value)

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
        # [(x, y, target), ...] distinct found toys in the odom frame.
        self.toys = []

    def on_vision_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._vision_result = data
            self._vision_event.set()

    def ask_one(self, target):
        """Ask the vision node about one target and wait for its result."""
        self._vision_event.clear()
        with self._lock:
            self._vision_result = None
        req_id = str(int(time.time() * 1000))
        self.ask_pub.publish(String(data=json.dumps(
            {"id": req_id, "target": target})))
        if not self._vision_event.wait(timeout=self.vision_timeout):
            return None
        with self._lock:
            return self._vision_result

    def ask_vision(self):
        """Ask about every target in turn; return the most useful result.

        A found result wins immediately, then a visible-but-not-found result
        (for homing), then the last not-visible result (for logging).
        """
        seen = None
        for target in self.targets:
            result = self.ask_one(target)
            if result is None:
                continue
            if self._is_found(result):
                return result
            if seen is None:
                seen = result
            elif (self._entry(result).get("target_visible")
                  and not self._entry(seen).get("target_visible")):
                seen = result
        return seen

    def current_pose(self):
        pose = self.pose("odom")
        if pose is None:
            return 0.0, 0.0, 0.0
        return pose

    def _entry(self, result):
        """Return the single-model result dict (handles results[] wrapping)."""
        if not isinstance(result, dict):
            return {}
        if "target_visible" not in result:
            results = result.get("results")
            if isinstance(results, list) and results and isinstance(results[0], dict):
                return results[0]
        return result

    def _distance(self, entry):
        """Return a sane VLM distance in metres, or None when unknown."""
        try:
            distance = float(entry.get("distance_m") or 0.0)
        except (TypeError, ValueError):
            return None
        return distance if distance > 0.0 else None

    def _is_found(self, result):
        """Decide whether the target counts as found.

        Found when the target is visible and either looks medium/large in the
        image or the VLM estimates it within found_distance metres.
        """
        entry = self._entry(result)
        if not entry.get("target_visible"):
            return False
        if entry.get("size") in ("medium", "large"):
            return True
        distance = self._distance(entry)
        return distance is not None and distance <= self.found_distance

    def _record_toy(self, result):
        """Estimate one found toy in the odom frame; True when it is new.

        Uses the same placement as the occupancy mapper: the VLM distance (or
        the forward fallback) ahead, offset sideways for left/right. Estimates
        within toy_merge_radius of an already-recorded toy are the same toy.
        """
        entry = self._entry(result)
        position = str(entry.get("position", "center")).strip().lower()
        sides = {"left": 1.0, "center": 0.0, "right": -1.0}
        if position not in sides:
            return False
        x, y, yaw = self.current_pose()
        ahead = self._distance(entry)
        if ahead is None:
            ahead = self.toy_forward
        side = sides[position] * self.toy_side
        # ROS convention: +x forward, +y left, yaw counter-clockwise.
        tx = x + ahead * math.cos(yaw) - side * math.sin(yaw)
        ty = y + ahead * math.sin(yaw) + side * math.cos(yaw)
        for ox, oy, _label in self.toys:
            if (ox - tx) ** 2 + (oy - ty) ** 2 <= self.toy_merge ** 2:
                return False
        label = str(entry.get("target") or self.target)
        self.toys.append((tx, ty, label))
        if len(self.toys) > self.toy_max_count:
            self.toys = self.toys[-self.toy_max_count:]
        self.get_logger().info(
            f"Toy #{len(self.toys)} ({label}) found at ({tx:.2f}, {ty:.2f})"
            f" in odom [{ahead:.2f} m ahead, {side:+.2f} m lateral]")
        return True

    def _note_found(self, result):
        """Count a found toy; True when a new toy was found this check.

        Single-target mode latches self.found (the run stops). Find-all mode
        records the toy and keeps searching.
        """
        if not self._is_found(result):
            return False
        if not self._record_toy(result):
            return False
        if self.find_all:
            return True
        self.found = True
        self.found_step = self.step
        return True

    def _escape_from_toy(self):
        """Back away from a just-found toy before searching again."""
        self.get_logger().info(
            f"Backing up {self.escape_distance:.2f} m from the toy "
            "before continuing the search.")
        self.straight(-abs(self.escape_distance))

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
        if not result or not self._entry(result).get("target_visible"):
            return False
        entry = self._entry(result)
        pos = entry.get("position", "center")
        turn_angle = 0.0
        if pos == "left":
            turn_angle = math.radians(15)
        elif pos == "right":
            turn_angle = math.radians(-15)
        if turn_angle != 0.0:
            self.turn(turn_angle)
        # Drive a short step toward the target. Shrink the step as the VLM
        # distance shrinks so homing slows down near the toy.
        distance = self._distance(entry)
        size = entry.get("size", "none")
        if distance is not None:
            step_dist = max(0.05, min(0.2, distance / 2.0))
        else:
            step_dist = 0.2 if size in ("small", "none") else 0.1
        self.straight(step_dist)
        self.step += 1
        if self.failsafe_hit():
            # The Pico IR failsafe stopped us at the toy; do not push further.
            self.get_logger().info(
                "Pico failsafe stopped homing next to the toy; re-checking.")
        # Re-check.
        new_result = self.ask_vision()
        found_now = self._note_found(new_result)
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
        try:
            self.wait_for_odom()
        except RuntimeError as error:
            # No pose feedback: still write the (empty) log so the failed run
            # leaves an artifact instead of a bare traceback.
            self.get_logger().error(str(error))
            self._write_csv()
            self._print_summary(t0)
            return
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
                home_new = False
                if result is not None:
                    if self._is_found(result):
                        found_now = self._note_found(result)
                    elif self._entry(result).get("target_visible") and self.home_on_visible:
                        # Try to home in on the visible target. Home logs its
                        # own row, so the side row below is not marked found
                        # for the same toy.
                        home_new = self.home_on_target(result, t0)
                self.log_step("side", result, found_now, t0)
                if self.found:
                    self.stop()
                    self._write_csv()
                    self._print_summary(t0)
                    return
                if (found_now or home_new) and self.find_all:
                    # Found a new toy in demo mode: back away and keep sweeping.
                    self._escape_from_toy()
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
        print(f"find_all:      {self.find_all}")
        print(f"found:         {self.found}")
        print(f"found_step:    {self.found_step}")
        print(f"toys_found:    {len(self.toys)}")
        for i, (tx, ty, label) in enumerate(self.toys, 1):
            print(f"  toy #{i}:       ({tx:.2f}, {ty:.2f}) {label}")
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
