"""Full-system search test: drive a search pattern, ask the vision node about the
target toy after every move, and log pose/vision data for evaluation.

Subscribes  /odometry/filtered   nav_msgs/Odometry
            /vision/result       std_msgs/String (JSON)
Publishes   /vision/ask         std_msgs/String (JSON)
            /cmd_vel_auto       (via Mover; cmd_vel_mux forwards to the Pico)

Parameters
----------
target          : str   toy description passed to the vision node
pattern         : str   "square" (expanding square) or "lawnmower"
side            : float starting side length in metres
side_increment  : float growth of the side per loop (metres)
loops           : int   number of search loops
max_time        : float abort after this many seconds
vision_timeout  : float seconds to wait for a /vision/result
output_csv      : str   path of the CSV log to write
run_id          : str   label stored in the log
found_distance  : float stop when the VLM estimates the target within (m)
find_all        : bool  demo mode: record each new toy and keep searching
                    instead of stopping at the first one
toy_side_offset / toy_forward_distance / toy_merge_radius : toy placement
                    for the found-toy estimate (same as the occupancy mapper)
escape_distance : float metres backed up after each find in demo mode
"""

import csv
import json
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from dog_toy_finder_motion.mover import Mover, yaw_from_quaternion


class ToySearchTest(Mover):
    def __init__(self):
        super().__init__(node_name="toy_search_test")
        self.declare_parameter("target", "red ball")
        self.declare_parameter("pattern", "square")
        self.declare_parameter("side", 0.5)
        self.declare_parameter("side_increment", 0.5)
        self.declare_parameter("loops", 3)
        self.declare_parameter("max_time", 120.0)
        self.declare_parameter("vision_timeout", 10.0)
        self.declare_parameter("output_csv", "/tmp/toy_search.csv")
        self.declare_parameter("run_id", "run1")
        self.declare_parameter("found_distance", 0.4)  # m, stop when this close
        self.declare_parameter("find_all", False)      # demo: keep searching
        self.declare_parameter("toy_side_offset", 0.25)  # m left/right estimate
        self.declare_parameter("toy_forward_distance", 0.6)  # fallback m ahead
        self.declare_parameter("toy_merge_radius", 0.15)  # m, same-toy radius
        self.declare_parameter("toy_max_count", 500)
        self.declare_parameter("escape_distance", 0.3)  # m backed up per find

        self.target = self.get_parameter("target").value
        # Comma-separated toy descriptions; the search asks the vision node
        # about each one in turn. A single name searches that one kind.
        self.targets = [t.strip() for t in str(self.target).split(",")
                        if t.strip()] or [str(self.target)]
        self.pattern = self.get_parameter("pattern").value
        self.side = float(self.get_parameter("side").value)
        self.side_increment = float(self.get_parameter("side_increment").value)
        self.loops = int(self.get_parameter("loops").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout").value)
        self.output_csv = self.get_parameter("output_csv").value
        self.run_id = self.get_parameter("run_id").value
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

        self._vision_lock = threading.Lock()
        self._vision_result = None
        self._vision_event = threading.Event()

        self.rows = []
        self.found = False
        self.found_step = None
        # [(x, y, target), ...] distinct found toys in the odom frame.
        self.toys = []

    def on_vision_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._vision_lock:
            self._vision_result = data
            self._vision_event.set()

    def ask_one(self, target):
        """Ask the vision node about one target and wait for its result."""
        self._vision_event.clear()
        with self._vision_lock:
            self._vision_result = None
        req_id = str(int(time.time() * 1000))
        self.ask_pub.publish(String(data=json.dumps(
            {"id": req_id, "target": target})))
        if not self._vision_event.wait(timeout=self.vision_timeout):
            return None
        with self._vision_lock:
            return self._vision_result

    def ask_vision(self):
        """Ask about every target in turn; return the most useful result.

        A found result wins immediately, then a visible-but-not-found
        result, then the last not-visible result (for logging).
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
        try:
            distance = float(entry.get("distance_m") or 0.0)
        except (TypeError, ValueError):
            return False
        return 0.0 < distance <= self.found_distance

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
        try:
            ahead = float(entry.get("distance_m") or 0.0)
        except (TypeError, ValueError):
            ahead = 0.0
        if ahead <= 0.0:
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
        return True

    def _escape_from_toy(self):
        """Back away from a just-found toy before searching again."""
        self.get_logger().info(
            f"Backing up {self.escape_distance:.2f} m from the toy "
            "before continuing the search.")
        self.straight(-abs(self.escape_distance))

    def log_step(self, step, result, found_now, t0):
        x, y, theta = self.current_pose()
        vision_seconds = None
        target_visible = False
        position = "none"
        size = "none"
        obstacle = "none"
        path_clear = True
        if result is not None:
            target_visible = bool(result.get("target_visible", False))
            position = result.get("position", "none")
            size = result.get("size", "none")
            obstacle = result.get("obstacle_ahead", "none")
            path_clear = bool(result.get("path_clear", True))
            vision_seconds = result.get("seconds")
        self.rows.append({
            "run_id": self.run_id,
            "step": step,
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
        })

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
        step = 0
        side = self.side

        for loop in range(self.loops):
            if time.time() - t0 > self.max_time:
                break
            for _ in range(4):
                if time.time() - t0 > self.max_time:
                    break
                # Drive one side.
                self.straight(side)
                step += 1
                result = self.ask_vision()
                found_now = False
                if result is not None:
                    found_now = self._note_found(result)
                    if found_now and not self.find_all:
                        self.found_step = step
                self.log_step(step, result, found_now, t0)
                self.get_logger().info(
                    f"step {step}: found={found_now} visible={result.get('target_visible') if result else None} "
                    f"pos={result.get('position') if result else None} size={result.get('size') if result else None}")
                if self.found:
                    self.stop()
                    self._write_csv()
                    self._print_summary(t0)
                    return
                if found_now and self.find_all:
                    # Found a new toy in demo mode: back away and keep sweeping.
                    self._escape_from_toy()
                # Turn 90 deg left for the next side.
                self.turn(math.pi / 2.0)
            side += self.side_increment

        self.stop()
        self._write_csv()
        self._print_summary(t0)

    def _write_csv(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.output_csv)), exist_ok=True)
        fieldnames = ["run_id", "step", "t_sec", "x", "y", "theta_deg", "target",
                      "target_visible", "position", "size", "obstacle_ahead",
                      "path_clear", "vision_seconds", "found"]
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)
        self.get_logger().info(f"Wrote {len(self.rows)} rows to {self.output_csv}")

    def _print_summary(self, t0):
        elapsed = time.time() - t0
        print("\n===== TOY SEARCH SUMMARY =====")
        print(f"run_id:        {self.run_id}")
        print(f"target:        {self.target}")
        print(f"pattern:       {self.pattern}")
        print(f"find_all:      {self.find_all}")
        print(f"found:         {self.found}")
        print(f"found_step:    {self.found_step}")
        print(f"toys_found:    {len(self.toys)}")
        for i, (tx, ty, label) in enumerate(self.toys, 1):
            print(f"  toy #{i}:       ({tx:.2f}, {ty:.2f}) {label}")
        print(f"time (s):     {elapsed:.2f}")
        print(f"steps:         {len(self.rows)}")
        if self.rows:
            xs = [r["x"] for r in self.rows]
            ys = [r["y"] for r in self.rows]
            path = 0.0
            for i in range(1, len(self.rows)):
                path += math.hypot(self.rows[i]["x"] - self.rows[i - 1]["x"],
                                   self.rows[i]["y"] - self.rows[i - 1]["y"])
            print(f"path (m):     {path:.3f}")
            print(f"final pose:   ({xs[-1]:.3f}, {ys[-1]:.3f})")
        print(f"csv:           {self.output_csv}")
        print("==============================\n")


def main(args=None):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ToySearchTest()
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
