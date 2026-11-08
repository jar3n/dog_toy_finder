"""Full-system search test: drive a search pattern, ask the vision node about the
target toy after every move, and log pose/vision data for evaluation.

Subscribes  /odometry/filtered   nav_msgs/Odometry
            /vision/result       std_msgs/String (JSON)
Publishes   /vision/ask         std_msgs/String (JSON)
            /cmd_vel            (via the Mover base class)

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

        self.target = self.get_parameter("target").value
        self.pattern = self.get_parameter("pattern").value
        self.side = float(self.get_parameter("side").value)
        self.side_increment = float(self.get_parameter("side_increment").value)
        self.loops = int(self.get_parameter("loops").value)
        self.max_time = float(self.get_parameter("max_time").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout").value)
        self.output_csv = self.get_parameter("output_csv").value
        self.run_id = self.get_parameter("run_id").value

        self.ask_pub = self.create_publisher(String, "/vision/ask", 10)
        self.create_subscription(String, "/vision/result", self.on_vision_result,
                                 qos_profile_sensor_data)

        self._vision_lock = threading.Lock()
        self._vision_result = None
        self._vision_event = threading.Event()

        self.rows = []
        self.found = False
        self.found_step = None

    def on_vision_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._vision_lock:
            self._vision_result = data
            self._vision_event.set()

    def ask_vision(self):
        """Publish a /vision/ask and wait for the matching result."""
        self._vision_event.clear()
        with self._vision_lock:
            self._vision_result = None
        req_id = str(int(time.time() * 1000))
        self.ask_pub.publish(String(data=json.dumps(
            {"id": req_id, "target": self.target})))
        if not self._vision_event.wait(timeout=self.vision_timeout):
            return None
        with self._vision_lock:
            return self._vision_result

    def current_pose(self):
        pose = self.pose("odom")
        if pose is None:
            return 0.0, 0.0, 0.0
        return pose

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
        self.wait_for_odom()
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
                    tv = result.get("target_visible", False)
                    sz = result.get("size", "none")
                    if tv and sz in ("medium", "large"):
                        found_now = True
                        self.found = True
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
        print(f"found:         {self.found}")
        print(f"found_step:    {self.found_step}")
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
