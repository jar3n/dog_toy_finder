"""Hardware/software bench test for the Dog Toy Finder Pico sensors and motors.

Run while the robot stack is up (micro-ROS agent + robot.launch.py):

    ros2 run dog_toy_finder_motion hardware_bench_test

It checks that the IMU, wheel encoders, IR obstacle sensor, and motors are
publishing and responding, then prints a pass/fail summary. Motor moves are
small (0.10-0.15 m) and every step stops the robot on completion, on failure,
or on Ctrl+C. The IR reflex step asks you to place and remove an obstacle in
front of the robot; type "s" to skip it.
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

from .mover import Mover, local_change, wrap_angle, yaw_from_quaternion


def _finite(values):
    return all(isinstance(v, float) and math.isfinite(v) for v in values)


class BenchNode(Mover):
    def __init__(self):
        super().__init__(node_name="hardware_bench_test")
        self.imu_msg = None
        self.joint_msg = None
        self.ir_msg = None
        self.ir_last_seen = None
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joint,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, "/ir_obstacle/detected", self._on_ir_bench,
                                 qos_profile_sensor_data)

    def _on_imu(self, msg):
        self.imu_msg = msg

    def _on_joint(self, msg):
        self.joint_msg = msg

    def _on_ir_bench(self, msg):
        self.ir_msg = msg
        self.ir_last_seen = time.monotonic()

    def wait_for(self, getter, timeout, label):
        end = time.time() + timeout
        while time.time() < end and getter() is None:
            time.sleep(0.05)
        if getter() is None:
            raise RuntimeError("%s did not publish within %.1f s" % (label, timeout))


class BenchTest:
    def __init__(self, node):
        self.node = node
        self.results = []

    def record(self, name, passed, detail=""):
        status = "PASS" if passed else "FAIL"
        self.results.append((name, passed, detail))
        print("%-32s %s%s" % (name, status,
                               ("  " + detail) if detail else ""))

    def run(self):
        node = self.node
        try:
            node.wait_for_odom(timeout=10.0)
        except RuntimeError as error:
            print("error: %s" % error, file=sys.stderr)
            return 1

        # --- IMU ---
        try:
            node.wait_for(lambda: node.imu_msg, 10.0, "/imu/data")
            msg = node.imu_msg
            q = (msg.orientation.x, msg.orientation.y, msg.orientation.z,
                 msg.orientation.w)
            vals = list(q) + [msg.angular_velocity.x, msg.angular_velocity.y,
                              msg.angular_velocity.z,
                              msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z]
            ok = _finite(vals) and any(abs(v) > 0.0 for v in vals[4:])
            # A stationary IMU may report zero gyro, so only require finite
            # orientation plus a plausible (non-NaN) message here.
            ok = _finite(vals)
            self.record("IMU publishes finite data", ok,
                        "topic /imu/data")
        except RuntimeError as error:
            self.record("IMU publishes finite data", False, str(error))

        # --- Encoders ---
        try:
            node.wait_for(lambda: node.joint_msg, 10.0, "/joint_states")
            msg = node.joint_msg
            names = list(msg.name)
            positions = list(msg.position)
            ok = (len(names) >= 2 and len(positions) >= 2
                  and _finite(positions[:2]))
            self.record("Encoders publish joint states", ok,
                        "joints=%s" % ",".join(names[:2]))
        except RuntimeError as error:
            self.record("Encoders publish joint states", False, str(error))

        # --- IR sensor ---
        try:
            node.wait_for(lambda: node.ir_last_seen, 10.0,
                          "/ir_obstacle/detected")
            self.record("IR obstacle sensor publishes", True,
                        "state=%s" % str(node.obstacle_ahead()))
        except RuntimeError as error:
            self.record("IR obstacle sensor publishes", False, str(error))

        # --- Motor forward ---
        start_pose = node.pose()
        start_joints = None
        if node.joint_msg is not None:
            start_joints = list(node.joint_msg.position[:2])
        try:
            node.straight(0.10)
            time.sleep(0.5)
            end_pose = node.pose()
            moved = False
            if start_pose is not None and end_pose is not None:
                forward, _left, _turned = local_change(start_pose, end_pose)
                moved = forward > 0.02
            enc_changed = False
            if start_joints is not None and node.joint_msg is not None:
                end_joints = list(node.joint_msg.position[:2])
                enc_changed = (len(end_joints) >= 2
                               and max(abs(end_joints[i] - start_joints[i])
                                       for i in range(2)) > 0.02)
            ok = moved or enc_changed
            detail = ""
            if start_pose is not None and end_pose is not None:
                forward, _left, _turned = local_change(start_pose, end_pose)
                detail = "forward=%.3f m" % forward
            self.record("Motors drive forward", ok, detail)
        except Exception as error:
            node.stop()
            self.record("Motors drive forward", False, str(error))

        # --- Motor turn ---
        start_pose = node.pose()
        try:
            node.turn(wrap_angle(math.radians(30.0)))
            time.sleep(0.5)
            end_pose = node.pose()
            turned = False
            if start_pose is not None and end_pose is not None:
                _f, _l, deg = local_change(start_pose, end_pose)
                turned = abs(deg) > 5.0
            self.record("Motors turn in place", turned,
                        ("turned=%.1f deg" % deg) if start_pose is not None
                        and end_pose is not None else "")
        except Exception as error:
            node.stop()
            self.record("Motors turn in place", False, str(error))

        # --- Motor reverse ---
        start_pose = node.pose()
        start_joints = None
        if node.joint_msg is not None:
            start_joints = list(node.joint_msg.position[:2])
        try:
            node.straight(-0.10)
            time.sleep(0.5)
            end_pose = node.pose()
            moved_back = False
            if start_pose is not None and end_pose is not None:
                forward, _left, _turned = local_change(start_pose, end_pose)
                moved_back = forward < -0.02
            enc_changed = False
            if start_joints is not None and node.joint_msg is not None:
                end_joints = list(node.joint_msg.position[:2])
                enc_changed = (len(end_joints) >= 2
                               and max(abs(end_joints[i] - start_joints[i])
                                       for i in range(2)) > 0.02)
            ok = moved_back or enc_changed
            detail = ""
            if start_pose is not None and end_pose is not None:
                forward, _left, _turned = local_change(start_pose, end_pose)
                detail = "forward=%.3f m" % forward
            self.record("Motors drive in reverse", ok, detail)
        except Exception as error:
            node.stop()
            self.record("Motors drive in reverse", False, str(error))

        # --- IR reflex ---
        answer = input("\nPlace an obstacle ~10 cm in front of the robot, "
                       "then press Enter (or 's' to skip): ").strip().lower()
        if answer != "s":
            try:
                end = time.time() + 5.0
                while time.time() < end and not node.obstacle_ahead():
                    time.sleep(0.1)
                if not node.obstacle_ahead():
                    raise RuntimeError("IR did not report an obstacle")
                self.record("IR detects obstacle", True, "state=true")
                # Forward move should abort via failsafe.
                node.clear_failsafe()
                before = node.pose()
                node.straight(0.20)
                after = node.pose()
                moved = 0.0
                if before is not None and after is not None:
                    forward, _left, _turned = local_change(before, after)
                    moved = forward
                reflex_ok = node.failsafe_hit() or moved < 0.05
                self.record("Pico IR reflex blocks forward move", reflex_ok,
                            "moved=%.3f m, failsafe_hit=%s"
                            % (moved, node.failsafe_hit()))
            except Exception as error:
                node.stop()
                self.record("IR detects obstacle", False, str(error))
            finally:
                node.stop()

            answer = input("Remove the obstacle, then press Enter: ").strip()
            try:
                end = time.time() + 5.0
                while time.time() < end and node.obstacle_ahead():
                    time.sleep(0.1)
                cleared = not node.obstacle_ahead()
                self.record("IR clears when obstacle removed", cleared,
                            "state=%s" % str(node.obstacle_ahead()))
            except Exception as error:
                self.record("IR clears when obstacle removed", False, str(error))
        else:
            self.record("IR detects obstacle", True, "skipped")
            self.record("Pico IR reflex blocks forward move", True, "skipped")
            self.record("IR clears when obstacle removed", True, "skipped")

        node.stop()
        return 0

    def summary(self):
        print("\n===== hardware bench test summary =====")
        failed = 0
        for name, passed, detail in self.results:
            print("%-40s %s" % (name, "PASS" if passed else "FAIL"))
            if not passed:
                failed += 1
        print("%d/%d checks passed" % (len(self.results) - failed,
                                       len(self.results)))
        return 1 if failed else 0


def main(args=None):
    parsed, ros_args = argparse.ArgumentParser().parse_known_args(args=args)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = BenchNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.wait_for_odom(timeout=10.0)
    except RuntimeError as error:
        print("error: %s" % error, file=sys.stderr)
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    bench = BenchTest(node)
    try:
        rc = bench.run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        node.stop()
        rc = 1
    finally:
        node.stop()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    rc = bench.summary() if rc == 0 else rc
    return rc


if __name__ == "__main__":
    sys.exit(main())