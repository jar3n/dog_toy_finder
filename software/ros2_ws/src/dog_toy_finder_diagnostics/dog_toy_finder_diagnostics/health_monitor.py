"""Sensor heartbeat health monitor for the Dog Toy Finder.

Subscribes to every hardware sensor topic (Pico IMU, wheel encoders, IR
obstacle sensor, both ESP32-CAM eyes) plus the fused odometry, and logs an
ERROR on /rosout when a sensor stops publishing (its heartbeat goes silent).
Each sensor is tracked independently: a failure is logged once when the
sensor goes stale past its timeout, and an INFO is logged when it recovers.

The Pico sensors publish continuously (IMU 50 Hz, encoders 50 Hz, IR 10 Hz)
and the cameras publish at the stereo rate (2 Hz by default), so topic
silence reliably means the sensor, the Pico, the camera, or the link died.
A startup grace period suppresses failure logs while the stack boots: a
sensor that has never published only fails once the grace has expired.
"""

import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Bool

# (key, topic, message type, timeout parameter, default timeout in seconds)
SENSORS = (
    ("imu", "/imu/data", Imu, "imu_timeout", 0.5),
    ("encoders", "/joint_states", JointState, "joint_states_timeout", 0.5),
    ("ir", "/ir_obstacle/detected", Bool, "ir_timeout", 1.0),
    ("left_camera", "/stereo/left/image_raw", Image,
     "left_camera_timeout", 3.0),
    ("right_camera", "/stereo/right/image_raw", Image,
     "right_camera_timeout", 3.0),
    ("odometry", "/odometry/filtered", Odometry,
     "odometry_timeout", 2.0),
)

OK = "ok"
FAIL = "fail"
UNKNOWN = "unknown"


class HealthMonitor(Node):
    """Log an ERROR whenever a monitored sensor heartbeat goes silent."""

    def __init__(self):
        super().__init__("health_monitor")

        self.declare_parameter("check_period", 0.5)    # s between checks
        self.declare_parameter("startup_grace", 10.0)  # s before first failure
        for _key, _topic, _msg, timeout_param, default in SENSORS:
            self.declare_parameter(timeout_param, default)

        self.check_period = float(self.get_parameter("check_period").value)
        self.startup_grace = float(self.get_parameter("startup_grace").value)
        self.start_time = time.monotonic()
        self.lock = threading.Lock()

        self.sensors = {}
        for key, topic, msg_type, timeout_param, _default in SENSORS:
            timeout = float(self.get_parameter(timeout_param).value)
            self.sensors[key] = {
                "topic": topic,
                "timeout": timeout,
                "last_seen": None,   # time.monotonic() of the last message
                "status": UNKNOWN,
            }
            self.create_subscription(
                msg_type, topic, self._make_callback(key),
                qos_profile_sensor_data,
            )

        self.create_timer(self.check_period, self.check)
        self.get_logger().info(
            "Health monitor ready: watching %d sensors %s"
            % (len(self.sensors), ", ".join(self.sensors)))

    def _make_callback(self, key: str):
        def callback(_msg):
            with self.lock:
                self.sensors[key]["last_seen"] = time.monotonic()
        return callback

    def sensor_states(self) -> dict:
        """Return {sensor key: "ok" | "fail" | "unknown"}."""
        with self.lock:
            return {key: s["status"] for key, s in self.sensors.items()}

    def all_ok(self) -> bool:
        """Return true when every monitored sensor is currently reporting."""
        return all(s == OK for s in self.sensor_states().values())

    def check(self) -> None:
        """Log failures for stale sensors and recoveries for revived ones."""
        now = time.monotonic()
        booted = now - self.start_time >= self.startup_grace
        with self.lock:
            for key, sensor in self.sensors.items():
                last_seen = sensor["last_seen"]
                status = sensor["status"]
                if last_seen is None:
                    if booted and status != FAIL:
                        sensor["status"] = FAIL
                        self.get_logger().error(
                            "Sensor %s FAILED: no message ever received on %s "
                            "(startup grace %.1f s expired). Check the sensor, "
                            "the Pico/micro-ROS agent, or the camera link."
                            % (key, sensor["topic"], self.startup_grace))
                    continue
                age = now - last_seen
                if age > sensor["timeout"] and status != FAIL:
                    sensor["status"] = FAIL
                    self.get_logger().error(
                        "Sensor %s FAILED: no message on %s for %.1f s "
                        "(timeout %.1f s). Check the sensor, the Pico/"
                        "micro-ROS agent, or the camera link."
                        % (key, sensor["topic"], age, sensor["timeout"]))
                elif age <= sensor["timeout"] and status == FAIL:
                    sensor["status"] = OK
                    self.get_logger().info(
                        "Sensor %s recovered: %s is publishing again."
                        % (key, sensor["topic"]))
                elif age <= sensor["timeout"] and status == UNKNOWN:
                    sensor["status"] = OK
                    self.get_logger().info(
                        "Sensor %s OK: receiving %s."
                        % (key, sensor["topic"]))


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
