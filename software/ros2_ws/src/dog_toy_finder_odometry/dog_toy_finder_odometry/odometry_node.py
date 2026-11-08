"""Wheel odometry: Pico JointState feedback in, /wheel/odom out.

The Pico publishes one sensor_msgs/JointState on /joint_states containing the
left and right wheel output-shaft positions (radians) and velocities (rad/s).
This node turns the position deltas into a dead-reckoned pose and forward
speed / turn rate, which the robot_localization EKF fuses with /imu/data.

Subscribes:
    /joint_states    sensor_msgs/msg/JointState   wheel positions in radians
    /odom_reset      std_msgs/msg/Empty           zero the pose and history

Publishes:
    /wheel/odom      nav_msgs/msg/Odometry        pose and twist, with covariance
    odom -> base_link transform                    only when publish_tf is true;
                                                   the EKF normally owns it

Run standalone:
    ros2 run dog_toy_finder_odometry odometry_node
    ros2 run dog_toy_finder_odometry odometry_node --ros-args \
        -p wheel_separation:=0.13 -p wheel_radius:=0.0325
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from tf2_ros import TransformBroadcaster


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class DiffDriveOdometry:
    """Dead-reckoning pose (x, y, theta) from wheel angular deltas."""

    def __init__(self, wheel_radius: float, wheel_separation: float):
        if wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be positive")
        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.reset()

    def reset(self) -> None:
        """Return the pose and velocity estimate to the origin."""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

    def update(self, d_left_rad: float, d_right_rad: float, dt: float):
        """Advance the pose by the two wheel-angle deltas over dt seconds."""
        s_left = d_left_rad * self.wheel_radius
        s_right = d_right_rad * self.wheel_radius
        ds = 0.5 * (s_left + s_right)
        dtheta = (s_right - s_left) / self.wheel_separation

        # Move along the heading halfway through the step (midpoint method).
        theta_mid = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = wrap_angle(self.theta + dtheta)

        if dt > 0.0:
            self.v, self.w = self.speeds(d_left_rad, d_right_rad, dt)
        return self.x, self.y, self.theta

    def speeds(self, d_left_rad: float, d_right_rad: float, dt: float):
        """Return forward speed (m/s) and turn rate (rad/s) over dt seconds."""
        s_left = d_left_rad * self.wheel_radius
        s_right = d_right_rad * self.wheel_radius
        forward = 0.5 * (s_left + s_right) / dt
        turn = (s_right - s_left) / self.wheel_separation / dt
        return forward, turn


class WheelOdometryNode(Node):
    """ROS 2 node that converts /joint_states into /wheel/odom."""

    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("wheel_radius", 0.0325)       # m
        self.declare_parameter("wheel_separation", 0.133)    # m
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("linear_speed_std", 0.03)     # m/s
        self.declare_parameter("turn_rate_std", 0.10)        # rad/s
        self.declare_parameter("velocity_window", 0.2)       # s
        self.declare_parameter("max_joint_jump_rad", 31.4159)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/wheel/odom")
        self.declare_parameter("reset_topic", "/odom_reset")

        def param(name):
            return self.get_parameter(name).value

        self.left_joint = str(param("left_joint_name"))
        self.right_joint = str(param("right_joint_name"))
        self.invert_left = bool(param("invert_left"))
        self.invert_right = bool(param("invert_right"))
        self.publish_tf = bool(param("publish_tf"))
        self.linear_var = float(param("linear_speed_std")) ** 2
        self.turn_var = float(param("turn_rate_std")) ** 2
        self.velocity_window = float(param("velocity_window"))
        self.max_jump = float(param("max_joint_jump_rad"))
        self.odom_frame = str(param("odom_frame"))
        self.base_frame = str(param("base_frame"))

        self.odom = DiffDriveOdometry(
            float(param("wheel_radius")), float(param("wheel_separation"))
        )
        self.last_positions = None
        self.last_time = None
        self.history = deque()   # (time, left_rad, right_rad)

        self.odom_pub = self.create_publisher(
            Odometry, str(param("odom_topic")), 10
        )
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Empty, str(param("reset_topic")), self.on_reset, 10
        )

        self.get_logger().info(
            "Wheel odometry ready: r = %.4f m, L = %.4f m, joints = %s / %s"
            % (
                self.odom.wheel_radius,
                self.odom.wheel_separation,
                self.left_joint,
                self.right_joint,
            )
        )

    def on_reset(self, _msg: Empty) -> None:
        """Zero the pose, velocity, and speed window."""
        self.odom.reset()
        self.history.clear()
        self.get_logger().info("Pose reset to (0, 0, 0)")

    def joint_index(self, msg: JointState, name: str):
        """Return the index of a joint in the message, or None when absent."""
        try:
            return msg.name.index(name)
        except ValueError:
            self.get_logger().warn(
                "JointState has no joint '%s'" % name,
                throttle_duration_sec=2.0,
            )
            return None

    def on_joint_state(self, msg: JointState) -> None:
        """Update the pose from one JointState message and publish odometry."""
        left_index = self.joint_index(msg, self.left_joint)
        right_index = self.joint_index(msg, self.right_joint)
        if left_index is None or right_index is None:
            return
        if left_index >= len(msg.position) or right_index >= len(msg.position):
            self.get_logger().warn(
                "JointState position array is shorter than the joint list",
                throttle_duration_sec=2.0,
            )
            return

        left = float(msg.position[left_index])
        right = float(msg.position[right_index])
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right

        now = self.get_clock().now()
        if self.last_positions is None:
            self.last_positions = (left, right)
            self.last_time = now
            self.history.append((now, left, right))
            return

        d_left = left - self.last_positions[0]
        d_right = right - self.last_positions[1]
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_positions = (left, right)
        self.last_time = now

        # A large jump means the Pico restarted and its counters went back to
        # zero. Keep the current pose and restart the window from this sample.
        if abs(d_left) > self.max_jump or abs(d_right) > self.max_jump:
            self.get_logger().warn(
                "Wheel position jumped (Pico restarted?); skipping this update",
                throttle_duration_sec=2.0,
            )
            self.history.clear()
            self.history.append((now, left, right))
            return

        self.odom.update(d_left, d_right, dt)

        # Replace the single-step speed with an average over velocity_window so
        # Wi-Fi bursts do not create speed spikes in the EKF.
        self.history.append((now, left, right))
        while (
            len(self.history) > 2
            and (now - self.history[1][0]).nanoseconds * 1e-9
            >= self.velocity_window
        ):
            self.history.popleft()
        t0, left0, right0 = self.history[0]
        span = (now - t0).nanoseconds * 1e-9
        if span > 0.0:
            self.odom.v, self.odom.w = self.odom.speeds(
                left - left0, right - right0, span
            )

        self.publish(now)

    def publish(self, now) -> None:
        """Publish the current pose and velocity as /wheel/odom."""
        qz = math.sin(self.odom.theta / 2.0)
        qw = math.cos(self.odom.theta / 2.0)

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.odom.x
        msg.pose.pose.position.y = self.odom.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.odom.v
        msg.twist.twist.angular.z = self.odom.w

        # Row-major 6x6 covariance over (x, y, z, roll, pitch, yaw). The EKF
        # fuses only forward speed [0] and turn rate [35].
        msg.twist.covariance = [0.0] * 36
        msg.twist.covariance[0] = self.linear_var
        msg.twist.covariance[7] = 1e-3      # no sideways sliding
        msg.twist.covariance[14] = 1e6
        msg.twist.covariance[21] = 1e6
        msg.twist.covariance[28] = 1e6
        msg.twist.covariance[35] = self.turn_var
        msg.pose.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[index] = 1e6  # wheel pose is not fused
        self.odom_pub.publish(msg)

        if self.tf_pub is None:
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.odom.x
        transform.transform.translation.y = self.odom.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_pub.sendTransform(transform)


def main(args=None):
    """Start the wheel-odometry node."""
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        print("wheel odometry configuration error: %s" % error)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
