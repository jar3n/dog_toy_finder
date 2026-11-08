"""Occupancy mapper: builds a 2D map of where the robot has been.

Subscribes  /odometry/filtered   nav_msgs/Odometry
            /ir_obstacle/detected  std_msgs/Bool   (optional)
Publishes   /map                nav_msgs/OccupancyGrid
            /tracked_path       nav_msgs/Path

The map is an occupancy grid centred on the origin. Cells the robot has
travelled through are marked free (0); when the IR obstacle sensor reports an
obstacle, the cells a short distance ahead of the robot are marked occupied
(100). Unknown cells stay -1. This gives a lightweight "where have I been"
map without a laser scanner.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__("occupancy_mapper")

        self.declare_parameter("resolution", 0.05)      # m/cell
        self.declare_parameter("width", 200)             # cells
        self.declare_parameter("height", 200)            # cells
        self.declare_parameter("origin_x", -5.0)        # m
        self.declare_parameter("origin_y", -5.0)        # m
        self.declare_parameter("obstacle_range", 0.25)  # m ahead when IR fires
        self.declare_parameter("publish_period", 1.0)   # s
        self.declare_parameter("ir_topic", "/ir_obstacle/detected")

        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.obs_range = float(self.get_parameter("obstacle_range").value)
        period = float(self.get_parameter("publish_period").value)
        ir_topic = self.get_parameter("ir_topic").value

        self.grid = np.full((self.h, self.w), -1, dtype=np.int8)
        self.obstacle_ahead = False

        self.create_subscription(Odometry, "/odometry/filtered", self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, ir_topic, self.on_ir,
                                 qos_profile_sensor_data)

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.path_pub = self.create_publisher(Path, "/tracked_path", 10)
        self.path = Path()
        self.path.header.frame_id = "odom"

        self.prev_cell = None
        self.last_pose = None
        self.create_timer(period, self.publish)

        self.get_logger().info(
            f"Occupancy mapper ready: {self.w}x{self.h} cells at {self.res} m, "
            f"origin ({self.origin_x}, {self.origin_y})")

    def world_to_cell(self, x, y):
        c = int((x - self.origin_x) / self.res)
        r = int((y - self.origin_y) / self.res)
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def bresenham(self, r0, c0, r1, c1):
        cells = []
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
        return cells

    def on_pose(self, msg):
        p = msg.pose.pose.position
        x, y = p.x, p.y
        yaw = math.atan2(2.0 * (msg.pose.pose.orientation.w *
                                msg.pose.pose.orientation.z +
                                msg.pose.pose.orientation.x *
                                msg.pose.pose.orientation.y),
                         1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 +
                                      msg.pose.pose.orientation.z ** 2))
        cell = self.world_to_cell(x, y)
        if cell is None:
            self.last_pose = (x, y, yaw)
            return
        if self.prev_cell is not None and self.prev_cell != cell:
            for r, c in self.bresenham(self.prev_cell[0], self.prev_cell[1],
                                       cell[0], cell[1]):
                if 0 <= r < self.h and 0 <= c < self.w:
                    self.grid[r, c] = 0  # free
        self.grid[cell[0], cell[1]] = 0
        self.prev_cell = cell
        self.last_pose = (x, y, yaw)

        # Mark obstacle cells ahead when the IR sensor fires.
        if self.obstacle_ahead and self.last_pose is not None:
            fx = x + self.obs_range * math.cos(yaw)
            fy = y + self.obs_range * math.sin(yaw)
            ocell = self.world_to_cell(fx, fy)
            if ocell is not None:
                # Mark a small block ahead.
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        rr, cc = ocell[0] + dr, ocell[1] + dc
                        if 0 <= rr < self.h and 0 <= cc < self.w:
                            self.grid[rr, cc] = 100  # occupied

        # Append to tracked path.
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "odom"
        ps.pose = msg.pose.pose
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(ps)
        # Keep path bounded.
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

    def on_ir(self, msg):
        self.obstacle_ahead = bool(msg.data)

    def publish(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "odom"
        grid_msg.info.resolution = self.res
        grid_msg.info.width = self.w
        grid_msg.info.height = self.h
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(grid_msg)
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init()
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()