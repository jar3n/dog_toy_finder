"""Occupancy mapper: builds a 2D map of where the robot has been.

Subscribes  /odometry/filtered    nav_msgs/Odometry
            /ir_obstacle/detected std_msgs/Bool   (optional)
            /vision/result        std_msgs/String (JSON, optional)
Publishes   /map                 nav_msgs/OccupancyGrid
            /tracked_path        nav_msgs/Path
            /detected_toys       geometry_msgs/PoseArray

Cells the robot has travelled through are marked free (0). Unknown cells stay
-1. Obstacle cells record which sensor reported them, so the browser view can
colour them separately:

    100  IR only
     70  vision only
     85  both sensors (concurrent detections or overlapping regions)
     60  toy (occupied by a detected toy)

When both sensors agree, both the near (IR) and far (vision) obstacle regions
are marked as "both". The toy location is estimated from the VLM's
``target_visible``/``position`` fields plus its ``distance_m`` estimate: the
distance gives the forward offset and the position gives the lateral offset to
the left or right. When the VLM has no usable distance, ``toy_forward_distance``
is used. Each toy also marks a small occupied patch in the grid (value 60) so
it is treated as occupied space, in addition to being published on
/detected_toys.
"""

import json
import math
import time
from collections import Counter

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


UNKNOWN = -1
FREE = 0
TOY = 60
VISION_ONLY = 70
BOTH = 85
IR_ONLY = 100

NO_OBSTACLE_LABELS = ("", "none", "nothing", "no", "n/a")
TOY_SIDES = {"left": 1.0, "center": 0.0, "right": -1.0}


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__("occupancy_mapper")

        self.declare_parameter("resolution", 0.05)      # m/cell
        self.declare_parameter("width", 200)             # cells
        self.declare_parameter("height", 200)            # cells
        self.declare_parameter("origin_x", -5.0)        # m
        self.declare_parameter("origin_y", -5.0)        # m
        self.declare_parameter("obstacle_range", 0.25)  # m ahead when IR fires
        self.declare_parameter("vision_obstacle_range", 0.8)  # m ahead for VLM obstacles
        self.declare_parameter("vision_timeout", 5.0)   # s a vision result stays fresh
        self.declare_parameter("publish_period", 1.0)   # s
        self.declare_parameter("ir_topic", "/ir_obstacle/detected")
        self.declare_parameter("use_vision", True)

        # Toy marker estimate.
        self.declare_parameter("toy_forward_distance", 0.6)  # fallback m ahead
        self.declare_parameter("toy_side_offset", 0.25)      # m left/right
        self.declare_parameter("toy_max_distance", 5.0)      # clamp VLM estimates
        self.declare_parameter("toy_dedupe_radius", 0.15)    # m
        self.declare_parameter("toy_occupancy_radius", 1)    # cells around toy
        self.declare_parameter("toy_max_count", 500)

        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.obs_range = float(self.get_parameter("obstacle_range").value)
        self.vision_obs_range = float(self.get_parameter("vision_obstacle_range").value)
        self.vision_timeout = float(self.get_parameter("vision_timeout").value)
        period = float(self.get_parameter("publish_period").value)
        ir_topic = self.get_parameter("ir_topic").value
        self.use_vision = bool(self.get_parameter("use_vision").value)
        self.toy_forward = float(self.get_parameter("toy_forward_distance").value)
        self.toy_side = float(self.get_parameter("toy_side_offset").value)
        self.toy_max_distance = float(self.get_parameter("toy_max_distance").value)
        self.toy_dedupe = float(self.get_parameter("toy_dedupe_radius").value)
        self.toy_occupancy_radius = int(self.get_parameter("toy_occupancy_radius").value)
        self.toy_max_count = int(self.get_parameter("toy_max_count").value)

        # Free/unknown space and one mask per obstacle source.
        self.free_grid = np.full((self.h, self.w), UNKNOWN, dtype=np.int8)
        self.ir_mask = np.zeros((self.h, self.w), dtype=bool)
        self.vision_mask = np.zeros((self.h, self.w), dtype=bool)
        self.both_mask = np.zeros((self.h, self.w), dtype=bool)
        self.toy_mask = np.zeros((self.h, self.w), dtype=bool)

        self.obstacle_ahead = False
        self.vision_obstacle = None   # obstacle label from the VLM, or None
        self.vision_stamp = None      # time.monotonic() of the last vision result
        self.toys = []                # [(x, y), ...] in the odom frame

        self.create_subscription(Odometry, "/odometry/filtered", self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, ir_topic, self.on_ir,
                                 qos_profile_sensor_data)
        if self.use_vision:
            self.create_subscription(String, "/vision/result", self.on_vision,
                                     qos_profile_sensor_data)

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.path_pub = self.create_publisher(Path, "/tracked_path", 10)
        self.toy_pub = self.create_publisher(PoseArray, "/detected_toys", 10)
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

    def mark_free(self, r, c):
        """Mark one cell as driven-through free space, clearing obstacle masks."""
        if 0 <= r < self.h and 0 <= c < self.w:
            self.free_grid[r, c] = FREE
            self.ir_mask[r, c] = False
            self.vision_mask[r, c] = False
            self.both_mask[r, c] = False

    def mark_patch(self, mask, x, y, radius):
        """Mark a square patch of cells around a world point."""
        cell = self.world_to_cell(x, y)
        if cell is None:
            return
        r0 = max(0, cell[0] - radius)
        r1 = min(self.h, cell[0] + radius + 1)
        c0 = max(0, cell[1] - radius)
        c1 = min(self.w, cell[1] + radius + 1)
        mask[r0:r1, c0:c1] = True

    def mark_ahead(self, mask, x, y, yaw, distance, radius):
        """Mark a square obstacle patch ``distance`` metres ahead."""
        self.mark_patch(mask, x + distance * math.cos(yaw),
                        y + distance * math.sin(yaw), radius)

    def vision_is_active(self):
        if self.vision_obstacle is None or self.vision_stamp is None:
            return False
        if self.vision_timeout <= 0:
            return True
        return time.monotonic() - self.vision_stamp <= self.vision_timeout

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
                    self.free_grid[r, c] = FREE
        self.mark_free(cell[0], cell[1])
        self.prev_cell = cell
        self.last_pose = (x, y, yaw)

        ir_active = bool(self.obstacle_ahead)
        vision_active = self.use_vision and self.vision_is_active()

        if ir_active and vision_active:
            # Both sensors agree: mark both the near and far regions as jointly
            # detected so the browser can show the fusion result.
            self.mark_ahead(self.both_mask, x, y, yaw, self.obs_range, 1)
            self.mark_ahead(self.both_mask, x, y, yaw, self.vision_obs_range, 2)
        elif ir_active:
            self.mark_ahead(self.ir_mask, x, y, yaw, self.obs_range, 1)
        elif vision_active:
            self.mark_ahead(self.vision_mask, x, y, yaw,
                            self.vision_obs_range, 2)

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

    def on_vision(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        results = data.get("results")
        if not isinstance(results, list):
            results = [data]

        labels = []
        toys = []  # [(position, distance_m or None), ...]
        for result in results:
            if not isinstance(result, dict):
                continue
            obs = result.get("obstacle_ahead", "none")
            if isinstance(obs, str) and obs.strip().lower() not in NO_OBSTACLE_LABELS:
                labels.append(obs)
            if result.get("target_visible") is True:
                position = str(result.get("position", "none")).strip().lower()
                if position in TOY_SIDES:
                    toys.append((position, self._toy_distance(result)))

        self.vision_obstacle = labels[0] if labels else None
        self.vision_stamp = time.monotonic()

        if toys:
            position = Counter(p for p, _ in toys).most_common(1)[0][0]
            distances = [d for p, d in toys if p == position and d is not None]
            distance = float(np.median(distances)) if distances else None
            self.record_toy(position, distance)

    def _toy_distance(self, result):
        """Return a sane VLM distance in metres, or None to use the fallback."""
        value = result.get("distance_m", result.get("distance"))
        try:
            distance = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(distance) or distance <= 0.0:
            return None
        return min(distance, self.toy_max_distance)

    def record_toy(self, position, distance=None):
        """Add one toy marker ahead and to the side, deduplicated."""
        if self.last_pose is None:
            return
        x, y, yaw = self.last_pose
        ahead = self.toy_forward if distance is None else distance
        side = TOY_SIDES[position] * self.toy_side
        # ROS convention: +x forward, +y left, yaw counter-clockwise.
        tx = x + ahead * math.cos(yaw) - side * math.sin(yaw)
        ty = y + ahead * math.sin(yaw) + side * math.cos(yaw)
        if self.world_to_cell(tx, ty) is None:
            return

        for ox, oy in self.toys:
            if (ox - tx) ** 2 + (oy - ty) ** 2 <= self.toy_dedupe ** 2:
                return

        self.toys.append((tx, ty))
        self.mark_patch(self.toy_mask, tx, ty, self.toy_occupancy_radius)
        if len(self.toys) > self.toy_max_count:
            self.toys = self.toys[-self.toy_max_count:]
        self.get_logger().info(
            f"Toy detected ({position}) at ({tx:.2f}, {ty:.2f}) in odom"
            f" [{ahead:.2f} m ahead, {side:+.2f} m lateral]")

    def publish(self):
        data = self.free_grid.copy()
        data[self.ir_mask] = IR_ONLY
        data[self.vision_mask] = VISION_ONLY
        # Cells seen by both sensors (including overlapping detections recorded
        # at different times) and the explicit joint near/far patches.
        data[self.ir_mask & self.vision_mask] = BOTH
        data[self.both_mask] = BOTH
        # Toys are occupied space, drawn separately from sensor obstacles.
        data[self.toy_mask] = TOY

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
        grid_msg.data = data.flatten().tolist()
        self.map_pub.publish(grid_msg)
        self.path_pub.publish(self.path)

        toys_msg = PoseArray()
        toys_msg.header = grid_msg.header
        for tx, ty in self.toys:
            pose = Pose()
            pose.position.x = tx
            pose.position.y = ty
            pose.orientation.w = 1.0
            toys_msg.poses.append(pose)
        self.toy_pub.publish(toys_msg)


def main(args=None):
    rclpy.init(args=args)
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
