"""Save the latest /map (nav_msgs/OccupancyGrid) to a PGM + YAML file pair.

Usage:
    ros2 run dog_toy_finder_mapping save_map -o /tmp/my_map
Writes /tmp/my_map.pgm and /tmp/my_map.yaml (map_server format).
"""

import argparse
import os
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MapSaver(Node):
    def __init__(self, output):
        super().__init__("map_saver")
        self.output = output
        self.done = False
        self.create_subscription(OccupancyGrid, "/map", self.on_map,
                                 qos_profile_sensor_data)

    def on_map(self, msg):
        if self.done:
            return
        self.done = True
        info = msg.info
        w, h = info.width, info.height
        # OccupancyGrid data is row-major, origin at bottom-left; PGM is
        # top-left origin, so flip vertically.
        rows = []
        for r in range(h):
            row = []
            for c in range(w):
                v = msg.data[r * w + c]
                if v < 0:
                    row.append(205)   # unknown -> light gray
                elif v == 0:
                    row.append(254)   # free -> white
                else:
                    row.append(0)     # occupied -> black
            rows.append(row)
        rows.reverse()

        pgm_path = self.output + ".pgm"
        with open(pgm_path, "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (w, h))
            for row in rows:
                f.write(bytes(row))

        yaml_path = self.output + ".yaml"
        with open(yaml_path, "w") as f:
            f.write("image: %s.pgm\n" % os.path.basename(self.output))
            f.write("mode: trinary\n")
            f.write("resolution: %f\n" % info.resolution)
            f.write("origin: [%f, %f, 0.0]\n" % (info.origin.position.x,
                                                  info.origin.position.y))
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
        self.get_logger().info(f"Saved map to {pgm_path} and {yaml_path}")


def main(args=None):
    parser = argparse.ArgumentParser(description="Save /map to PGM+YAML")
    parser.add_argument("-o", "--output", default="/tmp/map",
                        help="output base path (no extension)")
    parsed, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = MapSaver(parsed.output)
    try:
        # Wait up to 10 s for a map message.
        end = rclpy.clock.Clock().now().nanoseconds * 1e-9 + 10.0
        while rclpy.ok() and not node.done and rclpy.clock.Clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()