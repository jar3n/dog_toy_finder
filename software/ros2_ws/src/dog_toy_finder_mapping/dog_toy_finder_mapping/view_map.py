"""Print the latest /map as ASCII art in the terminal."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MapViewer(Node):
    def __init__(self):
        super().__init__("map_viewer")
        self.done = False
        self.grid = None
        self.info = None
        self.create_subscription(OccupancyGrid, "/map", self.on_map,
                                 qos_profile_sensor_data)

    def on_map(self, msg):
        self.grid = list(msg.data)
        self.info = msg.info
        self.done = True

    def render(self, max_cols=80):
        if self.grid is None:
            return "(no map received)"
        w, h = self.info.width, self.info.height
        # Downsample to fit max_cols.
        step = max(1, w // max_cols)
        lines = []
        for r in range(h - 1, -1, -step):
            row = []
            for c in range(0, w, step):
                v = self.grid[r * w + c]
                if v < 0:
                    row.append("?")
                elif v == 0:
                    row.append(" ")
                else:
                    row.append("#")
            lines.append("".join(row))
        return "\n".join(lines)


def main(args=None):
    rclpy.init()
    node = MapViewer()
    try:
        end = rclpy.clock.Clock().now().nanoseconds * 1e-9 + 5.0
        while rclpy.ok() and not node.done and rclpy.clock.Clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(node, timeout_sec=0.5)
        print(node.render())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()