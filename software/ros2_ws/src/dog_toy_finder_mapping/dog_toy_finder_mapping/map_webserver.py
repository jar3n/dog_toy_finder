"""Live 3D browser view of the robot's map.

Subscribes to /map (nav_msgs/OccupancyGrid) and /tracked_path (nav_msgs/Path)
and serves a self-refreshing page on port 8765. The page itself lives in
`web/` (HTML, CSS, and JavaScript); vendored Three.js files live in `static/`.
Python only publishes the map data and serves those files.

The page has a **Save PNG** button that downloads the current view to the
computer running the browser, and a **Reset view** button for the camera.

The server binds to 0.0.0.0, so any computer on the same network can open it
at http://<this-computer-ip>:8765. Three.js is served from the package itself,
so the viewing computer does not need internet access. This is the only
supported map viewer; it replaces the former terminal (ASCII) and PGM/YAML
export tools.

Usage:
    ros2 run dog_toy_finder_mapping map_webserver
    # open http://<computer-ip>:8765 in a browser
    # change the port with: --ros-args -p port:=9000
"""

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path as FilePath

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


WEB_DIR = FilePath(__file__).resolve().parent / "web"
STATIC_DIR = FilePath(__file__).resolve().parent / "static"

WEB_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}

STATIC_FILES = {
    "three.module.js": "text/javascript; charset=utf-8",
    "OrbitControls.js": "text/javascript; charset=utf-8",
    "LICENSE.three": "text/plain; charset=utf-8",
}


def _local_ip() -> str:
    """Return the LAN IP used for outbound traffic, or 127.0.0.1 offline."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


class MapWebServer(Node):
    def __init__(self):
        super().__init__("map_webserver")
        self.declare_parameter("port", 8765)
        self.port = int(self.get_parameter("port").value)
        self._lock = threading.Lock()
        self._map = None
        self._path = None

        self.create_subscription(OccupancyGrid, "/map", self.on_map,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "/tracked_path", self.on_path,
                                 qos_profile_sensor_data)

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.port), self._handler_factory())
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"Live map ready at http://{_local_ip()}:{self.port} "
            f"(listening on 0.0.0.0:{self.port})")

    def _handler_factory(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send(self, status, body, content_type="text/plain; charset=utf-8"):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def _send_file(self, directory, name, content_type):
                try:
                    body = (directory / name).read_bytes()
                except OSError:
                    self._send(404, b"not found")
                    return
                self._send(200, body, content_type)

            def do_GET(self):
                path = self.path.split("?", 1)[0]

                if path in WEB_FILES:
                    name, content_type = WEB_FILES[path]
                    self._send_file(WEB_DIR, name, content_type)
                elif path == "/map.json":
                    with node._lock:
                        data = node._map
                    if data is None:
                        self._send(404, b"no map yet")
                        return
                    self._send(200, json.dumps(data).encode("utf-8"),
                               "application/json")
                elif path == "/path.json":
                    with node._lock:
                        data = node._path
                    if data is None:
                        data = {"points": []}
                    self._send(200, json.dumps(data).encode("utf-8"),
                               "application/json")
                elif path.startswith("/static/"):
                    name = path[len("/static/"):]
                    content_type = STATIC_FILES.get(name)
                    if content_type is None:
                        self._send(404, b"not found")
                        return
                    self._send_file(STATIC_DIR, name, content_type)
                else:
                    self._send(404, b"not found")

            def log_message(self, *args):
                pass

        return Handler

    def on_map(self, msg):
        info = msg.info
        with self._lock:
            self._map = {
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "resolution": info.resolution,
                "width": info.width,
                "height": info.height,
                "origin": {"x": info.origin.position.x,
                           "y": info.origin.position.y},
                "data": list(msg.data),
            }

    def on_path(self, msg):
        pts = []
        for ps in msg.poses:
            pts.append({"x": ps.pose.position.x, "y": ps.pose.position.y})
        with self._lock:
            self._path = {"points": pts}

    def destroy_node(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapWebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
