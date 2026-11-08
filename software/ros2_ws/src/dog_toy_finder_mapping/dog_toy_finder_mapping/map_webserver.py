"""Simple web server that visualises the robot's map in 3D in a browser.

Subscribes to /map (nav_msgs/OccupancyGrid) and /tracked_path (nav_msgs/Path)
and serves an HTTP page on port 8765 with a Three.js 3D view. Occupied cells
are drawn as raised blocks, free cells as a flat plane, and the robot's
travelled path as a line.

Usage:
    ros2 run dog_toy_finder_mapping map_webserver
    # then open http://<computer-ip>:8765 in a browser
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dog Toy Finder - 3D Map</title>
<style>
  body { margin: 0; overflow: hidden; font-family: sans-serif; background: #111; color: #eee; }
  #info { position: absolute; top: 8px; left: 8px; background: rgba(0,0,0,0.6);
          padding: 8px 12px; border-radius: 6px; font-size: 13px; }
  #legend { position: absolute; bottom: 8px; left: 8px; background: rgba(0,0,0,0.6);
            padding: 6px 10px; border-radius: 6px; font-size: 12px; }
</style>
</head>
<body>
<div id="info">Waiting for map...</div>
<div id="legend">white = free floor, red blocks = obstacles, blue line = path travelled</div>
<script type="importmap">
{ "imports": { "three": "https://unpkg.com/three@0.160.0/build/three.module.js" } }
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.01, 1000);
camera.position.set(0, 8, 8);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(5, 10, 5);
scene.add(dir);

const group = new THREE.Group();
scene.add(group);

let lastMapStamp = 0;

async function fetchJSON(url) {
  const r = await fetch(url);
  if (!r.ok) return null;
  return r.json();
}

function rebuildMap(m) {
  // clear previous
  while (group.children.length) group.remove(group.children[0]);
  const res = m.resolution;
  const w = m.width, h = m.height;
  const ox = m.origin.x, oy = m.origin.y;
  // free floor plane
  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(w*res, h*res),
    new THREE.MeshStandardMaterial({ color: 0xdddddd, side: THREE.DoubleSide })
  );
  plane.rotation.x = -Math.PI/2;
  plane.position.set(ox + w*res/2, 0, oy + h*res/2);
  group.add(plane);
  // occupied blocks
  const blockGeo = new THREE.BoxGeometry(res*0.9, res*2, res*0.9);
  const blockMat = new THREE.MeshStandardMaterial({ color: 0xcc3333 });
  const inst = new THREE.InstancedMesh(blockGeo, blockMat, 1);
  inst.count = 0;
  const positions = [];
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      const v = m.data[r*w + c];
      if (v >= 50) {
        const x = ox + (c + 0.5)*res;
        const z = oy + (r + 0.5)*res;
        positions.push(x, res, z);
      }
    }
  }
  if (positions.length) {
    const mesh = new THREE.InstancedMesh(blockGeo, blockMat, positions.length/3);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < positions.length/3; i++) {
      dummy.position.set(positions[i*3], positions[i*3+1], positions[i*3+2]);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    }
    mesh.instanceMatrix.needsUpdate = true;
    group.add(mesh);
  }
  // path
  const path = await fetchJSON('/path.json');
  if (path && path.points && path.points.length) {
    const pts = path.points.map(p => new THREE.Vector3(p.x, 0.05, p.y));
    const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
    const line = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x3366ff, linewidth: 2 }));
    group.add(line);
  }
  document.getElementById('info').textContent =
    'Map ' + w + 'x' + h + ' @ ' + res + ' m  |  occupied cells: ' + (positions.length/3);
}

async function loop() {
  const m = await fetchJSON('/map.json');
  if (m && m.stamp && m.stamp !== lastMapStamp) {
    lastMapStamp = m.stamp;
    rebuildMap(m);
  }
  requestAnimationFrame(loop);
}
loop();

addEventListener('resize', () => {
  camera.aspect = innerWidth/innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
</script>
</body>
</html>
"""


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
        self.get_logger().info(f"Map web viewer running at http://0.0.0.0:{self.port}")

    def _handler_factory(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    body = HTML_PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/map.json":
                    with node._lock:
                        data = node._map
                    if data is None:
                        self.send_response(404)
                        self.end_headers()
                        return
                    body = json.dumps(data).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/path.json":
                    with node._lock:
                        data = node._path
                    if data is None:
                        data = {"points": []}
                    body = json.dumps(data).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

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
    rclpy.init()
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