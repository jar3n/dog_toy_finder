"""Live 3D browser view of the robot's map.

Subscribes to /map (nav_msgs/OccupancyGrid) and /tracked_path (nav_msgs/Path)
and serves a self-refreshing page on port 8765. Occupied cells are drawn as
raised blocks, free cells as a flat plane, and the robot's travelled path as a
line. The page has a **Save PNG** button that downloads the current view to
the computer running the browser.

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


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Dog Toy Finder - Live Map</title>
<style>
  html, body { margin: 0; height: 100%; overflow: hidden; background: #111;
               color: #eee; font-family: system-ui, sans-serif; }
  canvas { display: block; }
  #bar { position: absolute; top: 10px; left: 10px; right: 10px;
         display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
  #info { background: rgba(0,0,0,0.65); padding: 8px 12px; border-radius: 6px;
          font-size: 13px; }
  #actions { margin-left: auto; display: flex; gap: 8px; }
  button { background: #2d6cdf; color: #fff; border: 0; border-radius: 6px;
           padding: 9px 16px; font-size: 13px; cursor: pointer; }
  button:hover { background: #3b7ef0; }
  button.secondary { background: #444; }
  button.secondary:hover { background: #555; }
  #legend { position: absolute; bottom: 10px; left: 10px;
            background: rgba(0,0,0,0.65); padding: 6px 10px; border-radius: 6px;
            font-size: 12px; }
</style>
</head>
<body>
<div id="bar">
  <div id="info">Waiting for map&hellip;</div>
  <div id="actions">
    <button id="save">Save PNG</button>
    <button id="reset" class="secondary">Reset view</button>
  </div>
</div>
<div id="legend">
  white = free floor &nbsp;&middot;&nbsp; red blocks = obstacles &nbsp;&middot;&nbsp;
  blue line = path travelled &nbsp;&middot;&nbsp; drag to orbit, scroll to zoom
</div>
<script type="importmap">
{ "imports": { "three": "/static/three.module.js" } }
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from '/static/OrbitControls.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(60, innerWidth/innerHeight, 0.01, 1000);
camera.position.set(0, 8, 8);
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(5, 10, 5);
scene.add(dir);

const group = new THREE.Group();
scene.add(group);
const info = document.getElementById('info');

function clearGroup() {
  while (group.children.length) {
    const obj = group.children.pop();
    obj.traverse(child => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        const mats = Array.isArray(child.material) ? child.material : [child.material];
        mats.forEach(m => m.dispose());
      }
    });
  }
}

function resetView() {
  camera.position.set(0, 8, 8);
  controls.target.set(0, 0, 0);
  controls.update();
}

function savePng() {
  renderer.render(scene, camera);
  renderer.domElement.toBlob(blob => {
    if (!blob) return;
    const d = new Date();
    const pad = n => String(n).padStart(2, '0');
    const stamp = `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}`
                + `-${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `dog-toy-map-${stamp}.png`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }, 'image/png');
}

document.getElementById('save').addEventListener('click', savePng);
document.getElementById('reset').addEventListener('click', resetView);

async function fetchJSON(url) {
  const r = await fetch(url, { cache: 'no-store' });
  if (!r.ok) return null;
  return r.json();
}

function rebuildMap(m, path) {
  clearGroup();

  const res = m.resolution;
  const w = m.width, h = m.height;
  const ox = m.origin.x, oy = m.origin.y;

  // Free floor plane.
  const planeGeo = new THREE.PlaneGeometry(w * res, h * res);
  const planeMat = new THREE.MeshStandardMaterial({ color: 0xdddddd, side: THREE.DoubleSide });
  const plane = new THREE.Mesh(planeGeo, planeMat);
  plane.rotation.x = -Math.PI / 2;
  plane.position.set(ox + w * res / 2, 0, oy + h * res / 2);
  group.add(plane);

  // Occupied cells as raised blocks.
  const positions = [];
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      if (m.data[r * w + c] >= 50) {
        positions.push(ox + (c + 0.5) * res, res, oy + (r + 0.5) * res);
      }
    }
  }
  if (positions.length) {
    const blockGeo = new THREE.BoxGeometry(res * 0.9, res * 2, res * 0.9);
    const blockMat = new THREE.MeshStandardMaterial({ color: 0xcc3333 });
    const mesh = new THREE.InstancedMesh(blockGeo, blockMat, positions.length / 3);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < positions.length / 3; i++) {
      dummy.position.set(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    }
    mesh.instanceMatrix.needsUpdate = true;
    group.add(mesh);
  }

  // Travelled path.
  if (path && path.points && path.points.length) {
    const pts = path.points.map(p => new THREE.Vector3(p.x, 0.05, p.y));
    const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
    const line = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: 0x3366ff }));
    group.add(line);
  }

  info.textContent = `Map ${w}\u00d7${h} @ ${res} m/cell`
                   + `  \u00b7  obstacles: ${positions.length / 3}`;
}

let lastMapStamp = 0;
let polling = false;
let lastPoll = 0;
const POLL_MS = 300;

async function loop(now) {
  if (!polling && now - lastPoll >= POLL_MS) {
    polling = true;
    lastPoll = now;
    try {
      const m = await fetchJSON('/map.json');
      if (m && m.stamp != null && m.stamp !== lastMapStamp) {
        lastMapStamp = m.stamp;
        const path = await fetchJSON('/path.json');
        rebuildMap(m, path);
      }
    } catch (e) {
      info.textContent = 'Map server unreachable; retrying\u2026';
    } finally {
      polling = false;
    }
  }
  requestAnimationFrame(loop);
}
requestAnimationFrame(loop);

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
</script>
</body>
</html>
"""


STATIC_DIR = FilePath(__file__).resolve().parent / "static"
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

            def do_GET(self):
                if self.path == "/":
                    self._send(200, HTML_PAGE.encode("utf-8"),
                               "text/html; charset=utf-8")
                elif self.path == "/map.json":
                    with node._lock:
                        data = node._map
                    if data is None:
                        self._send(404, b"no map yet")
                        return
                    self._send(200, json.dumps(data).encode("utf-8"),
                               "application/json")
                elif self.path == "/path.json":
                    with node._lock:
                        data = node._path
                    if data is None:
                        data = {"points": []}
                    self._send(200, json.dumps(data).encode("utf-8"),
                               "application/json")
                elif self.path.startswith("/static/"):
                    name = self.path[len("/static/"):].split("?", 1)[0]
                    content_type = STATIC_FILES.get(name)
                    if content_type is None:
                        self._send(404, b"not found")
                        return
                    try:
                        body = (STATIC_DIR / name).read_bytes()
                    except OSError:
                        self._send(404, b"not found")
                        return
                    self._send(200, body, content_type)
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
