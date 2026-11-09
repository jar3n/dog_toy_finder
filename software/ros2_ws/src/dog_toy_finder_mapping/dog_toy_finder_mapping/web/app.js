import * as THREE from 'three';
import { OrbitControls } from '/static/OrbitControls.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.01, 1000);
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
