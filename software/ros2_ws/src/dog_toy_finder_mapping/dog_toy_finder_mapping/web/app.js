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

// Occupancy values published by occupancy_mapper.
const LAYERS = [
  { key: 'ir', value: 100, color: 0xcc3333, height: 2.0 },
  { key: 'vision', value: 70, color: 0x3377dd, height: 1.6 },
  { key: 'both', value: 85, color: 0xffaa22, height: 2.6 },
  { key: 'toy', value: 60, color: 0x22bb66, height: 1.4 },
];

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

function rebuildMap(m, path, toys) {
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

  // One instanced mesh per occupied-source value: IR, vision, both, and toy.
  const counts = { ir: 0, vision: 0, both: 0, toy: 0 };
  for (const layer of LAYERS) {
    const positions = [];
    for (let r = 0; r < h; r++) {
      for (let c = 0; c < w; c++) {
        if (m.data[r * w + c] === layer.value) {
          positions.push(ox + (c + 0.5) * res, oy + (r + 0.5) * res);
        }
      }
    }
    counts[layer.key] = positions.length / 2;
    if (!positions.length) continue;

    const height = res * layer.height;
    const blockGeo = new THREE.BoxGeometry(res * 0.9, height, res * 0.9);
    const blockMat = new THREE.MeshStandardMaterial({ color: layer.color });
    const mesh = new THREE.InstancedMesh(blockGeo, blockMat, positions.length / 2);
    const dummy = new THREE.Object3D();
    for (let i = 0; i < positions.length / 2; i++) {
      dummy.position.set(positions[i * 2], height / 2, positions[i * 2 + 1]);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    }
    mesh.instanceMatrix.needsUpdate = true;
    group.add(mesh);
  }

  // Toy detections as bright green spheres above their occupied cells.
  if (toys && toys.length) {
    const toyRadius = Math.max(res * 1.5, 0.05);
    const toyGeo = new THREE.SphereGeometry(toyRadius, 16, 12);
    const toyMat = new THREE.MeshStandardMaterial({
      color: 0x99ffbb,
      emissive: 0x0a3a1a,
    });
    const mesh = new THREE.InstancedMesh(toyGeo, toyMat, toys.length);
    const dummy = new THREE.Object3D();
    toys.forEach((toy, i) => {
      dummy.position.set(toy.x, Math.max(res * 2.5, 0.12), toy.y);
      dummy.updateMatrix();
      mesh.setMatrixAt(i, dummy.matrix);
    });
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
                   + `  \u00b7  IR ${counts.ir}`
                   + `  \u00b7  vision ${counts.vision}`
                   + `  \u00b7  both ${counts.both}`
                   + `  \u00b7  toy cells ${counts.toy}`
                   + `  \u00b7  toys ${toys ? toys.length : 0}`;
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
        const toys = await fetchJSON('/toys.json');
        rebuildMap(m, path, toys ? toys.toys : []);
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
