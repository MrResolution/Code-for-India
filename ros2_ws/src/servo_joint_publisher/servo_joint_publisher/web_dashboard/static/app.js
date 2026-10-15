/**
 * Light-Themed WebGL Robot Arm Dashboard
 * ---------------------------------------
 * Powered by Three.js (WebGL 3D Engine) & REST/WebSocket Sync
 */

// Joint definitions matching 5-DOF robot arm without claw grip linkages
const JOINTS = [
  { id: 'turntable_link_joint_dup', name: 'Turntable Yaw', min: 0.0, max: 180.0, def: 90.0, rad_min: -3.0, rad_max: 3.0 },
  { id: 'turntable_link_joint', name: 'Shoulder Pitch', min: 16.6, max: 91.4, def: 59.0, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_1', name: 'Elbow Pitch 1', min: 30.4, max: 153.1, def: 153.1, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_2', name: 'Elbow Pitch 2', min: 0.0, max: 180.0, def: 116.0, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_3', name: 'Wrist Twist / Roll', min: 0.0, max: 180.0, def: 90.0, rad_min: -3.14159, rad_max: 3.14159 }
];

let scene, camera, renderer, controls;
let robotMeshes = {};
let jointState = {};
let renderedJointState = {};
let isSimulating = false;

// Quard Bot (Sesame Quadruped) 3D State
let quardMeshes = {
  container: null,
  root: null,
  joints: {}
};
let quardState = {
  channels: [90, 0, 0, 90, 90, 0, 90, 0],
  joints: {
    'joint_r1_hip': 0.0,
    'joint_r2_hip': 0.0,
    'joint_l1_hip': 0.0,
    'joint_l2_hip': 0.0,
    'joint_r4_foot': 0.0,
    'joint_r3_foot': 0.0,
    'joint_l3_foot': 0.0,
    'joint_l4_foot': 0.0,
  },
  gaitMode: null,
  gaitTimer: null,
  waveTimer: null
};

// Latest rover readings from /api/sensors/data (null = not received yet)
let sensorState = {};

// ── Model Load & Dual View Management System ──────────────────────────────
const modelLoadSystem = {
  mode: 'auto', // 'auto' | 'quard' | 'arm' | 'dual' | 'sensors' | 'dogu' (Dogu is loaded by loadDoguModel)
  loaded: {
    arm: false,
    quard: false,
    sensors: true
  },
  loading: {
    arm: false,
    quard: false
  },
  callbacks: {
    arm: [],
    quard: []
  }
};

// ── Request Coalescing ───────────────────────────────────────────────────
// Keeps at most one request in flight per endpoint and merges updates queued
// meanwhile, so fast slider drags cannot pile up or arrive out of order.
function createCoalescingPoster(url, merge, onResult) {
  let inFlight = false;
  let pending = null;

  function flush() {
    if (inFlight || pending === null) return;
    const body = pending;
    pending = null;
    inFlight = true;
    fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    })
      .then(res => res.json())
      .then(data => { if (onResult) onResult(data, null); })
      .catch(err => { if (onResult) onResult(null, err); })
      .finally(() => {
        inFlight = false;
        flush();
      });
  }

  return function post(update) {
    pending = merge(pending, update);
    flush();
  };
}

const postJointUpdate = createCoalescingPoster('/api/update_joints', (pending, update) => {
  const merged = pending || { joints: {} };
  Object.assign(merged.joints, update.joints);
  return merged;
});

// ── UI Helpers ───────────────────────────────────────────────────────────
// Non-blocking notifications. alert() would freeze polling and the 3D render
// loop while the dialog is open.
function toast(message, type = 'info', timeoutMs = 3500) {
  const stack = document.getElementById('toast-stack');
  if (!stack) return;
  const el = document.createElement('div');
  el.className = `toast ${type}`;
  el.setAttribute('role', type === 'error' ? 'alert' : 'status');
  const text = document.createElement('span');
  text.textContent = message;
  const close = document.createElement('button');
  close.className = 'toast-close';
  close.setAttribute('aria-label', 'Dismiss');
  close.textContent = '✕';
  el.append(text, close);
  stack.appendChild(el);
  while (stack.children.length > 4) stack.firstElementChild.remove();

  const dismiss = () => {
    el.classList.add('leaving');
    setTimeout(() => el.remove(), 200);
  };
  close.addEventListener('click', dismiss);
  if (timeoutMs > 0) setTimeout(dismiss, timeoutMs);
}

function postJSON(url, body) {
  return fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {})
  }).then(res => res.json());
}

function setPill(el, state, text) {
  if (!el) return;
  el.className = 'status-pill' + (state ? ` status-${state}` : '');
  el.removeAttribute('style');
  if (text !== undefined) el.innerText = text;
}

function setNodeBadge(badgeId, state, text) {
  const badge = document.getElementById(badgeId);
  if (!badge) return;
  badge.className = `node-badge status-${state}`;
  const strong = badge.querySelector('strong');
  if (strong && text !== undefined) strong.innerText = text;
}

// Shows a spinner on a button until the promise settles.
function withBusy(btn, promise) {
  if (!btn) return promise;
  btn.classList.add('is-busy');
  btn.disabled = true;
  return promise.finally(() => {
    btn.classList.remove('is-busy');
    btn.disabled = false;
  });
}

function storageGet(key) {
  try { return localStorage.getItem(key); } catch (e) { return null; }
}

function storageSet(key, value) {
  try { localStorage.setItem(key, value); } catch (e) {}
}

// ── Theme ────────────────────────────────────────────────────────────────
function isDarkTheme() {
  const forced = document.documentElement.dataset.theme;
  if (forced) return forced === 'dark';
  return window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
}

let gridHelper = null;

function applySceneTheme() {
  if (!scene) return;
  const dark = isDarkTheme();
  const bg = getComputedStyle(document.documentElement).getPropertyValue('--viewport-bg').trim() || '#e9eef5';
  scene.background = new THREE.Color(bg);
  if (gridHelper) {
    scene.remove(gridHelper);
    gridHelper.geometry.dispose();
  }
  gridHelper = dark
    ? new THREE.GridHelper(500, 50, 0x0ea5e9, 0x24324f)
    : new THREE.GridHelper(500, 50, 0x0284c7, 0xcbd5e1);
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);
}

function initTheme() {
  const btn = document.getElementById('btn-theme');
  if (btn) {
    btn.addEventListener('click', () => {
      const next = isDarkTheme() ? 'light' : 'dark';
      document.documentElement.dataset.theme = next;
      storageSet('dashboard-theme', next);
      applySceneTheme();
    });
  }
  if (window.matchMedia) {
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applySceneTheme);
  }
}

// ── E-STOP (software stop) ───────────────────────────────────────────────
// Halts the arm simulation, cancels smooth moves and Quard gaits, tells the
// Quard Bot to stand, and locks motion controls until released. It does not
// cut servo power; that needs a hardware switch.
let estopEngaged = false;

function engageEstop() {
  estopEngaged = true;
  document.body.classList.add('estop-engaged');
  const btn = document.getElementById('btn-estop');
  if (btn) {
    btn.classList.add('engaged');
    btn.innerText = '⛔ E-STOP ON';
  }

  cancelArmSmoothMove();
  if (isSimulating) setArmSimulating(false);
  stopQuardGait();
  if (quardState.waveTimer) {
    clearInterval(quardState.waveTimer);
    quardState.waveTimer = null;
  }
  if (quardSmoothAnimId) {
    cancelAnimationFrame(quardSmoothAnimId);
    quardSmoothAnimId = null;
  }
  quardCmdQueue.length = 0;
  sendQuardCmd({ mode: 'stand' }, true);
  setGaitLabel('stand');
  document.querySelectorAll('.btn-dpad').forEach(b => b.classList.toggle('active', b.dataset.cmd === 'stand'));
  doguStop();
}

function releaseEstop() {
  estopEngaged = false;
  document.body.classList.remove('estop-engaged');
  const btn = document.getElementById('btn-estop');
  if (btn) {
    btn.classList.remove('engaged');
    btn.innerText = '⛔ E-STOP';
  }
  toast('E-STOP released. Motion controls unlocked.', 'success');
}

function initEstop() {
  const btn = document.getElementById('btn-estop');
  const release = document.getElementById('btn-estop-release');
  if (btn) btn.addEventListener('click', () => (estopEngaged ? releaseEstop() : engageEstop()));
  if (release) release.addEventListener('click', releaseEstop);
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && !estopEngaged) {
      e.preventDefault();
      engageEstop();
    }
  });
}

// ── Twin Orientation Synchronization System ──────────────────────────────
let syncCalibration = {};

// Calibration entry for an arm joint; the wrist may be stored under its old name.
function getJointCalibration(jointId) {
  const direct = syncCalibration[jointId] || {};
  const alias = jointId === 'turntable_link_joint_dup_3' ? (syncCalibration.wrist_twist_joint || {}) : {};
  return Object.assign({}, alias, direct);
}

function getJointSyncOffset(jointId) {
  if (syncCalibration[jointId] && syncCalibration[jointId].sync_offset_deg !== undefined) {
    return parseFloat(syncCalibration[jointId].sync_offset_deg) || 0.0;
  }
  return 0.0;
}

function getQuardSyncOffset(ch) {
  if (syncCalibration.quard_offsets && syncCalibration.quard_offsets[ch] !== undefined) {
    return parseFloat(syncCalibration.quard_offsets[ch]) || 0.0;
  }
  return 0.0;
}

function getImuOffsets() {
  return syncCalibration.imu_offsets || { pitch: 0.0, roll: 0.0, yaw: 0.0 };
}

function loadSyncCalibration(onDone) {
  fetch('/api/calibration')
    .then(res => res.json())
    .then(data => {
      syncCalibration = data || {};
      updateSyncReadouts();
      if (typeof onDone === 'function') onDone();
    })
    .catch(() => {});
}

// Show the saved limits and home pose instead of the built-in defaults, so
// Save Config does not overwrite the file with them.
function applySavedJointCalibration() {
  JOINTS.forEach(j => {
    const cal = getJointCalibration(j.id);
    const lo = parseFloat(cal.servo_min_deg);
    const hi = parseFloat(cal.servo_max_deg);
    const home = parseFloat(cal.home_deg);
    if (Number.isFinite(lo) && Number.isFinite(hi) && lo < hi) {
      document.getElementById(`spin-min-${j.id}`).value = lo.toFixed(1);
      document.getElementById(`spin-max-${j.id}`).value = hi.toFixed(1);
    }
    if (Number.isFinite(home)) j.def = Math.max(0, Math.min(180, home));
    updateLimitBand(j.id);
  });
}

function updateSyncReadouts() {
  JOINTS.forEach(j => {
    const el = document.getElementById(`sync-val-${j.id}`);
    if (el) {
      const off = getJointSyncOffset(j.id);
      el.innerText = `${off >= 0 ? '+' : ''}${off.toFixed(1)}°`;
    }
  });
}

function setJointSyncOffset(jointId, newOffset) {
  if (!syncCalibration[jointId]) syncCalibration[jointId] = {};
  syncCalibration[jointId].sync_offset_deg = Math.round(newOffset * 10) / 10.0;
  updateSyncReadouts();

  fetch('/api/sync_offset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ joint: jointId, offset: syncCalibration[jointId].sync_offset_deg })
  }).catch(() => {});
}

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  initThreeJS();
  initTheme();
  buildJointControls();
  setupTabs();
  initModelLoadSystem();
  initQuardBotControls();
  initSensorControls();
  setupEventListeners();
  initEstop();
  initLinkControls();
  initDoguControls();
  loadSyncCalibration(applySavedJointCalibration);
  startStatePolling();
  startConsolePolling();
  startSensorPolling();
  restoreActiveTab();
});

// ── Three.js 3D Viewport Setup ─────────────────────────────────────────────
function initThreeJS() {
  const container = document.getElementById('canvas-container');
  const width = container.clientWidth;
  const height = container.clientHeight;

  // Scene
  scene = new THREE.Scene();

  // Camera
  camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 1000);
  camera.position.set(350, 250, 350);

  // Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);

  // Orbit Controls
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, 80, 0);
  controls.update();

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambientLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(200, 400, 200);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  scene.add(dirLight);

  const dirLight2 = new THREE.DirectionalLight(0xe2e8f0, 0.4);
  dirLight2.position.set(-200, 100, -200);
  scene.add(dirLight2);

  // Background + grid floor, coloured for the current theme
  applySceneTheme();

  // Initialize procedural IMU Sensor Visual
  createSensorVisual();

  // The canvas also changes size when the responsive layout reflows, not only
  // on window resize.
  if (window.ResizeObserver) {
    new ResizeObserver(onWindowResize).observe(container);
  } else {
    window.addEventListener('resize', onWindowResize);
  }

  // Animation Render Loop
  animate();
}

function createSensorVisual() {
  const boxGeom = new THREE.BoxGeometry(70, 12, 45);
  const boxMat = new THREE.MeshStandardMaterial({ color: 0x2f6f4f, roughness: 0.55, metalness: 0.15 }); // PCB green; no env map, so keep it matte
  const sensorMesh = new THREE.Mesh(boxGeom, boxMat);
  sensorMesh.castShadow = true;

  const chipGeom = new THREE.BoxGeometry(18, 4, 18);
  const chipMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.4, metalness: 0.3 });
  const chipMesh = new THREE.Mesh(chipGeom, chipMat);
  chipMesh.position.set(0, 8, 0);
  sensorMesh.add(chipMesh);

  robotMeshes.sensorGroup = new THREE.Group();
  robotMeshes.sensorGroup.position.set(0, 50, 0);
  robotMeshes.sensorGroup.add(sensorMesh);
  robotMeshes.sensorGroup.visible = false;
  scene.add(robotMeshes.sensorGroup);
}

function loadRobotModel(onComplete) {
  if (modelLoadSystem.loaded.arm) {
    if (typeof onComplete === 'function') onComplete();
    return;
  }
  if (modelLoadSystem.loading.arm) {
    if (typeof onComplete === 'function') modelLoadSystem.callbacks.arm.push(onComplete);
    return;
  }
  modelLoadSystem.loading.arm = true;
  if (typeof onComplete === 'function') modelLoadSystem.callbacks.arm.push(onComplete);
  refreshModelBadge();

  const loader = new THREE.STLLoader();

  // Create kinematic joint groups
  robotMeshes.base = new THREE.Group();
  robotMeshes.turntable = new THREE.Group();
  robotMeshes.shoulder = new THREE.Group();
  robotMeshes.forearm = new THREE.Group();
  robotMeshes.wrist = new THREE.Group();
  robotMeshes.gripper = new THREE.Group();
  // Mount gripper group at the exact wrist twist joint origin
  robotMeshes.gripper.position.set(-1.615, 140.642, 40.411);

  scene.add(robotMeshes.base);
  robotMeshes.base.add(robotMeshes.turntable);
  robotMeshes.turntable.add(robotMeshes.shoulder);
  robotMeshes.shoulder.add(robotMeshes.forearm);
  robotMeshes.forearm.add(robotMeshes.wrist);
  robotMeshes.wrist.add(robotMeshes.gripper);

  // Materials (Vibrant light theme robot colors: Metallic Slate & Electric Blue)
  const matBase = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.4, metalness: 0.3 });
  const matArm = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.3, metalness: 0.2 });
  const matLink = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.5, metalness: 0.1 });
  const matGripper = new THREE.MeshStandardMaterial({ color: 0xea580c, roughness: 0.3, metalness: 0.2 });

  let loadedCount = 0;
  const totalCount = 6;
  const onPartLoaded = () => {
    loadedCount++;
    if (loadedCount >= totalCount) {
      modelLoadSystem.loaded.arm = true;
      modelLoadSystem.loading.arm = false;
      refreshModelBadge();
      updateModelVisibilityAndCamera(false);
      const cbs = [...modelLoadSystem.callbacks.arm];
      modelLoadSystem.callbacks.arm = [];
      cbs.forEach(cb => { if (typeof cb === 'function') cb(); });
    }
  };

  // 1. Base Mesh
  loader.load('/meshes/part_3_obj3.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matBase);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    robotMeshes.base.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // 2. Turntable Mesh
  loader.load('/meshes/part_44_obj44.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matArm);
    mesh.castShadow = true;
    mesh.position.set(0, 45, 0);
    robotMeshes.turntable.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // 3. Shoulder Link Mesh
  loader.load('/meshes/part_7_obj7.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matLink);
    mesh.castShadow = true;
    mesh.position.set(0, 90, 0);
    robotMeshes.shoulder.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // 4. Forearm Link Mesh
  loader.load('/meshes/part_9_obj9.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matArm);
    mesh.castShadow = true;
    mesh.position.set(0, 190, 0);
    robotMeshes.forearm.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // 5. Wrist Mesh
  loader.load('/meshes/Robot+Arm+Wrist.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matLink);
    mesh.castShadow = true;
    mesh.position.set(0, 140, 0);
    robotMeshes.wrist.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // 6. Gripper Base Mesh
  loader.load('/meshes/gripper_base.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matGripper);
    mesh.castShadow = true;
    mesh.rotation.set(0, 0, 0);
    mesh.position.set(0, 0, 0);
    robotMeshes.gripper.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());
}

function loadQuardBotModel(onComplete) {
  if (modelLoadSystem.loaded.quard) {
    if (typeof onComplete === 'function') onComplete();
    return;
  }
  if (modelLoadSystem.loading.quard) {
    if (typeof onComplete === 'function') modelLoadSystem.callbacks.quard.push(onComplete);
    return;
  }
  modelLoadSystem.loading.quard = true;
  if (typeof onComplete === 'function') modelLoadSystem.callbacks.quard.push(onComplete);
  refreshModelBadge();

  const loader = new THREE.STLLoader();

  quardMeshes.container = new THREE.Group();
  quardMeshes.container.name = 'quard_container';
  quardMeshes.container.position.set(0, 48, 0); // Elevation so feet rest at Y = 0 on grid
  quardMeshes.container.rotation.y = -Math.PI / 4; // 3/4 beauty view

  quardMeshes.root = new THREE.Group();
  quardMeshes.root.name = 'quard_root';
  // Map ROS frame (Z up, X fwd, Y left) to Three.js (Y up, Z south, X east)
  quardMeshes.root.rotation.x = -Math.PI / 2;
  quardMeshes.container.add(quardMeshes.root);

  quardMeshes.container.visible = false;
  scene.add(quardMeshes.container);

  // Vibrant materials matching URDF
  const matChassis = new THREE.MeshStandardMaterial({ color: 0x59a694, roughness: 0.35, metalness: 0.25 }); // Teal chassis
  const matR1 = new THREE.MeshStandardMaterial({ color: 0x618cb8, roughness: 0.35, metalness: 0.25 });      // Blue R1
  const matR3 = new THREE.MeshStandardMaterial({ color: 0xb88066, roughness: 0.35, metalness: 0.25 });      // Orange R3
  const matR2 = new THREE.MeshStandardMaterial({ color: 0x73ad80, roughness: 0.35, metalness: 0.25 });      // Green R2
  const matHip = new THREE.MeshStandardMaterial({ color: 0x9e709e, roughness: 0.35, metalness: 0.25 });     // Purple Hips (L1, L2)
  const matFoot = new THREE.MeshStandardMaterial({ color: 0x806b94, roughness: 0.35, metalness: 0.25 });    // Purple Feet (R4, L3, L4)

  let loadedCount = 0;
  const totalCount = 9;
  const onPartLoaded = () => {
    loadedCount++;
    if (loadedCount >= totalCount) {
      modelLoadSystem.loaded.quard = true;
      modelLoadSystem.loading.quard = false;
      refreshModelBadge();
      updateModelVisibilityAndCamera(false);
      const cbs = [...modelLoadSystem.callbacks.quard];
      modelLoadSystem.callbacks.quard = [];
      cbs.forEach(cb => { if (typeof cb === 'function') cb(); });
    }
  };

  // 1. Chassis Mesh
  loader.load('/meshes/chassis.stl', (geom) => {
    geom.computeVertexNormals();
    geom.scale(1000, 1000, 1000);
    const mesh = new THREE.Mesh(geom, matChassis);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    quardMeshes.root.add(mesh);
    onPartLoaded();
  }, undefined, () => onPartLoaded());

  // Hip specs from quard_bot.urdf (positions in mm, rotation order ZYX)
  const hipSpecs = [
    { id: 'joint_l1_hip', file: 'L1.stl', mat: matHip, pos: [24.639, 25.830, 20.363], rpy: [0, 0, 0] },
    { id: 'joint_l2_hip', file: 'L2.stl', mat: matHip, pos: [-23.953, 25.833, 20.363], rpy: [0, 0, Math.PI] },
    { id: 'joint_r1_hip', file: 'R1.stl', mat: matR1,  pos: [24.649, -25.106, 20.363], rpy: [0, 0, 0] },
    { id: 'joint_r2_hip', file: 'R2.stl', mat: matR2,  pos: [-24.034, -24.987, 20.363], rpy: [0, 0, Math.PI] },
  ];

  const footSpecs = [
    { id: 'joint_l3_foot', parent: 'joint_l1_hip', file: 'L3.stl', mat: matFoot, pos: [-9.338, 36.884, -19.524], rpy: [1.57080, 0, 1.57080] },
    { id: 'joint_l4_foot', parent: 'joint_l2_hip', file: 'L4.stl', mat: matFoot, pos: [-9.349, -36.870, -19.547], rpy: [1.55975, 0, 1.57080] },
    { id: 'joint_r3_foot', parent: 'joint_r1_hip', file: 'R3.stl', mat: matR3,   pos: [-9.414, -36.856, -19.531], rpy: [1.55156, 0, 1.57080] },
    { id: 'joint_r4_foot', parent: 'joint_r2_hip', file: 'R4.stl', mat: matFoot, pos: [-9.550, 36.735, -19.562], rpy: [1.55156, 0, 1.57080] },
  ];

  hipSpecs.forEach(spec => {
    const anchor = new THREE.Group();
    anchor.position.set(spec.pos[0], spec.pos[1], spec.pos[2]);
    anchor.rotation.order = 'ZYX';
    anchor.rotation.set(spec.rpy[0], spec.rpy[1], spec.rpy[2]);

    const rotGroup = new THREE.Group();
    anchor.add(rotGroup);
    quardMeshes.root.add(anchor);

    quardMeshes.joints[spec.id] = { anchor: anchor, rotGroup: rotGroup };

    loader.load('/meshes/' + spec.file, (geom) => {
      geom.computeVertexNormals();
      geom.scale(1000, 1000, 1000);
      const mesh = new THREE.Mesh(geom, spec.mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      rotGroup.add(mesh);
      onPartLoaded();
    }, undefined, () => onPartLoaded());
  });

  footSpecs.forEach(spec => {
    const parentHip = quardMeshes.joints[spec.parent];
    const anchor = new THREE.Group();
    anchor.position.set(spec.pos[0], spec.pos[1], spec.pos[2]);
    anchor.rotation.order = 'ZYX';
    anchor.rotation.set(spec.rpy[0], spec.rpy[1], spec.rpy[2]);

    const rotGroup = new THREE.Group();
    anchor.add(rotGroup);
    if (parentHip && parentHip.rotGroup) {
      parentHip.rotGroup.add(anchor);
    } else {
      quardMeshes.root.add(anchor);
    }

    quardMeshes.joints[spec.id] = { anchor: anchor, rotGroup: rotGroup };

    loader.load('/meshes/' + spec.file, (geom) => {
      geom.computeVertexNormals();
      geom.scale(1000, 1000, 1000);
      const mesh = new THREE.Mesh(geom, spec.mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      rotGroup.add(mesh);
      onPartLoaded();
    }, undefined, () => onPartLoaded());
  });
}

const CHANNEL_TO_JOINT = {
  0: { id: 'joint_r1_hip', sign: 1.0,  isHip: true,  name: 'R1 (Right Front Hip)', color: '#618cb8' },
  1: { id: 'joint_r2_hip', sign: -1.0, isHip: true,  name: 'R2 (Right Rear Hip)',  color: '#73ad80' },
  2: { id: 'joint_l1_hip', sign: -1.0, isHip: true,  name: 'L1 (Left Front Hip)',  color: '#9e709e' },
  3: { id: 'joint_l2_hip', sign: 1.0,  isHip: true,  name: 'L2 (Left Rear Hip)',   color: '#9e709e' },
  4: { id: 'joint_r4_foot', sign: -1.0, isHip: false, name: 'R4 (Right Rear Foot)', color: '#806b94' },
  5: { id: 'joint_r3_foot', sign: 1.0,  isHip: false, name: 'R3 (Right Front Foot)', color: '#b88066' },
  6: { id: 'joint_l3_foot', sign: -1.0, isHip: false, name: 'L3 (Left Front Foot)',  color: '#806b94' },
  7: { id: 'joint_l4_foot', sign: 1.0,  isHip: false, name: 'L4 (Left Rear Foot)',   color: '#806b94' },
};

function channelAngleToRad(ch, angleDeg) {
  const info = CHANNEL_TO_JOINT[ch];
  if (!info) return 0.0;
  // Base zero (0 deg) corresponds to straight neutral URDF (0.0 rad)
  const syncOff = getQuardSyncOffset(ch);
  const totalDeg = angleDeg + syncOff;
  let rad = (totalDeg * Math.PI / 180.0) * info.sign;
  return Math.max(-1.2, Math.min(1.2, rad));
}

function updateQuardKinematics() {
  for (const jId in quardState.joints) {
    const jointObj = quardMeshes.joints[jId];
    if (jointObj && jointObj.rotGroup) {
      jointObj.rotGroup.rotation.z = quardState.joints[jId];
    }
  }
}

const postQuardJoints = createCoalescingPoster('/api/quardbot/joints', (pending, update) => update);

function syncQuardJointsToServer() {
  postQuardJoints({
    joints: Object.assign({}, quardState.joints),
    channels: quardState.channels.slice()
  });
}

// ── Model Loading & Dual View Coordination ───────────────────────────────
function getEffectiveModelMode() {
  if (modelLoadSystem.mode === 'auto') {
    if (activeTab === 'quard') return 'quard';
    if (activeTab === 'sensors') return 'sensors';
    if (activeTab === 'dogu') return 'dogu';
    return 'arm';
  }
  return modelLoadSystem.mode;
}

function updateModelBadge(statusText, extraClass = '') {
  const badge = document.getElementById('badge-model-status');
  if (!badge) return;
  badge.className = 'model-status-badge' + (extraClass ? ' ' + extraClass : '');
  badge.innerText = statusText;
}

function refreshModelBadge() {
  const effectiveMode = getEffectiveModelMode();
  if (effectiveMode === 'dogu') {
    updateModelBadge(dogu.loaded ? 'Loaded: 🐕 Dogu (preview)' : '⏳ Loading Dogu...', dogu.loaded ? '' : 'badge-loading');
    return;
  }
  if (effectiveMode === 'dual') {
    if (!modelLoadSystem.loaded.arm || !modelLoadSystem.loaded.quard) {
      updateModelBadge('⏳ Loading Both...', 'badge-loading');
    } else {
      updateModelBadge('⚡ Dual View (Both Bots)', 'badge-dual');
    }
  } else if (effectiveMode === 'quard') {
    if (!modelLoadSystem.loaded.quard) {
      updateModelBadge('⏳ Loading Quard...', 'badge-loading');
    } else {
      updateModelBadge('Loaded: 🤖 Quard');
    }
  } else if (effectiveMode === 'sensors') {
    updateModelBadge('Loaded: 🧭 ESP32 IMU');
  } else {
    // arm
    if (!modelLoadSystem.loaded.arm) {
      updateModelBadge('⏳ Loading Arm...', 'badge-loading');
    } else {
      updateModelBadge('Loaded: 🦾 Arm');
    }
  }
}

function setModelMode(mode, updateCamera = true) {
  modelLoadSystem.mode = mode;
  const selectMode = document.getElementById('select-model-mode');
  if (selectMode && selectMode.value !== mode) {
    selectMode.value = mode;
  }

  ensureRequiredModelsLoaded(() => {
    updateModelVisibilityAndCamera(updateCamera);
    refreshModelBadge();
  });
}

function ensureRequiredModelsLoaded(callback) {
  const effectiveMode = getEffectiveModelMode();
  if (effectiveMode === 'dogu') {
    loadDoguModel(callback);
    return;
  }

  if (effectiveMode === 'dual') {
    let pending = 0;
    const checkDone = () => {
      pending--;
      if (pending <= 0) {
        refreshModelBadge();
        if (callback) callback();
      }
    };
    if (!modelLoadSystem.loaded.arm) {
      pending++;
      loadRobotModel(checkDone);
    }
    if (!modelLoadSystem.loaded.quard) {
      pending++;
      loadQuardBotModel(checkDone);
    }
    if (pending === 0 && callback) callback();
  } else if (effectiveMode === 'quard') {
    if (!modelLoadSystem.loaded.quard) {
      loadQuardBotModel(() => {
        refreshModelBadge();
        if (callback) callback();
      });
    } else if (callback) {
      callback();
    }
  } else if (effectiveMode === 'arm') {
    if (!modelLoadSystem.loaded.arm) {
      loadRobotModel(() => {
        refreshModelBadge();
        if (callback) callback();
      });
    } else if (callback) {
      callback();
    }
  } else {
    // sensors
    if (callback) callback();
  }
  refreshModelBadge();
}

function updateModelVisibilityAndCamera(updateCamera = true) {
  const effectiveMode = getEffectiveModelMode();
  if (dogu.container) dogu.container.visible = effectiveMode === 'dogu';
  if (effectiveMode === 'dogu') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (updateCamera && camera && controls) {
      camera.position.set(260, 190, 260);
      controls.target.set(0, 70, 0);
      controls.update();
    }
    return;
  }

  if (effectiveMode === 'dual') {
    // Both visible side-by-side: Arm at +110mm, Quard at -110mm
    if (robotMeshes.base) {
      robotMeshes.base.visible = true;
      robotMeshes.base.position.set(110, 0, 0);
    }
    if (quardMeshes.container) {
      quardMeshes.container.visible = true;
      quardMeshes.container.position.set(-110, 48, 0);
    }
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;

    if (updateCamera && camera && controls) {
      camera.position.set(0, 220, 360);
      controls.target.set(0, 50, 0);
      controls.update();
    }
  } else if (effectiveMode === 'quard') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (quardMeshes.container) {
      quardMeshes.container.visible = true;
      quardMeshes.container.position.set(0, 48, 0);
    }
    if (updateCamera && camera && controls) {
      camera.position.set(160, 120, 160);
      controls.target.set(0, 35, 0);
      controls.update();
    }
  } else if (effectiveMode === 'sensors') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = true;
    if (updateCamera && camera && controls) {
      camera.position.set(140, 100, 140);
      controls.target.set(0, 50, 0);
      controls.update();
    }
  } else {
    // arm
    if (robotMeshes.base) {
      robotMeshes.base.visible = true;
      robotMeshes.base.position.set(0, 0, 0);
    }
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (updateCamera && camera && controls) {
      camera.position.set(350, 250, 350);
      controls.target.set(0, 80, 0);
      controls.update();
    }
  }
}

function initModelLoadSystem() {
  const selectMode = document.getElementById('select-model-mode');
  if (selectMode) {
    selectMode.value = modelLoadSystem.mode;
    selectMode.addEventListener('change', (e) => {
      setModelMode(e.target.value);
    });
  }

  const btnLoadArm = document.getElementById('btn-load-arm-view');
  if (btnLoadArm) {
    btnLoadArm.addEventListener('click', () => {
      const tabBtnArm = document.getElementById('tab-btn-arm');
      if (tabBtnArm) tabBtnArm.click();
      setModelMode('arm');
    });
  }

  const btnLoadQuard = document.getElementById('btn-load-quard-view');
  if (btnLoadQuard) {
    btnLoadQuard.addEventListener('click', () => {
      const tabBtnQuard = document.getElementById('tab-btn-quard');
      if (tabBtnQuard) tabBtnQuard.click();
      setModelMode('quard');
    });
  }

  const btnLoadSensors = document.getElementById('btn-load-sensor-view');
  if (btnLoadSensors) {
    btnLoadSensors.addEventListener('click', () => {
      const tabBtnSensors = document.getElementById('tab-btn-sensors');
      if (tabBtnSensors) tabBtnSensors.click();
      setModelMode('sensors');
    });
  }

  // Load initial model based on active section
  setModelMode('auto', false);
}

// Smooth 60 FPS joint interpolator to bridge 25 Hz poll ticks and eliminate jitter
function getSmoothJointDeg(jointId, defaultDeg = 90.0) {
  const targetDeg = jointState[jointId] !== undefined ? jointState[jointId] : defaultDeg;
  if (renderedJointState[jointId] === undefined) {
    renderedJointState[jointId] = targetDeg;
  } else {
    const factor = isSimulating ? 0.35 : 0.85;
    renderedJointState[jointId] += (targetDeg - renderedJointState[jointId]) * factor;
  }
  return renderedJointState[jointId];
}

let fpsFrames = 0;
let fpsLastTime = performance.now();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  tickDogu();
  if (dogu.container && getEffectiveModelMode() !== 'dogu') dogu.container.visible = false;

  fpsFrames++;
  const nowMs = performance.now();
  if (nowMs - fpsLastTime >= 1000) {
    const fpsEl = document.getElementById('lbl-fps');
    if (fpsEl) fpsEl.innerText = Math.round(fpsFrames * 1000 / (nowMs - fpsLastTime));
    fpsFrames = 0;
    fpsLastTime = nowMs;
  }

  const effectiveMode = getEffectiveModelMode();

  if (effectiveMode === 'dual') {
    // Dual View: Render and update both robots simultaneously!
    if (robotMeshes.base) robotMeshes.base.visible = true;
    if (quardMeshes.container) {
      quardMeshes.container.visible = true;
      updateQuardKinematics();
    }
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;

    // Articulate Arm (all 5 joints)
    if (robotMeshes.turntable) {
      const yawDeg = getSmoothJointDeg('turntable_link_joint_dup', 90.0);
      const shoulderDeg = getSmoothJointDeg('turntable_link_joint', 59.0);
      const elbow1Deg = getSmoothJointDeg('turntable_link_joint_dup_1', 153.1);
      const elbow2Deg = getSmoothJointDeg('turntable_link_joint_dup_2', 116.0);
      const wristDeg = getSmoothJointDeg('turntable_link_joint_dup_3', jointState['wrist_twist_joint'] !== undefined ? jointState['wrist_twist_joint'] : 90.0);

      robotMeshes.turntable.rotation.y = ((yawDeg + getJointSyncOffset('turntable_link_joint_dup')) - 90.0) * (Math.PI / 180.0);
      robotMeshes.shoulder.rotation.z = ((shoulderDeg + getJointSyncOffset('turntable_link_joint')) - 59.0) * (Math.PI / 180.0);
      robotMeshes.forearm.rotation.z = ((elbow1Deg + getJointSyncOffset('turntable_link_joint_dup_1')) - 153.1) * (Math.PI / 180.0);
      robotMeshes.wrist.rotation.z = ((elbow2Deg + getJointSyncOffset('turntable_link_joint_dup_2')) - 116.0) * (Math.PI / 180.0);
      robotMeshes.gripper.rotation.z = ((wristDeg + getJointSyncOffset('turntable_link_joint_dup_3')) - 90.0) * (Math.PI / 180.0);
    }
  } else if (effectiveMode === 'sensors') {
    // Mode: Sensor Node active - show 3D IMU module tilting with MPU6050 pitch/roll
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (robotMeshes.sensorGroup) {
      robotMeshes.sensorGroup.visible = true;
      const imuOff = getImuOffsets();
      // No reading yet: draw it level, rather than the tare offset applied to nothing
      const hasImu = Number.isFinite(sensorState.raw_pitch) && Number.isFinite(sensorState.raw_roll);
      const pRad = hasImu ? (sensorState.raw_pitch - (imuOff.pitch || 0)) * (Math.PI / 180.0) : 0;
      const rRad = hasImu ? (sensorState.raw_roll - (imuOff.roll || 0)) * (Math.PI / 180.0) : 0;
      robotMeshes.sensorGroup.rotation.x = pRad;
      robotMeshes.sensorGroup.rotation.z = -rRad;
    }
  } else if (effectiveMode === 'dogu') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (dogu.container) dogu.container.visible = true;
  } else if (effectiveMode === 'quard') {
    // Mode: Quard active - articulate 8 PCA9685 hip and knee channels
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (quardMeshes.container) {
      quardMeshes.container.visible = true;
      updateQuardKinematics();
    }
  } else {
    // Mode: Arm active - articulate 5 MG996R servo joints
    if (robotMeshes.base) robotMeshes.base.visible = true;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;

    if (robotMeshes.turntable) {
      const yawDeg = getSmoothJointDeg('turntable_link_joint_dup', 90.0);
      const shoulderDeg = getSmoothJointDeg('turntable_link_joint', 59.0);
      const elbow1Deg = getSmoothJointDeg('turntable_link_joint_dup_1', 153.1);
      const elbow2Deg = getSmoothJointDeg('turntable_link_joint_dup_2', 116.0);
      const wristDeg = getSmoothJointDeg('turntable_link_joint_dup_3', jointState['wrist_twist_joint'] !== undefined ? jointState['wrist_twist_joint'] : 90.0);

      robotMeshes.turntable.rotation.y = ((yawDeg + getJointSyncOffset('turntable_link_joint_dup')) - 90.0) * (Math.PI / 180.0);
      robotMeshes.shoulder.rotation.z = ((shoulderDeg + getJointSyncOffset('turntable_link_joint')) - 59.0) * (Math.PI / 180.0);
      robotMeshes.forearm.rotation.z = ((elbow1Deg + getJointSyncOffset('turntable_link_joint_dup_1')) - 153.1) * (Math.PI / 180.0);
      robotMeshes.wrist.rotation.z = ((elbow2Deg + getJointSyncOffset('turntable_link_joint_dup_2')) - 116.0) * (Math.PI / 180.0);
      robotMeshes.gripper.rotation.z = ((wristDeg + getJointSyncOffset('turntable_link_joint_dup_3')) - 90.0) * (Math.PI / 180.0);
    }
  }

  renderer.render(scene, camera);
}

function onWindowResize() {
  const container = document.getElementById('canvas-container');
  if (!container.clientWidth || !container.clientHeight) return;
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// ── Left Control Panel GUI Generator ─────────────────────────────────────
function buildJointControls() {
  const container = document.getElementById('joints-list');
  container.innerHTML = '';

  JOINTS.forEach((j, idx) => {
    jointState[j.id] = j.def;

    const card = document.createElement('div');
    card.className = 'joint-card';
    card.id = `card-${j.id}`;

    const syncOffset = getJointSyncOffset(j.id);
    const syncText = `${syncOffset >= 0 ? '+' : ''}${syncOffset.toFixed(1)}°`;

    card.innerHTML = `
      <div class="joint-card-header">
        <label class="joint-name" for="slider-${j.id}"><span class="joint-index">${idx + 1}</span>${j.name}</label>
        <span class="joint-readout" id="readout-${j.id}"></span>
      </div>
      <div class="slider-row">
        <div class="range-band" id="band-${j.id}" title="Saved safe range"></div>
        <input type="range" id="slider-${j.id}" min="0" max="1800" value="${j.def * 10}">
      </div>
      <div class="slider-scale"><span>0°</span><span>90°</span><span>180°</span></div>
      <div class="limits-row">
        <label for="spin-min-${j.id}">Min</label>
        <input type="number" id="spin-min-${j.id}" class="spinbox spinbox-min" value="${j.min.toFixed(1)}" step="1.0" min="0" max="180">
        <button class="btn btn-outline btn-xs btn-set-low" data-joint="${j.id}" title="Use the current angle as the minimum">📍 Set Low</button>
        <span class="spacer"></span>
        <label for="spin-max-${j.id}">Max</label>
        <input type="number" id="spin-max-${j.id}" class="spinbox spinbox-max" value="${j.max.toFixed(1)}" step="1.0" min="0" max="180">
        <button class="btn btn-outline btn-xs btn-set-high" data-joint="${j.id}" title="Use the current angle as the maximum">📍 Set High</button>
      </div>
      <div class="sync-row">
        <span class="sync-label" title="Offset between the 3D model and the physical servo">🎯 Twin offset</span>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="-5">-5°</button>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="-1">-1°</button>
        <span id="sync-val-${j.id}" class="readout-pill">${syncText}</span>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="1">+1°</button>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="5">+5°</button>
        <button class="btn btn-outline btn-xs btn-zero-sync" data-joint="${j.id}" title="Reset offset to 0°">0°</button>
      </div>
    `;

    container.appendChild(card);
    setJointReadout(j.id, j.def);

    // Event listener for Slider changes
    const slider = card.querySelector(`#slider-${j.id}`);
    slider.addEventListener('input', (e) => {
      cancelArmSmoothMove();
      const val = parseFloat(e.target.value) / 10.0;
      onSliderChanged(j.id, val);
    });

    // Set Low / Set High click handlers
    card.querySelector('.btn-set-low').addEventListener('click', () => {
      const curr = jointState[j.id];
      card.querySelector(`#spin-min-${j.id}`).value = curr.toFixed(1);
      updateLimitBand(j.id);
    });

    card.querySelector('.btn-set-high').addEventListener('click', () => {
      const curr = jointState[j.id];
      card.querySelector(`#spin-max-${j.id}`).value = curr.toFixed(1);
      updateLimitBand(j.id);
    });

    card.querySelectorAll('.spinbox').forEach(input => {
      input.addEventListener('input', () => updateLimitBand(j.id));
    });

    updateLimitBand(j.id);

    // Nudge and Zero Sync handlers
    card.querySelectorAll('.btn-nudge').forEach(btn => {
      btn.addEventListener('click', () => {
        const nudge = parseFloat(btn.dataset.nudge);
        const curr = getJointSyncOffset(j.id);
        setJointSyncOffset(j.id, curr + nudge);
      });
    });

    const zeroBtn = card.querySelector('.btn-zero-sync');
    if (zeroBtn) {
      zeroBtn.addEventListener('click', () => {
        setJointSyncOffset(j.id, 0.0);
      });
    }
  });
}

const lastLocalJointEdit = {};

function setJointReadout(jointId, deg) {
  const readout = document.getElementById(`readout-${jointId}`);
  const jointDef = JOINTS.find(j => j.id === jointId);
  const rad = degToRad(deg, jointDef ? jointDef.rad_min : -2.0, jointDef ? jointDef.rad_max : 2.0);
  if (readout) readout.innerText = `${deg.toFixed(1)}° · ${rad.toFixed(3)} rad`;

  const card = document.getElementById(`card-${jointId}`);
  if (card) {
    const lim = getJointLimits(jointId);
    const out = lim && (deg < lim.min - 0.05 || deg > lim.max + 0.05);
    card.classList.toggle('out-of-range', Boolean(out));
    if (readout) readout.title = out ? `Outside saved limits (${lim.min.toFixed(1)}° – ${lim.max.toFixed(1)}°)` : '';
  }
}

function getJointLimits(jointId) {
  const minEl = document.getElementById(`spin-min-${jointId}`);
  const maxEl = document.getElementById(`spin-max-${jointId}`);
  if (!minEl || !maxEl) return null;
  const min = parseFloat(minEl.value);
  const max = parseFloat(maxEl.value);
  if (!Number.isFinite(min) || !Number.isFinite(max) || min >= max) return null;
  return { min: Math.max(0, min), max: Math.min(180, max) };
}

function updateLimitBand(jointId) {
  const band = document.getElementById(`band-${jointId}`);
  const lim = getJointLimits(jointId);
  if (band) {
    band.style.display = lim ? '' : 'none';
    if (lim) {
      band.style.setProperty('--lo', lim.min / 180);
      band.style.setProperty('--hi', lim.max / 180);
    }
  }
  if (jointState[jointId] !== undefined) setJointReadout(jointId, jointState[jointId]);
}

function setSliderPosition(jointId, deg) {
  const slider = document.getElementById(`slider-${jointId}`);
  if (slider) slider.value = deg * 10;
  setJointReadout(jointId, deg);
}

// Eased move of several arm joints (like the Qt GUI's Go Home), so the
// physical servos are not slammed to a new pose.
let armSmoothAnimId = null;

function cancelArmSmoothMove() {
  if (armSmoothAnimId) cancelAnimationFrame(armSmoothAnimId);
  armSmoothAnimId = null;
}

function smoothMoveArm(targets, durationMs = 1500) {
  cancelArmSmoothMove();
  const start = {};
  for (const id in targets) start[id] = jointState[id];
  const t0 = performance.now();

  const step = (now) => {
    if (estopEngaged) {
      armSmoothAnimId = null;
      return;
    }
    const p = Math.min(1, (now - t0) / durationMs);
    const e = 0.5 - 0.5 * Math.cos(p * Math.PI);
    for (const id in targets) {
      const deg = Math.round((start[id] + (targets[id] - start[id]) * e) * 10) / 10;
      onSliderChanged(id, deg);
      setSliderPosition(id, deg);
    }
    armSmoothAnimId = p < 1 ? requestAnimationFrame(step) : null;
  };
  armSmoothAnimId = requestAnimationFrame(step);
}

function onSliderChanged(jointId, deg) {
  if (estopEngaged) {
    setSliderPosition(jointId, jointState[jointId]);
    return;
  }
  jointState[jointId] = deg;
  lastLocalJointEdit[jointId] = Date.now();
  setJointReadout(jointId, deg);

  // Push only the changed joint, so stale values of other joints held in
  // jointState can never overwrite the server's copy.
  postJointUpdate({ joints: { [jointId]: deg } });
}

function degToRad(deg, rMin, rMax) {
  const ratio = deg / 180.0;
  return rMin + ratio * (rMax - rMin);
}

// ── Setup Action Bar Event Listeners ─────────────────────────────────────
function setArmSimulating(on) {
  isSimulating = on;
  const btnSim = document.getElementById('btn-sim');
  btnSim.innerText = isSimulating ? '⏸️ Stop Sim' : '▶️ Start Sim';
  btnSim.className = isSimulating ? 'btn btn-danger btn-sm' : 'btn btn-warning btn-sm';

  // Synchronize rendered angles so transition has zero initial displacement
  for (let k in jointState) {
    renderedJointState[k] = jointState[k];
  }

  return postJSON('/api/toggle_sim', {
    simulating: isSimulating,
    current_joints: jointState
  }).catch(() => toast('Could not reach the server to toggle the simulation.', 'error'));
}

function syncCommModeUI(isWifi) {
  const toggle = document.getElementById('toggle-comm-mode');
  if (toggle) toggle.checked = isWifi;
  document.getElementById('lbl-wifi').classList.toggle('active', isWifi);
  document.getElementById('lbl-serial').classList.toggle('active', !isWifi);
}

function requestTare(target, btn) {
  return withBusy(btn, postJSON('/api/tare_pose', { target: target }))
    .then(data => {
      if (data.status !== 'ok') throw new Error(data.message || 'server error');
      syncCalibration = data.calibration || syncCalibration;
      updateSyncReadouts();
      return data;
    });
}

// ── Setup Action Bar Event Listeners ─────────────────────────────────────
function setupEventListeners() {
  // Start Sim Toggle
  document.getElementById('btn-sim').addEventListener('click', () => {
    cancelArmSmoothMove();
    setArmSimulating(!isSimulating);
  });

  // Sim Speed Slider
  const sliderSpeed = document.getElementById('slider-speed');
  sliderSpeed.addEventListener('input', (e) => {
    const mult = (parseFloat(e.target.value) / 10.0).toFixed(1);
    document.getElementById('lbl-speed-val').innerText = `${mult}x`;
    postJSON('/api/toggle_sim', { speed: mult }).catch(() => {});
  });

  // Go Home: eased move to the saved home pose
  document.getElementById('btn-home').addEventListener('click', () => {
    if (isSimulating) setArmSimulating(false);
    const targets = {};
    JOINTS.forEach(j => { targets[j.id] = j.def; });
    smoothMoveArm(targets);
  });

  // Set Home: current pose becomes the home pose (saved immediately)
  document.getElementById('btn-set-home').addEventListener('click', (e) => {
    const calib = {};
    JOINTS.forEach(j => {
      j.def = Math.round(jointState[j.id] * 10) / 10;
      calib[j.id] = { home_deg: j.def };
    });
    withBusy(e.currentTarget, postJSON('/api/save_calibration', calib))
      .then(data => {
        if (data.status === 'ok') toast('📌 Current pose saved as home.', 'success');
        else toast(`Home not saved: ${data.message || 'server error'}`, 'error');
      })
      .catch(() => toast('Home not saved: server unreachable.', 'error'));
  });

  // Center All
  document.getElementById('btn-center').addEventListener('click', () => {
    if (isSimulating) setArmSimulating(false);
    const targets = {};
    JOINTS.forEach(j => { targets[j.id] = 90.0; });
    smoothMoveArm(targets);
  });

  // Save Config
  document.getElementById('btn-save').addEventListener('click', (e) => {
    const calib = {};
    const invalid = [];
    JOINTS.forEach(j => {
      const minVal = parseFloat(document.getElementById(`spin-min-${j.id}`).value);
      const maxVal = parseFloat(document.getElementById(`spin-max-${j.id}`).value);
      if (!Number.isFinite(minVal) || !Number.isFinite(maxVal) ||
          minVal < 0 || maxVal > 180 || minVal >= maxVal) {
        invalid.push(j.name);
        return;
      }
      calib[j.id] = {
        servo_min_deg: minVal,
        servo_max_deg: maxVal,
        home_deg: Math.max(minVal, Math.min(maxVal, jointState[j.id]))
      };
    });

    if (invalid.length > 0) {
      toast(`Not saved. Need 0 ≤ Min < Max ≤ 180 for: ${invalid.join(', ')}`, 'error', 6000);
      return;
    }

    withBusy(e.currentTarget, postJSON('/api/save_calibration', calib))
      .then(data => {
        if (data.status === 'ok') {
          JOINTS.forEach(j => { j.def = calib[j.id].home_deg; });
          toast('💾 Limits and home pose saved to servo_calibration.json', 'success');
        } else {
          toast(`Save failed: ${data.message || 'server error'}`, 'error', 6000);
        }
      })
      .catch(() => toast('Save failed: server unreachable.', 'error', 6000));
  });

  // 🎯 Sync Virtual to Physical Robot Arm
  const btnSyncArm = document.getElementById('btn-sync-all-arm');
  if (btnSyncArm) {
    btnSyncArm.addEventListener('click', () => {
      requestTare('arm', btnSyncArm)
        .then(() => toast('🎯 Twin aligned to the physical arm.', 'success'))
        .catch(err => toast(`Sync failed: ${err.message}`, 'error'));
    });
  }

  // 🎯 Tare IMU Horizon
  const btnTareImu = document.getElementById('btn-tare-imu');
  if (btnTareImu) {
    btnTareImu.addEventListener('click', () => {
      requestTare('imu', btnTareImu)
        .then(() => {
          document.getElementById('val-pitch').innerText = '0.0°';
          document.getElementById('val-roll').innerText = '0.0°';
          toast('🎯 IMU tared: pitch and roll now read 0.0°.', 'success');
        })
        .catch(err => toast(`IMU tare failed: ${err.message}`, 'error'));
    });
  }

  // 🎯 Quick Sync Button in 3D Viewport Header
  const btnQuickSync = document.getElementById('btn-quick-sync-vp');
  if (btnQuickSync) {
    btnQuickSync.addEventListener('click', () => {
      const mode = getEffectiveModelMode();
      const target = (mode === 'sensors') ? 'imu' : ((mode === 'quard') ? 'quard' : 'arm');
      requestTare(target, btnQuickSync)
        .then(() => toast(`🎯 ${target.toUpperCase()} twin synchronized.`, 'success'))
        .catch(err => toast(`Twin sync failed: ${err.message}`, 'error'));
    });
  }

  // Comm Mode Toggle Switch Listener
  const toggleCommMode = document.getElementById('toggle-comm-mode');
  if (toggleCommMode) {
    toggleCommMode.addEventListener('change', (e) => {
      const isWifi = e.target.checked;
      const mode = isWifi ? 'WIFI' : 'SERIAL';
      syncCommModeUI(isWifi);
      commModeEditedAt = Date.now();

      postJSON('/api/comm_mode', { mode: mode })
        .then(data => {
          if (data.status === 'ok') toast(`Arm link switched to ${isWifi ? 'Wi-Fi' : 'serial'}.`, 'info');
          else toast(`Could not switch link: ${data.message || 'server error'}`, 'error');
        })
        .catch(() => toast('Could not switch link: server unreachable.', 'error'));
    });
  }

  // Camera Resets
  document.getElementById('btn-reset-cam').addEventListener('click', () => {
    const effectiveMode = getEffectiveModelMode();
    if (effectiveMode === 'dogu') {
      camera.position.set(260, 190, 260);
      controls.target.set(0, 70, 0);
    } else if (effectiveMode === 'dual') {
      camera.position.set(0, 220, 360);
      controls.target.set(0, 50, 0);
    } else if (effectiveMode === 'quard') {
      camera.position.set(160, 120, 160);
      controls.target.set(0, 35, 0);
    } else if (effectiveMode === 'sensors') {
      camera.position.set(140, 100, 140);
      controls.target.set(0, 50, 0);
    } else {
      camera.position.set(350, 250, 350);
      controls.target.set(0, 80, 0);
    }
    controls.update();
  });

  document.getElementById('btn-top-cam').addEventListener('click', () => {
    const effectiveMode = getEffectiveModelMode();
    if (effectiveMode === 'dogu') {
      camera.position.set(0, 420, 0.1);
      controls.target.set(0, 70, 0);
    } else if (effectiveMode === 'dual') {
      camera.position.set(0, 480, 0.1);
      controls.target.set(0, 50, 0);
    } else if (effectiveMode === 'quard') {
      camera.position.set(0, 240, 0.1);
      controls.target.set(0, 35, 0);
    } else if (effectiveMode === 'sensors') {
      camera.position.set(0, 200, 0.1);
      controls.target.set(0, 50, 0);
    } else {
      camera.position.set(0, 450, 0.1);
      controls.target.set(0, 80, 0);
    }
    controls.update();
  });

  document.getElementById('btn-side-cam').addEventListener('click', () => {
    const effectiveMode = getEffectiveModelMode();
    if (effectiveMode === 'dogu') {
      camera.position.set(0, 70, 380);
      controls.target.set(0, 70, 0);
    } else if (effectiveMode === 'dual') {
      camera.position.set(400, 120, 0);
      controls.target.set(0, 50, 0);
    } else if (effectiveMode === 'quard') {
      camera.position.set(220, 35, 0);
      controls.target.set(0, 35, 0);
    } else if (effectiveMode === 'sensors') {
      camera.position.set(180, 50, 0);
      controls.target.set(0, 50, 0);
    } else {
      camera.position.set(450, 80, 0);
      controls.target.set(0, 80, 0);
    }
    controls.update();
  });
}

// ── Dogu (12-joint quadruped): model preview with simple animations ──────
// Kinematics, meshes and keyframes come from urdf/dogu/ (built by
// scripts/dogu/), served as /models/dogu/*.json and /meshes/dogu_*.stl.
// Dogu has no firmware yet, so these angles only move the 3D model.
const dogu = {
  container: null, pitch: null, root: null, joints: {}, feet: {},
  model: null, anims: null, angles: [], mode: null, speed: 1.0,
  loaded: false, loading: false, callbacks: [], lastTick: performance.now()
};

function doguLog(text, kind = 'info') {
  if (consoles.dogu) consoles.dogu.log(text, kind);
}

function loadDoguModel(onComplete) {
  if (dogu.loaded) { if (onComplete) onComplete(); return; }
  if (onComplete) dogu.callbacks.push(onComplete);
  if (dogu.loading) return;
  dogu.loading = true;
  refreshModelBadge();
  Promise.all([fetch('/models/dogu/model.json').then(r => r.json()),
               fetch('/models/dogu/animations.json').then(r => r.json())])
    .then(([model, anims]) => buildDogu(model, anims))
    .catch(err => {
      dogu.loading = false;
      doguLog(`Can't load the Dogu model: ${err.message || err} (run scripts/dogu/build_dogu.py)`, 'error');
      updateModelBadge('Dogu model missing', 'badge-loading');
    });
}

function buildDogu(model, anims) {
  dogu.model = model;
  dogu.anims = anims;
  dogu.angles = anims.poses.stand.slice();
  dogu.container = new THREE.Group();      // ground contact (height)
  dogu.pitch = new THREE.Group();          // ground contact (pitch, for sit/crouch)
  dogu.root = new THREE.Group();
  dogu.root.rotation.x = -Math.PI / 2;     // ROS z-up -> three.js y-up
  dogu.root.scale.setScalar(1000);         // metres -> scene millimetres
  dogu.container.add(dogu.pitch);
  dogu.pitch.add(dogu.root);
  dogu.container.visible = false;
  scene.add(dogu.container);

  const linkGroup = { base_link: dogu.root };
  // joints are listed leg by leg, parents first (hip, femur, tibia)
  model.joints.forEach(j => {
    const anchor = new THREE.Group();
    anchor.position.set(j.xyz[0], j.xyz[1], j.xyz[2]);
    anchor.rotation.order = 'ZYX';         // URDF rpy
    anchor.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2]);
    const turn = new THREE.Group();        // turns about the joint's z axis
    anchor.add(turn);
    linkGroup[j.parent].add(anchor);
    linkGroup[j.child] = turn;
    dogu.joints[j.name] = turn;
  });

  const colors = { base: 0x3d8bd8, hip: 0x4d525c, femur: 0xed9e33, tibia: 0xd9dbe0 };
  const loader = new THREE.STLLoader();
  const withMesh = model.links.filter(l => l.mesh);
  let pending = withMesh.length;
  const done = () => {
    if (--pending > 0) return;
    dogu.loaded = true;
    dogu.loading = false;
    applyDoguAngles();
    refreshModelBadge();
    updateModelVisibilityAndCamera(false);
    doguLog(`Dogu model loaded: ${model.joints.length} joints, ${Object.keys(anims.poses).length} poses, ` +
            `${Object.keys(anims.clips).length} moves`, 'ok');
    const cbs = dogu.callbacks.splice(0);
    cbs.forEach(cb => cb());
  };
  withMesh.forEach(l => {
    const kind = Object.keys(colors).find(k => l.name.includes(k)) || 'base';
    loader.load('/meshes/' + l.mesh, (geom) => {
      geom.computeVertexNormals();
      const mesh = new THREE.Mesh(geom, new THREE.MeshStandardMaterial({ color: colors[kind], roughness: 0.45, metalness: 0.15 }));
      mesh.castShadow = true;
      linkGroup[l.name].add(mesh);
      done();
    }, undefined, () => { doguLog(`Mesh ${l.mesh} failed to load`, 'warn'); done(); });
    if (l.foot) {
      const foot = new THREE.Object3D();
      foot.position.set(l.foot[0], l.foot[1], l.foot[2]);
      linkGroup[l.name].add(foot);
      dogu.feet[l.name.slice(0, 2)] = foot;
    }
  });
}

function applyDoguAngles() {
  if (!dogu.root) return;
  dogu.anims.joints.forEach((name, i) => {
    const turn = dogu.joints[name];
    if (turn) turn.rotation.z = dogu.angles[i];
  });
  // Keep the feet on the floor: pitch so the lowest front and back feet are
  // level, then drop the body until the lowest foot touches y = 0.
  dogu.pitch.rotation.z = 0;
  dogu.container.position.y = 0;
  dogu.container.updateMatrixWorld(true);
  const p = {};
  const v = new THREE.Vector3();
  ['fl', 'fr', 'bl', 'br'].forEach(k => { if (dogu.feet[k]) p[k] = dogu.feet[k].getWorldPosition(v.clone()); });
  if (!p.fl || !p.bl) return;
  const front = p.fl.y < p.fr.y ? p.fl : p.fr;
  const back = p.bl.y < p.br.y ? p.bl : p.br;
  dogu.pitch.rotation.z = Math.atan2(back.y - front.y, front.x - back.x);
  dogu.container.updateMatrixWorld(true);
  let minY = Infinity;
  Object.values(dogu.feet).forEach(f => { minY = Math.min(minY, f.getWorldPosition(v).y); });
  dogu.container.position.y = -minY;
}

const doguEase = (t) => { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); };

function doguClipAngles(clip, phase) {
  const frames = clip.frames;
  const x = ((phase % 1) + 1) % 1 * frames.length;
  const i = Math.floor(x) % frames.length;
  const j = (i + 1) % frames.length;
  const f = x - Math.floor(x);
  return frames[i].map((a, k) => a + (frames[j][k] - a) * f);
}

function tickDogu() {
  const now = performance.now();
  const dt = Math.min(0.1, (now - dogu.lastTick) / 1000);
  dogu.lastTick = now;
  if (!dogu.loaded || !dogu.mode) return;
  const m = dogu.mode;
  if (m.kind === 'pose') {
    const k = doguEase((now - m.t0) / 600);
    dogu.angles = m.from.map((a, i) => a + (m.to[i] - a) * k);
    if (k >= 1) { dogu.mode = null; setDoguStatus(null, 'Holding pose'); }
  } else {
    const clip = dogu.anims.clips[m.name];
    m.phase += dt * dogu.speed / clip.duration;
    const target = doguClipAngles(clip, m.phase);
    const k = doguEase((now - m.t0) / 400);          // blend in from the previous pose
    dogu.angles = m.from.map((a, i) => a + (target[i] - a) * k);
  }
  applyDoguAngles();
  updateDoguReadouts();
}

function setDoguStatus(kind, text) {
  setPill(document.getElementById('pill-dogu-status'), kind, text);
}

function doguPlay(kind, name) {
  if (estopEngaged) { doguLog(`Blocked by E-STOP: ${name}`, 'warn'); return; }
  if (!dogu.loaded) { loadDoguModel(() => doguPlay(kind, name)); return; }
  if (kind === 'pose') {
    dogu.mode = { kind, from: dogu.angles.slice(), to: dogu.anims.poses[name].slice(), t0: performance.now() };
    setDoguStatus('info', name.charAt(0).toUpperCase() + name.slice(1));
    doguLog(`Pose: ${name}`);
  } else {
    dogu.mode = { kind, name, phase: 0, from: dogu.angles.slice(), t0: performance.now() };
    setDoguStatus('online', `Playing ${name.replace('_', ' ')}`);
    doguLog(`Playing ${name} at ${dogu.speed.toFixed(2)}×`);
  }
  document.querySelectorAll('.btn-dogu').forEach(b => b.classList.toggle('active', b.dataset.name === name));
}

function doguStop() {
  if (dogu.mode) doguLog('Stopped');
  dogu.mode = null;
  setDoguStatus(null, 'Stopped');
  document.querySelectorAll('.btn-dogu').forEach(b => b.classList.remove('active'));
}

function updateDoguReadouts() {
  if (!dogu.anims) return;
  dogu.anims.joints.forEach((name, i) => {
    const el = document.getElementById(`dogu-angle-${name}`);
    const deg = Math.round(dogu.angles[i] * 1800 / Math.PI) / 10 || 0;   // || 0 drops -0
    if (el) el.textContent = `${deg.toFixed(1)}°`;
  });
}

function initDoguControls() {
  consoles.dogu = new ConsolePanel(document.getElementById('console-dogu'), { name: 'dogu' });
  document.querySelectorAll('.btn-dogu').forEach(b => {
    b.addEventListener('click', () => doguPlay(b.dataset.kind, b.dataset.name));
  });
  document.getElementById('btn-dogu-stop').addEventListener('click', doguStop);
  const speed = document.getElementById('slider-dogu-speed');
  speed.addEventListener('input', () => {
    dogu.speed = Number(speed.value) / 100;
    document.getElementById('lbl-dogu-speed').textContent = `${dogu.speed.toFixed(2)}×`;
  });
  document.getElementById('btn-load-dogu-view').addEventListener('click', () => {
    document.getElementById('tab-btn-dogu').click();
    setModelMode('dogu');
  });
}

// ── Console panels (one per robot) ───────────────────────────────────────
// Lines come from the server (/api/console) and from this page. Device text is
// only ever set with textContent, never parsed as HTML.
const CONSOLE_KINDS = {
  tx: '→', rx: '←', data: ' ', info: '•', ok: '✓', warn: '!', error: '✗'
};
const CONSOLE_FILTERS = [
  ['All', null], ['Sent', ['tx']], ['Received', ['rx', 'data']],
  ['Events', ['info', 'ok', 'warn', 'error']], ['Problems', ['warn', 'error']]
];
const consoles = {};

class ConsolePanel {
  constructor(root, opts = {}) {
    this.root = root;
    this.name = opts.name || root.dataset.channel || 'console';
    this.max = opts.max || 1000;
    this.entries = [];          // {id, t, kind, text, count, el}
    this.byId = new Map();
    this.paused = false;
    this.pending = 0;
    this.kindFilter = null;
    this.textFilter = '';
    this.localSeq = 0;
    this.history = [];
    this.historyKey = `console-history-${this.name}`;
    try { this.history = JSON.parse(storageGet(this.historyKey) || '[]'); } catch (e) { this.history = []; }
    this.histIdx = this.history.length;
    this.build(opts);
  }

  build(opts) {
    const r = this.root;
    r.classList.add('console');
    const bar = document.createElement('div');
    bar.className = 'console-bar';
    this.filterSel = document.createElement('select');
    this.filterSel.className = 'form-select-sm';
    this.filterSel.setAttribute('aria-label', 'Show');
    CONSOLE_FILTERS.forEach(([label], i) => this.filterSel.add(new Option(label, String(i))));
    this.filterSel.addEventListener('change', () => this.applyFilter());
    this.find = document.createElement('input');
    this.find.type = 'search';
    this.find.className = 'form-input console-find';
    this.find.placeholder = 'Find…';
    this.find.setAttribute('aria-label', 'Find in console');
    let t = null;
    this.find.addEventListener('input', () => { clearTimeout(t); t = setTimeout(() => this.applyFilter(), 150); });
    this.extra = document.createElement('span');
    this.extra.className = 'console-extra';
    this.count = document.createElement('span');
    this.count.className = 'console-count';
    const mk = (label, title, fn, icon = false) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'btn btn-outline btn-xs' + (icon ? ' btn-icon' : '');
      b.textContent = label;
      b.title = title;
      if (icon) b.setAttribute('aria-label', title);
      b.addEventListener('click', fn);
      return b;
    };
    this.mk = mk;
    this.pauseBtn = mk('⏸', 'Pause: freeze the view to read it (new lines are kept)', () => this.setPaused(!this.paused), true);
    bar.append(this.filterSel, this.find, this.extra, this.count, this.pauseBtn,
      mk('⧉', 'Copy the lines shown', () => this.copy(), true),
      mk('⤓', 'Download every line as a .log file', () => this.save(), true),
      mk('⌫', 'Clear this view', () => this.clear(), true));
    this.view = document.createElement('div');
    this.view.className = 'console-view';
    this.view.setAttribute('role', 'log');
    this.view.setAttribute('aria-live', 'off');
    this.view.tabIndex = 0;
    r.append(bar, this.view);

    if (opts.placeholder) {
      const form = document.createElement('form');
      form.className = 'console-input-row';
      this.input = document.createElement('input');
      this.input.className = 'form-input';
      this.input.placeholder = opts.placeholder;
      this.input.spellcheck = false;
      this.input.setAttribute('aria-label', `${this.name} command`);
      this.input.addEventListener('keydown', (e) => this.onKey(e));
      const send = document.createElement('button');
      send.type = 'submit';
      send.className = 'btn btn-primary btn-xs';
      send.textContent = 'Send';
      form.append(this.input, send);
      (opts.quick || []).forEach(([label, command, tip]) => {
        form.append(mk(label, `${tip} [${command}]`, () => opts.onSubmit && opts.onSubmit(command)));
      });
      form.addEventListener('submit', (e) => {
        e.preventDefault();
        const text = this.input.value.trim();
        if (!text) return;
        this.remember(text);
        this.input.value = '';
        if (opts.onSubmit) opts.onSubmit(text);
      });
      r.append(form);
    }
    this.updateCount();
  }

  onKey(e) {
    if (e.key === 'ArrowUp' && this.history.length) {
      e.preventDefault();
      this.histIdx = Math.max(0, this.histIdx - 1);
      this.input.value = this.history[this.histIdx];
    } else if (e.key === 'ArrowDown' && this.history.length) {
      e.preventDefault();
      this.histIdx = Math.min(this.history.length, this.histIdx + 1);
      this.input.value = this.history[this.histIdx] || '';
    }
    e.stopPropagation();   // typing here is not a Quard walk key
  }

  remember(text) {
    this.history = this.history.filter(h => h !== text).concat([text]).slice(-50);
    this.histIdx = this.history.length;
    storageSet(this.historyKey, JSON.stringify(this.history));
  }

  addExtra(el) { this.extra.append(el); }

  // Local line (from this page)
  log(text, kind = 'info') {
    this.add({ id: `L${++this.localSeq}`, t: Date.now() / 1000, kind, text: String(text), count: 1 });
  }

  // Server entry; an id we already have is an in-place update
  add(e) {
    const existing = this.byId.get(e.id);
    if (existing) {
      Object.assign(existing, { t: e.t, kind: e.kind, text: e.text, count: e.count });
      if (existing.el) this.fillLine(existing.el, existing);
      if (existing.el && !this.matches(existing)) { existing.el.remove(); existing.el = null; }
      return;
    }
    const entry = { id: e.id, t: e.t, kind: CONSOLE_KINDS[e.kind] !== undefined ? e.kind : 'info', text: e.text, count: e.count || 1, el: null };
    // Server lines arrive by polling, so a line typed here may already be
    // newer: keep the view in time order.
    let idx = this.entries.length;
    while (idx > 0 && this.entries[idx - 1].t > entry.t) idx--;
    this.entries.splice(idx, 0, entry);
    this.byId.set(entry.id, entry);
    if (this.entries.length > this.max) {
      const old = this.entries.shift();
      this.byId.delete(old.id);
      if (old.el) old.el.remove();
      idx--;
    }
    if (this.paused) {
      this.pending++;
    } else if (this.matches(entry)) {
      const atBottom = this.view.scrollHeight - this.view.scrollTop - this.view.clientHeight < 12;
      entry.el = this.makeLine(entry);
      const next = this.entries.slice(idx + 1).find(x => x.el);
      if (next) this.view.insertBefore(entry.el, next.el); else this.view.append(entry.el);
      if (atBottom) this.view.scrollTop = this.view.scrollHeight;
    }
    this.updateCount();
  }

  matches(e) {
    if (this.kindFilter && !this.kindFilter.includes(e.kind)) return false;
    if (this.textFilter && !e.text.toLowerCase().includes(this.textFilter)) return false;
    return true;
  }

  stamp(t) {
    const d = new Date(t * 1000);
    const p = (n, w = 2) => String(n).padStart(w, '0');
    return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
  }

  makeLine(e) {
    const el = document.createElement('div');
    this.fillLine(el, e);
    return el;
  }

  fillLine(el, e) {
    el.className = `cl cl-${e.kind}`;
    el.replaceChildren();
    const time = document.createElement('time');
    time.textContent = this.stamp(e.t);
    const sym = document.createElement('span');
    sym.className = 'cl-sym';
    sym.textContent = CONSOLE_KINDS[e.kind] || '•';
    const text = document.createElement('span');
    text.className = 'cl-text';
    text.textContent = e.text;
    el.append(time, sym, text);
    if (e.count > 1) {
      const c = document.createElement('span');
      c.className = 'cl-count';
      c.textContent = `×${e.count}`;
      el.append(c);
    }
  }

  plain(e) {
    return `${this.stamp(e.t)} ${e.kind.toUpperCase().padEnd(5)} ${e.text}${e.count > 1 ? `  (x${e.count})` : ''}`;
  }

  renderAll() {
    const frag = document.createDocumentFragment();
    this.entries.forEach(e => {
      if (this.matches(e)) { e.el = this.makeLine(e); frag.append(e.el); } else { e.el = null; }
    });
    this.view.replaceChildren(frag);
    this.view.scrollTop = this.view.scrollHeight;
    this.pending = 0;
    this.updateCount();
  }

  applyFilter() {
    this.kindFilter = CONSOLE_FILTERS[Number(this.filterSel.value)][1];
    this.textFilter = this.find.value.trim().toLowerCase();
    this.renderAll();
  }

  setPaused(on) {
    this.paused = on;
    this.pauseBtn.textContent = on ? '▶' : '⏸';
    this.pauseBtn.title = on ? 'Resume: show the lines that arrived while paused' : 'Pause: freeze the view to read it (new lines are kept)';
    this.pauseBtn.setAttribute('aria-label', this.pauseBtn.title);
    this.pauseBtn.classList.toggle('btn-warning', on);
    this.pauseBtn.classList.toggle('btn-outline', !on);
    if (!on && this.pending) this.renderAll();
    this.updateCount();
  }

  clear() {
    this.entries.forEach(e => this.byId.delete(e.id));
    this.entries = [];
    this.view.replaceChildren();
    this.pending = 0;
    this.updateCount();
  }

  copy() {
    const text = this.entries.filter(e => this.matches(e)).map(e => this.plain(e)).join('\n');
    const done = () => toast(`Copied ${text ? text.split('\n').length : 0} lines`, 'success', 1500);
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(done, () => toast('Copy failed: clipboard not available', 'error'));
    } else {
      toast('Copy failed: clipboard not available', 'error');
    }
  }

  save() {
    const text = this.entries.map(e => this.plain(e)).join('\n') + '\n';
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    const a = document.createElement('a');
    a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' }));
    a.download = `${this.name}-console-${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.log`;
    document.body.append(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  updateCount() {
    let txt = `${this.entries.length}`;
    if (this.paused) txt += ` · paused, ${this.pending} new`;
    this.count.title = `${this.entries.length} lines kept (newest ${this.max})`;
    this.count.textContent = txt;
  }
}

let consoleSeq = 0;

function startConsolePolling() {
  const poll = () => {
    fetch(`/api/console?since=${consoleSeq}`)
      .then(res => res.json())
      .then(data => {
        if (data.reset) {
          // Server restarted: its ids start again from 1
          Object.values(consoles).forEach(c => c.clear());
          consoleSeq = 0;
          return;
        }
        (data.entries || []).forEach(e => { if (consoles[e.channel]) consoles[e.channel].add(e); });
        consoleSeq = data.seq || consoleSeq;
      })
      .catch(() => {})
      .finally(() => setTimeout(poll, 500));
  };
  poll();
}

// ── Link cards (arm, Quard Bot, rover) ────────────────────────────────────
let lastLinks = null;
let lastLinksRender = 0;
let armLinkPrevState = null;

function ageText(sec) {
  if (sec === null || sec === undefined) return 'never';
  return sec < 60 ? `${sec.toFixed(1)} s ago` : `${Math.round(sec / 60)} min ago`;
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el && el.textContent !== text) { el.textContent = text; el.title = text; }
}

function renderLinks(links, force = false) {
  if (!links) return;
  lastLinks = links;
  const now = Date.now();
  if (!force && now - lastLinksRender < 250) return;   // /api/state arrives at 25 Hz
  lastLinksRender = now;
  renderArmLink(links.arm || {});
  renderQuardLink(links.quard || {});
  renderSensorLink(links.sensors || {});
}

function renderArmLink(a) {
  const pill = document.getElementById('pill-arm-link');
  let kind, label, detail, badge;
  let mismatch = null;
  const serialLink = a.transport !== 'wifi';
  const parts = [a.target || ''];
  switch (a.state) {
    case 'no_ros':
      kind = 'offline'; label = 'ROS 2 not available'; badge = ['warn', 'UI only (no ROS)'];
      detail = 'Nothing reaches the arm. Start with ./launch_web_dashboard.sh so ROS 2 is sourced.';
      break;
    case 'no_commander':
      kind = 'offline'; label = 'Commander not running'; badge = ['offline', 'No commander'];
      detail = `No link state from servo_serial_commander${a.last_heard ? ` (last heard ${Math.round(a.last_heard)} s ago)` : ''}. ` +
        'Start it: ros2 run servo_joint_publisher servo_serial_commander';
      break;
    case 'legacy':
      kind = a.connected ? 'online' : 'warn'; label = a.connected ? 'Online' : 'Disconnected';
      badge = [kind, label];
      detail = `${a.detail || ''} (basic status from an older commander)`;
      break;
    case 'online':
      kind = 'online'; label = 'Online'; badge = ['online', `Online · ${serialLink ? 'Serial' : 'Wi-Fi'}`];
      if (a.rtt_ms !== null && a.rtt_ms !== undefined) parts.push(`ping ${Math.round(a.rtt_ms)} ms`);
      parts.push(`last reply ${ageText(a.last_rx_age)}`);
      break;
    case 'mismatch': {
      kind = 'warn'; label = 'Mode mismatch'; badge = ['warn', 'Mode mismatch'];
      const listen = a.mismatch_mode === 'WIFI' ? 'Wi-Fi' : 'serial';
      mismatch = { text: `The ESP32 replies, but it is listening on ${listen}, so commands sent over ${serialLink ? 'serial' : 'Wi-Fi'} are ignored.`,
                   mode: serialLink ? 'SERIAL' : 'WIFI' };
      parts.push(`last reply ${ageText(a.last_rx_age)}`);
      break;
    }
    case 'port_error':
      kind = 'offline'; label = serialLink ? 'Port error' : 'Socket error'; badge = ['offline', label];
      parts.push(a.last_error || 'cannot open the link');
      break;
    default:
      kind = 'warn'; label = 'No reply'; badge = ['warn', 'No reply'];
      parts.push(`last reply ${ageText(a.last_rx_age)}`, 'is the ESP32 powered and on this link?');
  }
  const alive = ['online', 'mismatch', 'port_error', 'no_reply'].includes(a.state);
  if (alive) {
    parts.push(`RX ${a.rx || 0} · TX ${a.tx || 0}`);
    if (a.esp32_ip) parts.push(`ESP32 IP ${a.esp32_ip}`);
    detail = parts.filter(Boolean).join(' · ');
    // Fill the settings once per change of the commander's target
    const key = `${a.port}|${a.baudrate}|${a.ip}`;
    if (key !== renderArmLink.settingsKey) {
      renderArmLink.settingsKey = key;
      const portIn = document.getElementById('arm-port');
      const baudSel = document.getElementById('arm-baud');
      const ipIn = document.getElementById('arm-ip');
      if (portIn && a.port && document.activeElement !== portIn) portIn.value = a.port;
      if (baudSel && a.baudrate) {
        if (![...baudSel.options].some(o => o.value === String(a.baudrate))) baudSel.add(new Option(String(a.baudrate), String(a.baudrate)));
        baudSel.value = String(a.baudrate);
      }
      if (ipIn && a.ip && document.activeElement !== ipIn) ipIn.value = a.ip;
    }
    const verbose = document.getElementById('chk-arm-verbose');
    if (verbose && Date.now() - (renderArmLink.verboseSetAt || 0) > 2500) verbose.checked = Boolean(a.verbose);
    if (a.state === 'port_error' && armLinkPrevState !== 'port_error') showArmSettings(true);
    if (a.transport && Date.now() - commModeEditedAt > 2500) syncCommModeUI(a.transport === 'wifi');
  }
  armLinkPrevState = a.state;
  setPill(pill, kind, label);
  setText('arm-link-detail', detail || '');
  setNodeBadge('badge-arm', badge[0], badge[1]);
  const mm = document.getElementById('arm-mismatch');
  if (mm) {
    mm.hidden = !mismatch;
    if (mismatch) {
      setText('arm-mismatch-text', `⚠ ${mismatch.text}`);
      const btn = document.getElementById('btn-arm-fix-mode');
      btn.dataset.mode = mismatch.mode;
      btn.textContent = `Switch ESP32 to ${mismatch.mode === 'WIFI' ? 'Wi-Fi' : 'serial'}`;
    }
  }
  const reconnect = document.getElementById('btn-arm-reconnect');
  if (reconnect) reconnect.disabled = !alive;
}

function showArmSettings(on) {
  const panel = document.getElementById('arm-link-settings');
  const btn = document.getElementById('btn-arm-link-settings');
  if (!panel || !btn) return;
  panel.hidden = !on;
  btn.setAttribute('aria-expanded', String(on));
  btn.classList.toggle('active', on);
}

function renderQuardLink(q) {
  let kind, label;
  const parts = [];
  if (q.online === null || q.online === undefined) {
    kind = 'info'; label = 'Checking…';
    parts.push(q.host || '');
  } else if (q.online) {
    kind = 'online';
    label = 'Online' + (q.rtt_ms !== null && q.rtt_ms !== undefined ? ` · ${Math.round(q.rtt_ms)} ms` : '');
    parts.push(`seen ${ageText(q.last_seen_age)}`);
    if (q.rssi) parts.push(`RSSI ${q.rssi} dBm`);
  } else {
    kind = 'offline'; label = 'Unreachable';
    parts.push(q.last_error || 'no reply');
    if (q.last_seen_age !== null && q.last_seen_age !== undefined) parts.push(`last seen ${ageText(q.last_seen_age)}`);
    if (quardPing.nextAt && !quardPing.inFlight) parts.push(`retry in ${Math.max(0, Math.round((quardPing.nextAt - Date.now()) / 1000))} s`);
  }
  if (q.sent) parts.push(`sent ${q.sent}` + (q.failed ? `, failed ${q.failed}` : ''));
  setPill(document.getElementById('pill-quard-status'), kind, label);
  setText('quard-link-detail', parts.filter(Boolean).join(' · '));
  setNodeBadge('badge-quardbot', kind === 'info' ? 'pending' : kind, label);
  quardPing.online = q.online;
}

function renderSensorLink(sl) {
  let kind, label, detail;
  const src = sl.source;
  const target = sl.target || '';
  if (src === 'off') {
    kind = null; label = 'Not connected'; detail = 'No data source. Connect the rover over Wi-Fi or Serial.';
  } else if (sl.phase === 'live' && sl.last_rx_age !== null && sl.last_rx_age < 5) {
    kind = 'online'; label = 'Live';
    detail = `${target} · ${sl.rate} ${src === 'wifi' ? 'updates' : 'lines'}/s · last ${ageText(sl.last_rx_age)}`;
  } else if (sl.phase === 'live') {
    kind = 'warn'; label = 'No data'; detail = `${target} · nothing for ${Math.round(sl.last_rx_age || 0)} s`;
  } else if (sl.phase === 'waiting') {
    kind = 'info'; label = 'Waiting for data';
    detail = target + (sl.phase_age > 5 ? " · port open but silent: wrong baud rate, or the board isn't sending?" : '');
  } else if (sl.phase === 'error') {
    kind = 'offline'; label = 'Error'; detail = `${target} · ${sl.error} · retrying`;
  } else {
    kind = 'info'; label = 'Connecting…'; detail = target;
  }
  setPill(document.getElementById('pill-sensor-wifi-status'), kind, label);
  setText('sensor-link-detail', detail);
  const badge = { online: ['online', src === 'wifi' ? 'Wi-Fi' : 'Serial'], info: ['warn', label],
                  warn: ['warn', label], offline: ['offline', label] }[kind] || ['offline', 'Off'];
  setNodeBadge('badge-sensornode', badge[0], badge[1]);

  if (src !== renderSensorLink.lastSrc) {
    renderSensorLink.lastSrc = src;
    const wifiBtn = document.getElementById('btn-sensor-wifi-connect');
    const wifiOff = document.getElementById('btn-sensor-wifi-disconnect');
    const serBtn = document.getElementById('btn-sensor-connect');
    if (wifiBtn) wifiBtn.hidden = src === 'wifi';
    if (wifiOff) { wifiOff.hidden = src !== 'wifi'; wifiOff.style.display = ''; }
    if (serBtn) {
      serBtn.textContent = src === 'serial' ? '⏹ Disconnect Serial' : 'Connect Serial';
      serBtn.className = src === 'serial' ? 'btn btn-danger btn-sm' : 'btn btn-outline btn-sm';
    }
  }
}

// Refresh age read-outs between /api/state renders even when nothing changes
setInterval(() => { if (lastLinks) renderLinks(lastLinks, true); }, 1000);

function armConsoleSubmit(text) {
  const t = text.trim();
  if (!t) return;
  const upper = t.toUpperCase();
  if ((upper.startsWith('CMD:') || upper.startsWith('RAW:CMD:')) && estopEngaged) {
    consoles.arm.log(`Blocked by E-STOP: ${t}`, 'warn');
    return;
  }
  postJSON('/api/arm/console', { line: t })
    .then(r => { if (r.status !== 'ok' && r.message) consoles.arm.log(`Not sent: ${r.message}`, 'error'); })
    .catch(() => consoles.arm.log('Not sent: server unreachable', 'error'));
  if ((upper.startsWith('CMD:') || upper.startsWith('RAW:CMD:')) && !armConsoleSubmit.hinted) {
    armConsoleSubmit.hinted = true;
    consoles.arm.log('Joint commands typed here are clamped to the joint limits; the next slider move overrides them.', 'info');
  }
}

function sendArmLink(cmd) {
  return postJSON('/api/arm/link', { cmd })
    .then(r => { if (r.status !== 'ok') consoles.arm.log(`LINK:${cmd} not sent: ${r.message || 'error'}`, 'error'); })
    .catch(() => consoles.arm.log(`LINK:${cmd} not sent: server unreachable`, 'error'));
}

function refreshSerialPorts(report = false) {
  return fetch('/api/serial_ports').then(r => r.json()).then(data => {
    const list = document.getElementById('serial-ports-list');
    if (list) {
      list.replaceChildren(...(data.ports || []).map(p => {
        const o = document.createElement('option');
        o.value = p.device;
        o.label = [p.description, p.arm ? 'arm' : ''].filter(Boolean).join(' · ');
        return o;
      }));
    }
    const sensorPort = document.getElementById('sensor-serial-port');
    if (sensorPort && !sensorPort.value) {
      const free = (data.ports || []).find(p => !p.arm);
      sensorPort.value = free ? free.device : '/dev/ttyUSB1';
    }
    if (report) {
      const names = (data.ports || []).map(p => p.device + (p.arm ? ' (arm)' : '')).join(', ') || 'none';
      toast(`Serial ports: ${names}`, 'info');
    }
    return data;
  }).catch(() => {});
}

function initLinkControls() {
  consoles.arm = new ConsolePanel(document.getElementById('console-arm'), {
    name: 'arm',
    placeholder: 'Command for the arm ESP32: PING, MODE:SERIAL, CMD:turntable_link_joint=90 …  (↑/↓ history)',
    quick: [['PING', 'PING', 'Ask the ESP32 to reply'], ['Status', 'LINK:STATUS', 'Ask the commander for its link state']],
    onSubmit: armConsoleSubmit
  });
  const verboseLabel = document.createElement('label');
  verboseLabel.className = 'console-check';
  verboseLabel.title = 'Also show every joint command and ACK (busy while sliders move)';
  verboseLabel.innerHTML = '<input type="checkbox" id="chk-arm-verbose"> Joint traffic';
  consoles.arm.addExtra(verboseLabel);
  document.getElementById('chk-arm-verbose').addEventListener('change', (e) => {
    renderArmLink.verboseSetAt = Date.now();
    sendArmLink(`VERBOSE=${e.target.checked ? 1 : 0}`);
  });

  consoles.quard = new ConsolePanel(document.getElementById('console-quard'), {
    name: 'quard',
    placeholder: 'Quard command: move=w · ch=3&deg=70 · all=45 · speed=180 · status  (↑/↓ history)',
    quick: [['Status', 'status', 'GET /status and show the reply'], ['Stand', 'move=s', 'Stand still']],
    onSubmit: quardConsoleSubmit
  });
  consoles.sensors = new ConsolePanel(document.getElementById('console-sensors'), { name: 'rover' });

  document.getElementById('btn-arm-reconnect').addEventListener('click', () => sendArmLink('RECONNECT'));
  document.getElementById('btn-arm-link-settings').addEventListener('click', () => {
    showArmSettings(document.getElementById('arm-link-settings').hidden);
  });
  document.getElementById('btn-arm-console-toggle').addEventListener('click', (e) => {
    const box = document.getElementById('console-arm');
    box.hidden = !box.hidden;
    e.currentTarget.setAttribute('aria-expanded', String(!box.hidden));
    e.currentTarget.classList.toggle('active', !box.hidden);
    storageSet('arm-console-open', box.hidden ? '0' : '1');
  });
  if (storageGet('arm-console-open') === '0') document.getElementById('btn-arm-console-toggle').click();
  document.getElementById('btn-arm-fix-mode').addEventListener('click', (e) => {
    const mode = e.currentTarget.dataset.mode || 'SERIAL';
    armConsoleSubmit(`MODE:${mode}`);
  });
  document.getElementById('btn-arm-use-port').addEventListener('click', () => {
    const port = document.getElementById('arm-port').value.trim();
    const baud = document.getElementById('arm-baud').value;
    const a = (lastLinks && lastLinks.arm) || {};
    let changed = false;
    if (baud && String(a.baudrate) !== baud) { sendArmLink(`BAUD=${baud}`); changed = true; }
    if (port && a.port !== port) { sendArmLink(`PORT=${port}`); changed = true; }
    if (!changed) sendArmLink('RECONNECT');
    if (a.transport === 'wifi') consoles.arm.log('Port saved. The commander is on Wi-Fi: switch the header toggle to Serial to use it.', 'info');
  });
  const useIp = () => {
    const ip = document.getElementById('arm-ip').value.trim();
    if (!ip) return;
    sendArmLink(`IP=${ip}`);
    const a = (lastLinks && lastLinks.arm) || {};
    if (a.transport === 'serial') consoles.arm.log('Address saved. The commander is on serial: switch the header toggle to Wi-Fi to use it.', 'info');
  };
  document.getElementById('btn-arm-use-ip').addEventListener('click', useIp);
  document.getElementById('arm-ip').addEventListener('keydown', (e) => { if (e.key === 'Enter') useIp(); });
  document.querySelectorAll('.btn-rescan-ports').forEach(b => b.addEventListener('click', () => refreshSerialPorts(true)));
  refreshSerialPorts();
}

// ── Real-time State Polling ──────────────────────────────────────────────
let serverOnline = null;
let stateFailures = 0;
let commModeEditedAt = 0;

function setServerOnline(online) {
  if (online === serverOnline) return;
  serverOnline = online;
  document.body.classList.toggle('server-offline', !online);
  if (!online) {
    ['badge-arm', 'badge-quardbot', 'badge-sensornode'].forEach(id => setNodeBadge(id, 'offline', 'Server offline'));
    setPill(document.getElementById('pill-arm-link'), 'offline', 'Server offline');
    lastLinks = null;
  }
}

function startStatePolling() {
  // Chained timeouts (not setInterval) so a slow server never accumulates
  // overlapping /api/state requests.
  const poll = () => {
    fetch('/api/state')
      .then(res => res.json())
      .then(data => {
        stateFailures = 0;
        setServerOnline(true);
        renderLinks(data.links);
        if (data.comm_mode && !(data.links && data.links.arm && data.links.arm.transport) &&
            Date.now() - commModeEditedAt > 1500) {
          syncCommModeUI(String(data.comm_mode).toUpperCase() === 'WIFI');
        }

        if (typeof data.simulating === 'boolean' && data.simulating !== isSimulating) {
          // Another client (or the server) changed it: follow without re-posting.
          isSimulating = data.simulating;
          const btnSim = document.getElementById('btn-sim');
          btnSim.innerText = isSimulating ? '⏸️ Stop Sim' : '▶️ Start Sim';
          btnSim.className = isSimulating ? 'btn btn-danger btn-sm' : 'btn btn-warning btn-sm';
        }

        if (data.joints) {
          const now = Date.now();
          for (let k in data.joints) {
            // Don't let a poll that raced a local slider move snap it back.
            if (!isSimulating && (armSmoothAnimId || now - (lastLocalJointEdit[k] || 0) < 500)) continue;
            jointState[k] = data.joints[k];
            // Keep sliders in step with the server (sim, or another client)
            setSliderPosition(k, data.joints[k]);
          }
        }
      })
      .catch(() => {
        // One dropped request is noise; a few in a row means the server is gone.
        if (++stateFailures >= 3) setServerOnline(false);
      })
      .finally(() => setTimeout(poll, serverOnline === false ? 1000 : 40)); // ~25 Hz state sync
  };
  poll();
}

// ── Tab Management System ────────────────────────────────────────────────
let activeTab = 'arm';

function setupTabs() {
  const btnArm = document.getElementById('tab-btn-arm');
  const btnQuard = document.getElementById('tab-btn-quard');
  const btnSensors = document.getElementById('tab-btn-sensors');
  const btnDogu = document.getElementById('tab-btn-dogu');

  const panelArm = document.getElementById('panel-arm-control');
  const panelQuard = document.getElementById('panel-quard-control');
  const panelSensors = document.getElementById('panel-sensors-control');
  const panelDogu = document.getElementById('panel-dogu-control');

  function setActive(tabName, btn, panel) {
    activeTab = tabName;
    storageSet('dashboard-tab', tabName);
    [btnArm, btnQuard, btnSensors, btnDogu].forEach(b => {
      if (!b) return;
      b.classList.remove('active');
      b.setAttribute('aria-selected', 'false');
      b.tabIndex = -1;
    });
    [panelArm, panelQuard, panelSensors, panelDogu].forEach(p => p && p.classList.remove('active'));

    if (btn) {
      btn.classList.add('active');
      btn.setAttribute('aria-selected', 'true');
      btn.tabIndex = 0;
    }
    if (panel) panel.classList.add('active');
  }

  // Arrow keys move between tabs (WAI-ARIA tabs pattern)
  const tabs = [btnArm, btnQuard, btnSensors, btnDogu].filter(Boolean);
  tabs.forEach((b, i) => {
    b.tabIndex = b.classList.contains('active') ? 0 : -1;
    b.addEventListener('keydown', (e) => {
      if (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft') return;
      e.preventDefault();
      e.stopPropagation(); // not a Quard walk command
      const next = tabs[(i + (e.key === 'ArrowRight' ? 1 : tabs.length - 1)) % tabs.length];
      next.focus();
      next.click();
    });
  });

  if (btnArm) {
    btnArm.addEventListener('click', () => {
      setActive('arm', btnArm, panelArm);
      if (modelLoadSystem.mode === 'auto') {
        setModelMode('auto');
      }
    });
  }
  if (btnQuard) {
    btnQuard.addEventListener('click', () => {
      setActive('quard', btnQuard, panelQuard);
      if (modelLoadSystem.mode === 'auto') {
        setModelMode('auto');
      }
      pingQuardBot();
    });
  }
  if (btnSensors) {
    btnSensors.addEventListener('click', () => {
      setActive('sensors', btnSensors, panelSensors);
      if (modelLoadSystem.mode === 'auto') {
        setModelMode('auto');
      }
    });
  }
  if (btnDogu) {
    btnDogu.addEventListener('click', () => {
      setActive('dogu', btnDogu, panelDogu);
      if (modelLoadSystem.mode === 'auto') {
        setModelMode('auto');
      }
    });
  }
}

function restoreActiveTab() {
  const saved = storageGet('dashboard-tab');
  const btn = saved && saved !== 'arm' ? document.getElementById(`tab-btn-${saved}`) : null;
  if (btn) btn.click();
}

// ── Quard Bot Locomotion & PCA9685 Control System ────────────────────────
const QUARD_CHANNELS = [
  { ch: 0, name: 'R1 (Right Front Hip)', baseZero: 90 },
  { ch: 1, name: 'R2 (Right Rear Hip)', baseZero: 0 },
  { ch: 2, name: 'L1 (Left Front Hip)', baseZero: 0 },
  { ch: 3, name: 'L2 (Left Rear Hip)', baseZero: 90 },
  { ch: 4, name: 'R4 (Right Rear Foot)', baseZero: 90 },
  { ch: 5, name: 'R3 (Right Front Foot)', baseZero: 0 },
  { ch: 6, name: 'L3 (Left Front Foot)', baseZero: 90 },
  { ch: 7, name: 'L4 (Left Rear Foot)', baseZero: 0 }
];

function initQuardBotControls() {
  buildQuardChannelSliders();
  setupQuardEventListeners();
  setupQuardKeyboard();
  initQuardHost();
  setTimeout(() => pingQuardBot(), 600);
}

function buildQuardChannelSliders() {
  const container = document.getElementById('quard-channels-list');
  if (!container) return;
  container.innerHTML = '';

  QUARD_CHANNELS.forEach(c => {
    const jInfo = CHANNEL_TO_JOINT[c.ch];
    const badgeColor = jInfo ? jInfo.color : '#0284c7';
    const card = document.createElement('div');
    card.className = 'quard-channel-card';
    card.innerHTML = `
      <div class="quard-channel-header">
        <label class="quard-channel-name" for="slider-quard-ch-${c.ch}">
          <span class="channel-dot" style="background-color:${badgeColor}"></span>
          <strong>Ch ${c.ch}</strong> ${c.name}
        </label>
        <span class="quard-channel-val" id="readout-quard-ch-${c.ch}">${c.baseZero}°</span>
      </div>
      <div class="slider-row">
        <input type="range" id="slider-quard-ch-${c.ch}" min="0" max="180" value="${c.baseZero}">
      </div>
    `;
    container.appendChild(card);

    const slider = card.querySelector(`#slider-quard-ch-${c.ch}`);
    slider.addEventListener('input', (e) => {
      const angle = parseInt(e.target.value);
      document.getElementById(`readout-quard-ch-${c.ch}`).innerText = `${angle}°`;
      onQuardSliderChanged(c.ch, angle);
    });
  });
}

function onQuardSliderChanged(ch, angle) {
  if (estopEngaged) {
    const slider = document.getElementById(`slider-quard-ch-${ch}`);
    if (slider) slider.value = quardState.channels[ch];
    document.getElementById(`readout-quard-ch-${ch}`).innerText = `${quardState.channels[ch]}°`;
    return;
  }
  quardState.channels[ch] = angle;
  const jInfo = CHANNEL_TO_JOINT[ch];
  if (jInfo) {
    quardState.joints[jInfo.id] = channelAngleToRad(ch, angle);
  }
  updateQuardKinematics();
  syncQuardJointsToServer();
  // The firmware reads the angle from 'deg' (or 'val'); without one it moves to 0°.
  sendQuardCmd({ ch: ch, deg: angle });
}

let quardSmoothAnimId = null;

function smoothSetQuardPose(targetAngles, sendCmd = true, durationMs = 300, onComplete = null) {
  if (quardSmoothAnimId) cancelAnimationFrame(quardSmoothAnimId);

  const startAngles = (quardState.channels && quardState.channels.length === 8) 
    ? quardState.channels.slice() 
    : [45, 45, 45, 45, 45, 45, 45, 45];
  const startTime = performance.now();

  function animate(now) {
    const elapsed = now - startTime;
    let progress = Math.min(1.0, elapsed / durationMs);
    const tSmooth = 0.5 - 0.5 * Math.cos(progress * Math.PI);

    startAngles.forEach((startA, ch) => {
      const targetA = targetAngles[ch] !== undefined ? targetAngles[ch] : startA;
      const interpA = Math.round(startA + (targetA - startA) * tSmooth);
      quardState.channels[ch] = interpA;

      const jInfo = CHANNEL_TO_JOINT[ch];
      if (jInfo) {
        quardState.joints[jInfo.id] = channelAngleToRad(ch, interpA);
      }
      const slider = document.getElementById(`slider-quard-ch-${ch}`);
      const readout = document.getElementById(`readout-quard-ch-${ch}`);
      if (slider) slider.value = interpA;
      if (readout) readout.innerText = `${interpA}°`;
    });

    const sliderAll = document.getElementById('slider-quard-all');
    const readoutAll = document.getElementById('readout-quard-all');
    if (sliderAll) sliderAll.value = targetAngles[0];
    if (readoutAll) readoutAll.innerText = `${targetAngles[0]}°`;

    updateQuardKinematics();
    syncQuardJointsToServer();

    if (progress < 1.0) {
      quardSmoothAnimId = requestAnimationFrame(animate);
    } else {
      quardSmoothAnimId = null;
      if (sendCmd) {
        sendQuardCmd({ all: targetAngles[0] });
      }
      if (onComplete) onComplete();
    }
  }

  quardSmoothAnimId = requestAnimationFrame(animate);
}

function setQuardPose(angles, sendCmd = true) {
  smoothSetQuardPose(angles, sendCmd, 300);
}

function stopQuardGait() {
  if (quardState.gaitTimer) {
    clearInterval(quardState.gaitTimer);
    quardState.gaitTimer = null;
  }
  quardState.gaitMode = null;
  setGaitLabel('stand');
}

function setGaitLabel(mode) {
  const el = document.getElementById('lbl-gait-mode');
  if (el) el.innerText = { forward: 'Walking forward', backward: 'Walking back', left: 'Turning left', right: 'Turning right' }[mode] || 'Stand';
}

// The firmware stands the robot up after 2 s without a /cmd request
// (LOCOMOTION_WATCHDOG_MS), so keep re-sending the active gait once a second.
// Repeating the same mode does not restart the gait on the robot.
setInterval(() => {
  if (quardState.gaitMode && !estopEngaged) sendQuardCmd({ mode: quardState.gaitMode }, false, true);
}, 1000);

function playQuardWaveAnimation() {
  if (quardState.waveTimer) clearInterval(quardState.waveTimer);
  stopQuardGait();

  // High stance first
  setQuardPose([60, 60, 60, 60, 60, 60, 60, 60], false);

  let step = 0;
  quardState.waveTimer = setInterval(() => {
    step++;
    const waveAngle = (step % 2 === 0) ? 0 : 35;
    quardState.channels[5] = waveAngle;
    quardState.joints['joint_r3_foot'] = channelAngleToRad(5, waveAngle);
    const slider = document.getElementById(`slider-quard-ch-5`);
    const readout = document.getElementById(`readout-quard-ch-5`);
    if (slider) slider.value = waveAngle;
    if (readout) readout.innerText = `${waveAngle}°`;

    updateQuardKinematics();
    syncQuardJointsToServer();

    if (step >= 8) {
      clearInterval(quardState.waveTimer);
      quardState.waveTimer = null;
      setTimeout(() => {
        setQuardPose([45, 45, 45, 45, 45, 45, 45, 45], false);
      }, 300);
    }
  }, 220);
}

function startQuardGait(mode) {
  stopQuardGait();
  quardState.gaitMode = mode;
  setGaitLabel(mode);

  let phase = 0;
  const stepDelay = 220; // Walking gait trot cycle

  quardState.gaitTimer = setInterval(() => {
    phase = (phase + 1) % 4;

    let chs = [45, 45, 45, 45, 45, 45, 45, 45];
    if (mode === 'forward') {
      if (phase === 0) {
        chs[0] = 70; chs[3] = 70; chs[5] = 25; chs[7] = 65;
      } else if (phase === 1) {
        chs[0] = 45; chs[3] = 45; chs[5] = 45; chs[7] = 45;
      } else if (phase === 2) {
        chs[1] = 20; chs[2] = 20; chs[4] = 65; chs[6] = 25;
      } else {
        chs[1] = 45; chs[2] = 45; chs[4] = 45; chs[6] = 45;
      }
    } else if (mode === 'backward') {
      if (phase === 0) {
        chs[1] = 70; chs[2] = 70; chs[4] = 25; chs[6] = 65;
      } else if (phase === 1) {
        chs[1] = 45; chs[2] = 45; chs[4] = 45; chs[6] = 45;
      } else if (phase === 2) {
        chs[0] = 20; chs[3] = 20; chs[5] = 65; chs[7] = 25;
      } else {
        chs[0] = 45; chs[3] = 45; chs[5] = 45; chs[7] = 45;
      }
    } else if (mode === 'left') {
      if (phase === 0) {
        chs[0] = 70; chs[1] = 70; chs[5] = 30; chs[4] = 60;
      } else if (phase === 1) {
        chs[0] = 45; chs[1] = 45; chs[5] = 45; chs[4] = 45;
      } else if (phase === 2) {
        chs[2] = 20; chs[3] = 20; chs[6] = 60; chs[7] = 30;
      } else {
        chs[2] = 45; chs[3] = 45; chs[6] = 45; chs[7] = 45;
      }
    } else if (mode === 'right') {
      if (phase === 0) {
        chs[2] = 70; chs[3] = 70; chs[6] = 30; chs[7] = 60;
      } else if (phase === 1) {
        chs[2] = 45; chs[3] = 45; chs[6] = 45; chs[7] = 45;
      } else if (phase === 2) {
        chs[0] = 20; chs[1] = 20; chs[5] = 60; chs[4] = 30;
      } else {
        chs[0] = 45; chs[1] = 45; chs[5] = 45; chs[4] = 45;
      }
    }

    for (let c = 0; c < 8; c++) {
      quardState.channels[c] = chs[c];
      const jInfo = CHANNEL_TO_JOINT[c];
      if (jInfo) {
        quardState.joints[jInfo.id] = channelAngleToRad(c, chs[c]);
      }
      const readout = document.getElementById(`readout-quard-ch-${c}`);
      const slider = document.getElementById(`slider-quard-ch-${c}`);
      if (readout) readout.innerText = `${chs[c]}°`;
      if (slider) slider.value = chs[c];
    }
    updateQuardKinematics();
  }, stepDelay);
}

function setupQuardEventListeners() {
  // Locomotion D-Pad Buttons
  const dpadButtons = document.querySelectorAll('.btn-dpad');
  dpadButtons.forEach(btn => {
    const cmd = btn.dataset.cmd;
    btn.addEventListener('click', () => {
      dpadButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');

      if (cmd === 'stand') {
        stopQuardGait();
        setQuardPose([45, 45, 45, 45, 45, 45, 45, 45], false);
        sendQuardCmd({ mode: 'stand' });
      } else {
        startQuardGait(cmd);
        sendQuardCmd({ mode: cmd });
      }
    });
  });

  // Preset Stances Buttons
  document.querySelectorAll('.btn-preset').forEach(btn => {
    btn.addEventListener('click', () => {
      const preset = btn.dataset.preset;
      stopQuardGait();
      if (quardState.waveTimer) {
        clearInterval(quardState.waveTimer);
        quardState.waveTimer = null;
      }

      if (preset === 'stand') {
        setQuardPose([45, 45, 45, 45, 45, 45, 45, 45], false);
        sendQuardCmd({ mode: 'stand' });
      } else if (preset === 'zero') {
        setQuardPose([0, 0, 0, 0, 0, 0, 0, 0], false);
        sendQuardCmd({ mode: 'zero' });
      } else if (preset === 'crouch') {
        setQuardPose([30, 30, 30, 30, 30, 30, 30, 30], false);
        sendQuardCmd({ all: 30 });
      } else if (preset === 'high') {
        setQuardPose([60, 60, 60, 60, 60, 60, 60, 60], false);
        sendQuardCmd({ all: 60 });
      } else if (preset === 'wave') {
        playQuardWaveAnimation();
        sendQuardCmd({ mode: 'wave' });
      }
    });
  });

  // OLED Face Buttons
  document.querySelectorAll('.btn-face').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.btn-face').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const face = btn.dataset.face;
      sendQuardCmd({ face: face });
    });
  });

  // Set All Servos Slider
  const sliderAll = document.getElementById('slider-quard-all');
  if (sliderAll) {
    sliderAll.addEventListener('input', (e) => {
      const angle = parseInt(e.target.value);
      document.getElementById('readout-quard-all').innerText = `${angle}°`;
      setQuardPose([angle, angle, angle, angle, angle, angle, angle, angle], true);
    });
  }

  // Ping Test Button
  const btnPing = document.getElementById('btn-quard-ping');
  if (btnPing) {
    btnPing.addEventListener('click', () => pingQuardBot(true));
  }
}

// Keyboard WASD / Arrow locomotion controls for Quard Bot
function setupQuardKeyboard() {
  window.addEventListener('keydown', (e) => {
    if (activeTab !== 'quard' || estopEngaged) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (['input', 'select', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;

    let mode = null;
    switch (e.key.toLowerCase()) {
      case 'w': case 'arrowup':    mode = 'forward'; break;
      case 's': case 'arrowdown':  mode = 'backward'; break;
      case 'a': case 'arrowleft':  mode = 'left'; break;
      case 'd': case 'arrowright': mode = 'right'; break;
      case ' ':                   mode = 'stand'; break;
    }

    if (mode) {
      e.preventDefault();
      const btn = document.querySelector(`.btn-dpad[data-cmd="${mode}"]`);
      if (btn) btn.click();
    }
  });
}

// Quard commands are sent strictly in order, one at a time. While a request is
// in flight, repeated angle updates for the same channel collapse into the
// latest value instead of queueing dozens of stale angles.
const quardCmdQueue = [];
let quardCmdInFlight = false;
const QUARD_STALE_MS = 2000;   // motion older than this is never replayed

function quardHost() {
  const el = document.getElementById('input-quard-host');
  return (el && el.value.trim()) || 'http://sesame-robot.local';
}

function isStandParams(params) {
  const m = String(params.mode !== undefined ? params.mode : (params.move !== undefined ? params.move : '')).toLowerCase();
  return ['s', 'stand', 'stop'].includes(m) && Object.keys(params).every(k => ['mode', 'move', 'cmd'].includes(k));
}

function sendQuardCmd(params, force = false, keepalive = false) {
  if (estopEngaged && !force) return;
  const host = quardHost();
  const entry = { host, params, keepalive, t: Date.now(), always: isStandParams(params) };

  // Only the newest gait/mode matters: drop older queued ones so a slow or
  // unreachable robot does not replay stale walk commands later.
  if (params.mode !== undefined || params.move !== undefined) {
    for (let i = quardCmdQueue.length - 1; i >= 0; i--) {
      const qp = quardCmdQueue[i].params;
      if (qp.mode !== undefined || qp.move !== undefined) quardCmdQueue.splice(i, 1);
    }
  }

  const last = quardCmdQueue[quardCmdQueue.length - 1];
  if (last && last.host === host && params.ch !== undefined && last.params.ch === params.ch) {
    quardCmdQueue[quardCmdQueue.length - 1] = entry;
  } else if (keepalive && quardCmdQueue.length > 0) {
    return; // something is already queued, which also resets the watchdog
  } else {
    quardCmdQueue.push(entry);
  }
  flushQuardCmdQueue();
}

let lastQuardRejectToast = 0;

function flushQuardCmdQueue() {
  if (quardCmdInFlight || quardCmdQueue.length === 0) return;
  const entry = quardCmdQueue.shift();
  const waited = Date.now() - entry.t;
  if (waited > QUARD_STALE_MS && !entry.always) {
    // Queued behind a slow request: moving now would surprise whoever is watching
    if (!entry.keepalive) {
      const q = new URLSearchParams(entry.params).toString();
      consoles.quard.log(`/cmd?${q}  not sent: waited ${(waited / 1000).toFixed(1)} s behind a slow request (motion is never replayed late)`, 'warn');
    }
    flushQuardCmdQueue();
    return;
  }
  quardCmdInFlight = true;

  postJSON('/api/quardbot/cmd', { host: entry.host, params: entry.params, keepalive: entry.keepalive })
  .then(data => {
    if (data.status === 'rejected' && Date.now() - lastQuardRejectToast > 3000) {
      // The robot answered, so it is online; it just did not accept this command.
      lastQuardRejectToast = Date.now();
      const what = Object.keys(entry.params).join(', ');
      toast(`Quard Bot refused the command (${what}); the firmware may not support it.`, 'warn');
    }
  })
  .catch(err => console.warn('[Dashboard] Quard Bot command error:', err))
  .finally(() => {
    quardCmdInFlight = false;
    flushQuardCmdQueue();
  });
}

// Heartbeat: every 3 s while online, backing off 2 → 4 → 8 → 10 s while offline.
const quardPing = { gen: 0, inFlight: false, failures: 0, timer: null, nextAt: null, online: null };

function scheduleQuardPing() {
  clearTimeout(quardPing.timer);
  const auto = document.getElementById('chk-quard-auto');
  if (auto && !auto.checked) { quardPing.nextAt = null; return; }
  const delay = quardPing.online ? 3000 : Math.min(10000, 2000 * 2 ** Math.max(0, quardPing.failures - 1));
  quardPing.nextAt = Date.now() + delay;
  quardPing.timer = setTimeout(() => pingQuardBot(false), delay);
}

function pingQuardBot(manual = false) {
  if (quardPing.inFlight && !manual) return;   // a manual ping replaces a stuck one
  clearTimeout(quardPing.timer);
  const gen = ++quardPing.gen;
  quardPing.inFlight = true;
  if (manual) btnPingBusy(true);
  postJSON('/api/quardbot/ping', { host: quardHost(), manual })
    .then(data => {
      if (gen !== quardPing.gen) return;
      const online = data.online === true;
      quardPing.online = online;
      quardPing.failures = online ? 0 : quardPing.failures + 1;
      if (online) rememberQuardHost(quardHost());
    })
    .catch(() => { if (gen === quardPing.gen) { quardPing.online = false; quardPing.failures++; } })
    .finally(() => {
      if (gen !== quardPing.gen) return;
      quardPing.inFlight = false;
      btnPingBusy(false);
      scheduleQuardPing();
    });
}

function btnPingBusy(on) {
  const btn = document.getElementById('btn-quard-ping');
  if (btn) btn.classList.toggle('is-busy', on);
}

function rememberQuardHost(host) {
  let hosts = [];
  try { hosts = JSON.parse(storageGet('quard-hosts') || '[]'); } catch (e) { hosts = []; }
  if (hosts[0] === host) return;
  hosts = [host].concat(hosts.filter(h => h !== host)).slice(0, 6);
  storageSet('quard-hosts', JSON.stringify(hosts));
  fillQuardHostList(hosts);
}

function fillQuardHostList(hosts) {
  const list = document.getElementById('quard-hosts-list');
  if (list) list.replaceChildren(...hosts.map(h => new Option(h, h)));
}

function initQuardHost() {
  const input = document.getElementById('input-quard-host');
  let hosts = [];
  try { hosts = JSON.parse(storageGet('quard-hosts') || '[]'); } catch (e) { hosts = []; }
  fillQuardHostList(hosts);
  if (input && hosts[0]) input.value = hosts[0];
  let t = null;
  input.addEventListener('input', () => {
    // A new address: commands queued for the old one are pointless
    const dropped = quardCmdQueue.filter(e => !e.keepalive).length;
    quardCmdQueue.length = 0;
    if (dropped) consoles.quard.log(`Address changed: dropped ${dropped} queued command(s)`, 'warn');
    quardPing.gen++;             // ignore a ping still running for the old address
    quardPing.inFlight = false;
    clearTimeout(t);
    t = setTimeout(() => pingQuardBot(false), 900);
  });
  input.addEventListener('keydown', (e) => {
    e.stopPropagation();
    if (e.key === 'Enter') pingQuardBot(true);
  });
  const auto = document.getElementById('chk-quard-auto');
  if (auto) auto.addEventListener('change', () => (auto.checked ? pingQuardBot(false) : scheduleQuardPing()));
}

function quardConsoleSubmit(text) {
  const t = text.trim();
  if (['status', '/status', 'ping'].includes(t.toLowerCase())) { pingQuardBot(true); return; }
  let q = t;
  for (const prefix of ['/cmd?', 'cmd?', '?']) {
    if (q.toLowerCase().startsWith(prefix)) { q = q.slice(prefix.length); break; }
  }
  const params = Object.fromEntries(new URLSearchParams(q));
  delete params[''];
  if (!Object.keys(params).length || Object.values(params).some(v => v === '')) {
    consoles.quard.log(`Not understood: "${t}". Use key=value pairs such as move=w or ch=3&deg=70`, 'warn');
    return;
  }
  if (estopEngaged && !isStandParams(params)) {
    consoles.quard.log(`Blocked by E-STOP: ${t}`, 'warn');
    return;
  }
  sendQuardCmd(params, isStandParams(params));
}

// ── ESP32 Sensor Node Telemetry & Controls ────────────────────────────────
function initSensorControls() {
  const btnConnect = document.getElementById('btn-sensor-connect');
  const btnWifiConnect = document.getElementById('btn-sensor-wifi-connect');
  const btnWifiDisconnect = document.getElementById('btn-sensor-wifi-disconnect');
  const inputWifiIp = document.getElementById('sensor-wifi-ip');
  const savedIp = storageGet('rover-wifi-host');
  if (savedIp && inputWifiIp) inputWifiIp.value = savedIp;

  const connectWifi = () => {
    const ip = (inputWifiIp ? inputWifiIp.value.trim() : '') || '192.168.1.100';
    storageSet('rover-wifi-host', ip);
    withBusy(btnWifiConnect, postJSON('/api/sensors/wifi_connect', { ip }))
      .catch(() => toast('Could not reach the server.', 'error'));
  };
  if (btnWifiConnect) btnWifiConnect.addEventListener('click', connectWifi);
  if (inputWifiIp) inputWifiIp.addEventListener('keydown', (e) => { if (e.key === 'Enter') connectWifi(); });
  if (btnWifiDisconnect) {
    btnWifiDisconnect.addEventListener('click', () => {
      postJSON('/api/sensors/wifi_disconnect', {}).catch(() => toast('Could not reach the server.', 'error'));
    });
  }

  let forceUntil = 0;
  if (btnConnect) {
    btnConnect.addEventListener('click', () => {
      if (lastLinks && lastLinks.sensors && lastLinks.sensors.source === 'serial') {
        postJSON('/api/sensors/disconnect', {}).catch(() => toast('Could not reach the server.', 'error'));
        return;
      }
      const port = document.getElementById('sensor-serial-port').value.trim();
      const baud = parseInt(document.getElementById('sensor-serial-baud').value);
      if (!port) { toast('Pick a serial port first (⟳ rescans).', 'warn'); return; }
      const force = Date.now() < forceUntil;
      fetch('/api/sensors/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port, baudrate: baud, force })
      })
      .then(res => res.json().then(data => ({ code: res.status, data })))
      .then(({ code, data }) => {
        if (code === 409) {
          forceUntil = Date.now() + 10000;
          toast(`${data.message} Click Connect Serial again within 10 s to connect anyway.`, 'warn', 10000);
        } else if (data.status !== 'ok') {
          toast(`Not connected: ${data.message || 'error'}`, 'error');
        }
      })
      .catch(() => toast('Could not reach the server.', 'error'));
    });
  }

}

function startSensorPolling() {
  // Readings the rover hasn't sent are null on the server and shown as "--":
  // nothing on this panel is made up.
  const num = (v) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  const show = (id, text) => { const el = document.getElementById(id); if (el) el.innerText = text; };
  const meter = (id, pct) => {
    const el = document.getElementById(id);
    if (el) el.style.width = pct === null ? '0%' : `${Math.min(100, Math.max(0, pct)).toFixed(1)}%`;
  };
  const noData = (el) => setPill(el, null, 'No data');

  setInterval(() => {
    fetch('/api/sensors/data')
      .then(res => res.json())
      .then(data => {
        if (!data) return;
        sensorState = data;

        // 1. Emergency fire alert: only on a real LOW flame pin or FIRE alert
        const flame = num(data.flame);
        const alertStr = data.alert === null || data.alert === undefined ? null : String(data.alert).toUpperCase();
        const isFire = flame === 0 || alertStr === 'FIRE';
        const fireBanner = document.getElementById('banner-sensor-fire');
        if (fireBanner) fireBanner.classList.toggle('visible', isFire);

        // 2. Climate (DHT11 & BMP280)
        const tempC = num(data.temp_dht) !== null ? num(data.temp_dht) : num(data.temperature);
        const hum = num(data.humidity);
        const tempBMP = num(data.temp_bmp);
        const pressure = num(data.pressure);
        show('val-temp', tempC === null ? '-- °C' : `${tempC.toFixed(1)} °C`);
        show('val-temp-f', tempC === null ? ' ' : `(${(tempC * 9 / 5 + 32).toFixed(1)} °F)`);
        show('val-humidity', hum === null ? '-- %' : `${hum.toFixed(1)} %`);
        show('val-bmp-temp', tempBMP === null ? '-- °C' : `${tempBMP.toFixed(1)} °C`);
        show('val-pressure', pressure === null ? '-- hPa' : `${pressure.toFixed(1)} hPa`);
        meter('meter-temp', tempC === null ? null : (tempC / 50) * 100);
        meter('meter-humidity', hum);
        meter('meter-bmp-temp', tempBMP === null ? null : (tempBMP / 50) * 100);
        meter('meter-pressure', pressure === null ? null : pressure - 950);   // 950–1050 hPa scale
        const badgeDht = document.getElementById('badge-dht-status');
        if (data.dht_error) setPill(badgeDht, 'offline', 'DHT ERROR');
        else if (tempC === null && tempBMP === null) noData(badgeDht);
        else setPill(badgeDht, 'online', 'OK');

        // 3. Hazards: gas, water, flame
        const gas = num(data.gas);
        const water = num(data.water);
        const badgeGas = document.getElementById('badge-gas-status');
        const badgeWater = document.getElementById('badge-water-status');
        show('val-gas', gas === null ? '-- / 4095' : `${gas} / 4095`);
        meter('meter-gas', gas === null ? null : (gas / 4095) * 100);
        const meterGas = document.getElementById('meter-gas');
        if (meterGas) {
          meterGas.className = 'meter-fill ' + (gas === null ? 'meter-gas-fill'
            : gas > 2500 ? 'meter-gas-danger' : gas > 1500 ? 'meter-gas-warning' : 'meter-gas-fill');
        }
        if (gas === null) noData(badgeGas);
        else if (gas > 2500) setPill(badgeGas, 'offline', 'Hazard');
        else if (gas > 1500) setPill(badgeGas, 'warn', 'Moderate');
        else setPill(badgeGas, 'online', 'Clean Air');

        show('val-water', water === null ? '-- / 4095' : `${water} / 4095`);
        meter('meter-water', water === null ? null : (water / 4095) * 100);
        if (water === null) noData(badgeWater);
        else if (water > 2000) setPill(badgeWater, 'warn', 'Submerged');
        else if (water > 600) setPill(badgeWater, 'info', 'Moisture');
        else setPill(badgeWater, 'online', 'Dry');

        const elFlame = document.getElementById('val-flame');
        const badgeFlame = document.getElementById('badge-flame-status');
        if (elFlame) {
          elFlame.innerText = flame === null ? 'Pin DO: --' : flame === 0 ? 'Pin DO: LOW (🔥 Fire detected)' : 'Pin DO: HIGH (Clear)';
          elFlame.className = 'readout-pill ' + (flame === null ? '' : flame === 0 ? 'bad' : 'ok');
        }
        if (flame === null) noData(badgeFlame);
        else setPill(badgeFlame, flame === 0 ? 'offline' : 'online', flame === 0 ? '🚨 FIRE' : '✅ Clear');
        const valAlert = document.getElementById('val-alert-state');
        if (alertStr === null) setPill(valAlert, null, 'Alert: --');
        else setPill(valAlert, alertStr === 'OK' ? 'online' : 'offline', `Alert: ${alertStr}`);

        // Card-level summary of the hazard sensors
        const badgeHazard = document.getElementById('badge-hazard-status');
        if (isFire || (gas !== null && gas > 2500)) setPill(badgeHazard, 'offline', 'Danger');
        else if ((gas !== null && gas > 1500) || (water !== null && water > 2000)) setPill(badgeHazard, 'warn', 'Caution');
        else if (gas === null && water === null && flame === null) noData(badgeHazard);
        else setPill(badgeHazard, 'online', 'Safe');

        // 4. MPU6050 IMU
        const accel = data.accel && typeof data.accel === 'object' ? data.accel : null;
        const gyro = data.gyro && typeof data.gyro === 'object' ? data.gyro : null;
        const pitch = num(data.pitch);
        const roll = num(data.roll);
        const fx = (o, k, d) => (o && num(o[k]) !== null ? o[k].toFixed(d) : '--');
        show('val-accel-x', fx(accel, 'x', 2));
        show('val-accel-y', fx(accel, 'y', 2));
        show('val-accel-z', fx(accel, 'z', 2));
        show('val-gyro-x', fx(gyro, 'x', 3));
        show('val-gyro-y', fx(gyro, 'y', 3));
        show('val-gyro-z', fx(gyro, 'z', 3));
        show('val-pitch', pitch === null ? '--' : `${pitch.toFixed(1)}°`);
        show('val-roll', roll === null ? '--' : `${roll.toFixed(1)}°`);
        const badgeImu = document.getElementById('badge-imu-status');
        if (accel || gyro) setPill(badgeImu, 'online', 'Live'); else noData(badgeImu);

        // 5. GPS & SD status
        const gpsValid = data.gps_valid;
        const sats = num(data.gps_sats);
        const lat = num(data.lat);
        const lng = num(data.lng);
        const badgeGps = document.getElementById('badge-gps-status');
        if (gpsValid === true && lat !== null && lng !== null) {
          show('val-gps-coords', `${lat.toFixed(6)}, ${lng.toFixed(6)}`);
          setPill(badgeGps, 'online', '3D Fix');
        } else if (gpsValid === false) {
          show('val-gps-coords', 'No fix / searching…');
          setPill(badgeGps, 'offline', 'No Fix');
        } else {
          show('val-gps-coords', '--');
          noData(badgeGps);
        }
        show('val-gps-sats', sats === null ? '🛰️ -- satellites' : `🛰️ ${sats} satellite${sats === 1 ? '' : 's'}`);

        const elSdStatus = document.getElementById('val-sd-status');
        if (elSdStatus) {
          if (data.sd_ok === true) { elSdStatus.innerText = '💾 Ready (/log.csv)'; elSdStatus.className = 'metric-val sm ok'; }
          else if (data.sd_ok === false) { elSdStatus.innerText = '❌ No SD card'; elSdStatus.className = 'metric-val sm bad'; }
          else { elSdStatus.innerText = '--'; elSdStatus.className = 'metric-val sm'; }
        }
        const uptimeMs = num(data.uptime_ms);
        if (uptimeMs === null) {
          show('val-uptime', 'Uptime: --');
        } else {
          const uptimeSec = Math.floor(uptimeMs / 1000);
          show('val-uptime', `Uptime: ${Math.floor(uptimeSec / 60)}m ${uptimeSec % 60}s`);
        }
      })
      .catch(err => {});
  }, 200); // 5 Hz telemetry sync
}