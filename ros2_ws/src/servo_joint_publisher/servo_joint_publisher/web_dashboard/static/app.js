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

let sensorState = {
  temperature: 24.5,
  humidity: 48.0,
  pitch: 0,
  roll: 0,
  gas: 415,
  simulating: true,
  connected: false,
  raw_log: []
};

// ── Model Load & Dual View Management System ──────────────────────────────
const modelLoadSystem = {
  mode: 'auto', // 'auto' | 'quard' | 'arm' | 'dual' | 'sensors'
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
  loadSyncCalibration(applySavedJointCalibration);
  startStatePolling();
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
      const rawPitch = sensorState.raw_pitch !== undefined ? sensorState.raw_pitch : (sensorState.pitch || 0);
      const rawRoll = sensorState.raw_roll !== undefined ? sensorState.raw_roll : (sensorState.roll || 0);
      const pRad = (rawPitch - (imuOff.pitch || 0)) * (Math.PI / 180.0);
      const rRad = (rawRoll - (imuOff.roll || 0)) * (Math.PI / 180.0);
      robotMeshes.sensorGroup.rotation.x = pRad;
      robotMeshes.sensorGroup.rotation.z = -rRad;
    }
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
    if (effectiveMode === 'dual') {
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
    if (effectiveMode === 'dual') {
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
    if (effectiveMode === 'dual') {
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

// ── Real-time State Polling ──────────────────────────────────────────────
let serverOnline = null;
let stateFailures = 0;
let commModeEditedAt = 0;

function setServerOnline(online) {
  if (online === serverOnline) return;
  serverOnline = online;
  document.body.classList.toggle('server-offline', !online);
  if (!online) {
    setNodeBadge('badge-arm', 'offline', 'Server offline');
    setPill(document.getElementById('pill-arm-link'), 'offline', 'Server offline');
  }
}

function updateArmLinkStatus(data) {
  const viaWifi = String(data.comm_mode || '').toUpperCase() === 'WIFI';
  const link = viaWifi ? 'Wi-Fi' : 'Serial';
  if (data.ros_bridge === true) {
    setNodeBadge('badge-arm', 'online', `ROS 2 · ${link}`);
    setPill(document.getElementById('pill-arm-link'), 'online', `ROS 2 bridge · ${link}`);
  } else if (data.ros_bridge === false) {
    setNodeBadge('badge-arm', 'warn', 'UI only (no ROS)');
    setPill(document.getElementById('pill-arm-link'), 'warn', 'No ROS 2 — not sent to arm');
  }
  // Don't fight a toggle the user flipped a moment ago.
  if (data.comm_mode && Date.now() - commModeEditedAt > 1500) syncCommModeUI(viaWifi);
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
        updateArmLinkStatus(data);

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

  const panelArm = document.getElementById('panel-arm-control');
  const panelQuard = document.getElementById('panel-quard-control');
  const panelSensors = document.getElementById('panel-sensors-control');

  function setActive(tabName, btn, panel) {
    activeTab = tabName;
    storageSet('dashboard-tab', tabName);
    [btnArm, btnQuard, btnSensors].forEach(b => {
      if (!b) return;
      b.classList.remove('active');
      b.setAttribute('aria-selected', 'false');
      b.tabIndex = -1;
    });
    [panelArm, panelQuard, panelSensors].forEach(p => p && p.classList.remove('active'));

    if (btn) {
      btn.classList.add('active');
      btn.setAttribute('aria-selected', 'true');
      btn.tabIndex = 0;
    }
    if (panel) panel.classList.add('active');
  }

  // Arrow keys move between tabs (WAI-ARIA tabs pattern)
  const tabs = [btnArm, btnQuard, btnSensors].filter(Boolean);
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
  pingQuardBot();
  setInterval(pingQuardBot, 5000); // 5s periodic heartbeat check
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
    btnPing.addEventListener('click', pingQuardBot);
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

function sendQuardCmd(params, force = false, keepalive = false) {
  if (estopEngaged && !force) return;
  const host = document.getElementById('input-quard-host') ? document.getElementById('input-quard-host').value.trim() : 'http://sesame-robot.local';
  const entry = { host: host, params: params, keepalive: keepalive };

  // Only the newest gait/mode matters: drop older queued ones so a slow or
  // unreachable robot does not replay stale walk commands later.
  if (params.mode !== undefined) {
    for (let i = quardCmdQueue.length - 1; i >= 0; i--) {
      if (quardCmdQueue[i].params.mode !== undefined) quardCmdQueue.splice(i, 1);
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
  quardCmdInFlight = true;

  postJSON('/api/quardbot/cmd', { host: entry.host, params: entry.params })
  .then(data => {
    if (data.status === 'rejected') {
      // The robot answered, so it is online; it just did not accept this command.
      updateQuardStatusPill(true);
      if (Date.now() - lastQuardRejectToast > 3000) {
        lastQuardRejectToast = Date.now();
        const what = Object.keys(entry.params).join(', ');
        toast(`Quard Bot rejected the command (${what}); the firmware may not support it.`, 'warn');
      }
    } else {
      updateQuardStatusPill(data.status === 'ok');
    }
  })
  .catch(err => {
    console.warn('[Dashboard] Quard Bot command error:', err);
    updateQuardStatusPill(false);
  })
  .finally(() => {
    quardCmdInFlight = false;
    flushQuardCmdQueue();
  });
}

function pingQuardBot(e) {
  if (e && e.type === 'click') btnPingBusy(true);
  const host = document.getElementById('input-quard-host') ? document.getElementById('input-quard-host').value.trim() : 'http://sesame-robot.local';

  fetch('/api/quardbot/ping', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ host: host })
  })
  .then(res => res.json())
  .then(data => {
    const online = data.online === true;
    updateQuardStatusPill(online);
  })
  .catch(() => updateQuardStatusPill(false))
  .finally(() => btnPingBusy(false));
}

function btnPingBusy(on) {
  const btn = document.getElementById('btn-quard-ping');
  if (btn) btn.classList.toggle('is-busy', on);
}

function updateQuardStatusPill(online) {
  setPill(document.getElementById('pill-quard-status'), online ? 'online' : 'offline', online ? 'Connected' : 'Offline');
  setNodeBadge('badge-quardbot', online ? 'online' : 'offline', online ? 'Connected' : 'Offline');
}

// ── ESP32 Sensor Node Telemetry & Controls ────────────────────────────────
function initSensorControls() {
  const btnConnect = document.getElementById('btn-sensor-connect');
  const btnSim = document.getElementById('btn-sensors-sim');
  const btnClear = document.getElementById('btn-clear-sensor-log');
  const btnWifiConnect = document.getElementById('btn-sensor-wifi-connect');
  const btnWifiDisconnect = document.getElementById('btn-sensor-wifi-disconnect');
  const inputWifiIp = document.getElementById('sensor-wifi-ip');

  if (btnWifiConnect) {
    btnWifiConnect.addEventListener('click', () => {
      const ip = (inputWifiIp ? inputWifiIp.value.trim() : '') || '192.168.1.100';
      btnWifiConnect.classList.add('is-busy');
      btnWifiConnect.disabled = true;

      fetch('/api/sensors/wifi_connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ip: ip })
      })
      .then(res => res.json())
      .then(data => {
        btnWifiConnect.classList.remove('is-busy');
        btnWifiConnect.disabled = false;
        if (btnWifiDisconnect) btnWifiDisconnect.style.display = 'inline-block';
        const pill = document.getElementById('pill-sensor-wifi-status');
        if (pill) {
          pill.innerText = `Connected (${ip})`;
          pill.className = 'status-pill status-online';
        }
      })
      .catch(() => {
        btnWifiConnect.classList.remove('is-busy');
        btnWifiConnect.disabled = false;
        toast('Could not reach the server.', 'error');
      });
    });
  }

  if (btnWifiDisconnect) {
    btnWifiDisconnect.addEventListener('click', () => {
      fetch('/api/sensors/wifi_disconnect', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
      .then(res => res.json())
      .then(data => {
        btnWifiDisconnect.style.display = 'none';
        const pill = document.getElementById('pill-sensor-wifi-status');
        if (pill) {
          pill.innerText = 'Wi-Fi: Offline';
          pill.className = 'status-pill status-offline';
        }
      });
    });
  }

  if (btnConnect) {
    btnConnect.addEventListener('click', () => {
      const port = document.getElementById('sensor-serial-port').value;
      const baud = parseInt(document.getElementById('sensor-serial-baud').value);

      fetch('/api/sensors/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port: port, baudrate: baud })
      })
      .then(res => res.json())
      .then(() => toast(`🔌 Reading sensor node on ${port} @ ${baud} baud…`, 'info'))
      .catch(() => toast('Could not reach the server.', 'error'));
    });
  }

  if (btnSim) {
    btnSim.addEventListener('click', () => {
      sensorState.simulating = !sensorState.simulating;
      sensorSimEditedAt = Date.now();
      updateSensorSimButton(sensorState.simulating);

      fetch('/api/sensors/sim', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ simulating: sensorState.simulating })
      });
    });
  }

  if (btnClear) {
    btnClear.addEventListener('click', () => {
      const consoleLog = document.getElementById('sensor-console-log');
      if (consoleLog) consoleLog.innerText = '';
    });
  }
}

// The button names the action it performs, not the current state.
function updateSensorSimButton(simulating) {
  const btnSim = document.getElementById('btn-sensors-sim');
  if (!btnSim) return;
  btnSim.innerText = simulating ? '⏸️ Stop Simulation' : '▶️ Simulate Data';
  btnSim.className = simulating ? 'btn btn-warning btn-sm' : 'btn btn-outline btn-sm';
}

let lastSensorLog = '';
let sensorSimEditedAt = 0;

function startSensorPolling() {
  setInterval(() => {
    fetch('/api/sensors/data')
      .then(res => res.json())
      .then(data => {
        if (!data) return;
        sensorState = data;

        // 1. Emergency Fire Alert Banner
        const flameVal = data.flame !== undefined ? data.flame : 1;
        const alertStr = String(data.alert || 'OK').toUpperCase();
        const isFire = (flameVal === 0 || alertStr === 'FIRE');
        const fireBanner = document.getElementById('banner-sensor-fire');
        if (fireBanner) fireBanner.classList.toggle('visible', isFire);
        // A poll that left before the toggle POST landed would flip it back.
        if (Date.now() - sensorSimEditedAt > 1000) updateSensorSimButton(Boolean(data.simulating));

        // 2. DHT11 Climate & BMP280
        const tempC = data.temp_dht !== undefined ? data.temp_dht : (data.temperature || 24.5);
        const tempF = (tempC * 9/5) + 32;
        const hum = data.humidity !== undefined ? data.humidity : 48.0;
        const tempBMP = data.temp_bmp !== undefined ? data.temp_bmp : 24.8;
        const pressure = data.pressure !== undefined ? data.pressure : 1013.25;

        const elTemp = document.getElementById('val-temp');
        const elTempF = document.getElementById('val-temp-f');
        const elHum = document.getElementById('val-humidity');
        const elBmpTemp = document.getElementById('val-bmp-temp');
        const elPressure = document.getElementById('val-pressure');
        const meterTemp = document.getElementById('meter-temp');
        const meterHum = document.getElementById('meter-humidity');
        const meterBmpTemp = document.getElementById('meter-bmp-temp');
        const meterPressure = document.getElementById('meter-pressure');
        const badgeDht = document.getElementById('badge-dht-status');

        if (elTemp) elTemp.innerText = `${tempC.toFixed(1)} °C`;
        if (elTempF) elTempF.innerText = `(${tempF.toFixed(1)} °F)`;
        if (elHum) elHum.innerText = `${hum.toFixed(1)} %`;
        if (elBmpTemp) elBmpTemp.innerText = `${tempBMP.toFixed(1)} °C`;
        if (elPressure) elPressure.innerText = `${pressure.toFixed(1)} hPa`;

        if (meterTemp) meterTemp.style.width = `${Math.min(100, Math.max(0, (tempC / 50) * 100))}%`;
        if (meterHum) meterHum.style.width = `${Math.min(100, Math.max(0, hum))}%`;
        if (meterBmpTemp) meterBmpTemp.style.width = `${Math.min(100, Math.max(0, (tempBMP / 50) * 100))}%`;
        if (meterPressure) {
          // Normalize pressure range roughly 950 - 1050 hPa
          const pPct = Math.min(100, Math.max(0, ((pressure - 950) / 100) * 100));
          meterPressure.style.width = `${pPct}%`;
        }

        setPill(badgeDht, data.dht_error ? 'offline' : 'online', data.dht_error ? 'DHT ERROR' : 'OK');

        // 3. Hazards: Gas, Water, Flame
        const gasVal = data.gas !== undefined ? data.gas : 415;
        const waterVal = data.water !== undefined ? data.water : 120;

        const elGas = document.getElementById('val-gas');
        const meterGas = document.getElementById('meter-gas');
        const badgeGas = document.getElementById('badge-gas-status');
        const elWater = document.getElementById('val-water');
        const meterWater = document.getElementById('meter-water');
        const badgeWater = document.getElementById('badge-water-status');
        const elFlame = document.getElementById('val-flame');
        const badgeFlame = document.getElementById('badge-flame-status');
        const valAlert = document.getElementById('val-alert-state');

        if (elGas) elGas.innerText = `${gasVal} / 4095`;
        if (meterGas) {
          const pct = Math.min(100, Math.max(0, (gasVal / 4095) * 100));
          meterGas.style.width = `${pct.toFixed(1)}%`;
          if (gasVal > 2500) {
            meterGas.className = 'meter-fill meter-gas-danger';
          } else if (gasVal > 1500) {
            meterGas.className = 'meter-fill meter-gas-warning';
          } else {
            meterGas.className = 'meter-fill meter-gas-fill';
          }
        }
        if (gasVal > 2500) setPill(badgeGas, 'offline', 'Hazard');
        else if (gasVal > 1500) setPill(badgeGas, 'warn', 'Moderate');
        else setPill(badgeGas, 'online', 'Clean Air');

        if (elWater) elWater.innerText = `${waterVal} / 4095`;
        if (meterWater) {
          const wPct = Math.min(100, Math.max(0, (waterVal / 4095) * 100));
          meterWater.style.width = `${wPct.toFixed(1)}%`;
        }
        if (waterVal > 2000) setPill(badgeWater, 'warn', 'Submerged');
        else if (waterVal > 600) setPill(badgeWater, 'info', 'Moisture');
        else setPill(badgeWater, 'online', 'Dry');

        if (elFlame) {
          elFlame.innerText = (flameVal === 0) ? 'Pin DO: LOW (🔥 Fire detected)' : 'Pin DO: HIGH (Clear)';
          elFlame.className = 'readout-pill ' + (flameVal === 0 ? 'bad' : 'ok');
        }
        setPill(badgeFlame, flameVal === 0 ? 'offline' : 'online', flameVal === 0 ? '🚨 FIRE' : '✅ Clear');
        setPill(valAlert, alertStr === 'OK' ? 'online' : 'offline', `Alert: ${alertStr}`);

        // Card-level summary of all three hazard sensors
        const badgeHazard = document.getElementById('badge-hazard-status');
        if (isFire || gasVal > 2500) setPill(badgeHazard, 'offline', 'Danger');
        else if (gasVal > 1500 || waterVal > 2000) setPill(badgeHazard, 'warn', 'Caution');
        else setPill(badgeHazard, 'online', 'Safe');

        // 4. MPU6050 IMU
        const accel = data.accel || { x: 0.12, y: -0.05, z: 9.81 };
        const gyro = data.gyro || { x: 0.01, y: 0.00, z: -0.02 };
        const pitch = data.pitch !== undefined ? data.pitch : 0.0;
        const roll = data.roll !== undefined ? data.roll : 0.0;

        const elAx = document.getElementById('val-accel-x');
        const elAy = document.getElementById('val-accel-y');
        const elAz = document.getElementById('val-accel-z');
        const elGx = document.getElementById('val-gyro-x');
        const elGy = document.getElementById('val-gyro-y');
        const elGz = document.getElementById('val-gyro-z');
        const elPitch = document.getElementById('val-pitch');
        const elRoll = document.getElementById('val-roll');

        if (elAx) elAx.innerText = accel.x.toFixed(2);
        if (elAy) elAy.innerText = accel.y.toFixed(2);
        if (elAz) elAz.innerText = accel.z.toFixed(2);
        if (elGx) elGx.innerText = gyro.x.toFixed(3);
        if (elGy) elGy.innerText = gyro.y.toFixed(3);
        if (elGz) elGz.innerText = gyro.z.toFixed(3);
        if (elPitch) elPitch.innerText = `${pitch.toFixed(1)}°`;
        if (elRoll) elRoll.innerText = `${roll.toFixed(1)}°`;

        // 5. GPS & SD Status
        const gpsValid = Boolean(data.gps_valid);
        const sats = data.gps_sats !== undefined ? data.gps_sats : 0;
        const lat = data.lat !== undefined ? data.lat : 0.0;
        const lng = data.lng !== undefined ? data.lng : 0.0;

        const elGpsCoords = document.getElementById('val-gps-coords');
        const elGpsSats = document.getElementById('val-gps-sats');
        const badgeGps = document.getElementById('badge-gps-status');
        const elSdStatus = document.getElementById('val-sd-status');
        const elUptime = document.getElementById('val-uptime');

        if (elGpsCoords) {
          elGpsCoords.innerText = gpsValid ? `${lat.toFixed(6)}, ${lng.toFixed(6)}` : 'No Fix / Searching...';
        }
        if (elGpsSats) {
          elGpsSats.innerText = `🛰️ ${sats} satellite${sats === 1 ? '' : 's'}`;
        }
        setPill(badgeGps, gpsValid ? 'online' : 'offline', gpsValid ? '3D Fix' : 'No Fix');

        const sdOk = data.sd_ok !== undefined ? data.sd_ok : true;
        if (elSdStatus) {
          elSdStatus.innerText = sdOk ? '💾 Ready (/log.csv)' : '❌ No SD card';
          elSdStatus.className = 'metric-val sm ' + (sdOk ? 'ok' : 'bad');
        }
        if (elUptime) {
          const uptimeSec = Math.floor((data.uptime_ms || 0) / 1000);
          const mins = Math.floor(uptimeSec / 60);
          const secs = uptimeSec % 60;
          elUptime.innerText = `Uptime: ${mins}m ${secs}s`;
        }

        // Update Wi-Fi status pill
        const pillWifi = document.getElementById('pill-sensor-wifi-status');
        if (data.wifi_connected) setPill(pillWifi, 'online', `Wi-Fi: ${data.wifi_ip || 'online'}`);
        else if (data.connected) setPill(pillWifi, 'info', `Serial: ${data.port || ''}`);
        else if (data.simulating) setPill(pillWifi, 'warn', 'Simulated data');
        else setPill(pillWifi, 'offline', 'Wi-Fi: Offline');

        // Update Console Log
        const elConsole = document.getElementById('sensor-console-log');
        if (elConsole && data.raw_log && data.raw_log.length > 0) {
          const text = data.raw_log.slice(-30).join('\n');
          if (text !== lastSensorLog) {
            lastSensorLog = text;
            // Only follow the tail if the user hasn't scrolled up to read.
            const atBottom = elConsole.scrollHeight - elConsole.scrollTop - elConsole.clientHeight < 24;
            elConsole.innerText = text;
            if (atBottom) elConsole.scrollTop = elConsole.scrollHeight;
          }
        }

        // Update Top Header Badge
        if (data.wifi_connected) setNodeBadge('badge-sensornode', 'online', 'Wi-Fi');
        else if (data.connected) setNodeBadge('badge-sensornode', 'online', 'Serial');
        else if (data.simulating) setNodeBadge('badge-sensornode', 'warn', 'Simulated');
        else setNodeBadge('badge-sensornode', 'offline', 'Disconnected');
      })
      .catch(err => {});
  }, 200); // 5 Hz telemetry sync
}
