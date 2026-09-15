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

// ── Visual SLAM 3D & Telemetry State ───────────────────────────────────────
let slamMeshes = {
  container: null,
  pointCloud: null,
  trajectoryLine: null,
  cameraFrustum: null,
  tofRayLine: null
};

let slamState = {
  active: false,
  viewMode: 'cam' // 'cam' | 'map' | 'dual'
};

// ── Model Load & Dual View Management System ──────────────────────────────
const modelLoadSystem = {
  mode: 'auto', // 'auto' | 'quard' | 'arm' | 'dual' | 'sensors' | 'slam'
  loaded: {
    arm: false,
    quard: false,
    sensors: true,
    slam: true
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

// ── Twin Orientation Synchronization System ──────────────────────────────
let syncCalibration = {};

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
  buildJointControls();
  setupTabs();
  initModelLoadSystem();
  initQuardBotControls();
  initSensorControls();
  setupEventListeners();
  loadSyncCalibration();
  startStatePolling();
  startSensorPolling();
  initSLAMControls();
  startSLAMPolling();
});

// ── Three.js 3D Viewport Setup ─────────────────────────────────────────────
function initThreeJS() {
  const container = document.getElementById('canvas-container');
  const width = container.clientWidth;
  const height = container.clientHeight;

  // Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf1f5f9);

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

  // Grid Floor Helper (Light Theme)
  const gridHelper = new THREE.GridHelper(500, 50, 0x0284c7, 0xcbd5e1);
  gridHelper.position.y = -0.5;
  scene.add(gridHelper);

  // Initialize procedural IMU Sensor Visual
  createSensorVisual();

  // Initialize Visual SLAM 3D Point Cloud & Frustum
  initSLAMThreeJS();

  // Window Resize Listener
  window.addEventListener('resize', onWindowResize);

  // Animation Render Loop
  animate();
}

function createSensorVisual() {
  const boxGeom = new THREE.BoxGeometry(70, 12, 45);
  const boxMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.2, metalness: 0.8 });
  const sensorMesh = new THREE.Mesh(boxGeom, boxMat);
  sensorMesh.castShadow = true;

  const chipGeom = new THREE.BoxGeometry(18, 4, 18);
  const chipMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, roughness: 0.1, metalness: 0.9 });
  const chipMesh = new THREE.Mesh(chipGeom, chipMat);
  chipMesh.position.set(0, 8, 0);
  sensorMesh.add(chipMesh);

  robotMeshes.sensorGroup = new THREE.Group();
  robotMeshes.sensorGroup.position.set(0, 50, 0);
  robotMeshes.sensorGroup.add(sensorMesh);
  robotMeshes.sensorGroup.visible = false;
  scene.add(robotMeshes.sensorGroup);
}

function initSLAMThreeJS() {
  slamMeshes.container = new THREE.Group();
  slamMeshes.container.name = 'slam_container';
  slamMeshes.container.visible = false;
  scene.add(slamMeshes.container);

  // 1. Sparse 3D Point Cloud with RGB vertex colors
  const maxPts = 3000;
  const posArr = new Float32Array(maxPts * 3);
  const colArr = new Float32Array(maxPts * 3);
  for (let i = 0; i < maxPts; i++) {
    colArr[i * 3 + 0] = 0.0;
    colArr[i * 3 + 1] = 0.85;
    colArr[i * 3 + 2] = 1.0;
  }
  const ptGeom = new THREE.BufferGeometry();
  ptGeom.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
  ptGeom.setAttribute('color', new THREE.BufferAttribute(colArr, 3));
  ptGeom.setDrawRange(0, 0);

  const ptMat = new THREE.PointsMaterial({
    size: 5,
    vertexColors: true,
    transparent: true,
    opacity: 0.95
  });
  slamMeshes.pointCloud = new THREE.Points(ptGeom, ptMat);
  slamMeshes.pointCloud.frustumCulled = false;
  slamMeshes.container.add(slamMeshes.pointCloud);

  // 2. Camera Trajectory Path Line
  const maxTraj = 500;
  const trajArr = new Float32Array(maxTraj * 3);
  const trajGeom = new THREE.BufferGeometry();
  trajGeom.setAttribute('position', new THREE.BufferAttribute(trajArr, 3));
  trajGeom.setDrawRange(0, 0);
  const trajMat = new THREE.LineBasicMaterial({ color: 0xef4444, linewidth: 2 });
  slamMeshes.trajectoryLine = new THREE.Line(trajGeom, trajMat);
  slamMeshes.trajectoryLine.frustumCulled = false;
  slamMeshes.container.add(slamMeshes.trajectoryLine);

  // 3. 3D Camera Wireframe Frustum & Orientation Axes
  const frustumGroup = new THREE.Group();
  const s = 22.0;
  const fCorners = new Float32Array([
    0, 0, 0,
    -s * 0.8, -s * 0.6, s * 1.2,
     s * 0.8, -s * 0.6, s * 1.2,
     s * 0.8,  s * 0.6, s * 1.2,
    -s * 0.8,  s * 0.6, s * 1.2
  ]);
  const fIndices = [
    0, 1, 0, 2, 0, 3, 0, 4,
    1, 2, 2, 3, 3, 4, 4, 1
  ];
  const fGeom = new THREE.BufferGeometry();
  fGeom.setAttribute('position', new THREE.BufferAttribute(fCorners, 3));
  fGeom.setIndex(fIndices);
  const fMat = new THREE.LineBasicMaterial({ color: 0x0284c7, linewidth: 2 });
  const fMesh = new THREE.LineSegments(fGeom, fMat);
  frustumGroup.add(fMesh);

  const axes = new THREE.AxesHelper(24);
  frustumGroup.add(axes);

  slamMeshes.cameraFrustum = frustumGroup;
  slamMeshes.container.add(slamMeshes.cameraFrustum);

  // 4. VL53L1X ToF Laser Ray Line
  const tofArr = new Float32Array(6);
  const tofGeom = new THREE.BufferGeometry();
  tofGeom.setAttribute('position', new THREE.BufferAttribute(tofArr, 3));
  const tofMat = new THREE.LineBasicMaterial({ color: 0x10b981, linewidth: 2 });
  slamMeshes.tofRayLine = new THREE.Line(tofGeom, tofMat);
  slamMeshes.tofRayLine.frustumCulled = false;
  slamMeshes.container.add(slamMeshes.tofRayLine);
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

function syncQuardJointsToServer() {
  fetch('/api/quardbot/joints', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      joints: quardState.joints,
      channels: quardState.channels
    })
  }).catch(() => {});
}

// ── Model Loading & Dual View Coordination ───────────────────────────────
function getEffectiveModelMode() {
  if (modelLoadSystem.mode === 'auto') {
    if (activeTab === 'quard') return 'quard';
    if (activeTab === 'sensors') return 'sensors';
    if (activeTab === 'slam') return 'slam';
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
  } else if (effectiveMode === 'slam') {
    updateModelBadge('Loaded: 👁️ Visual SLAM 3D');
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
    // sensors or slam
    if (callback) callback();
  }
  refreshModelBadge();
}

function updateModelVisibilityAndCamera(updateCamera = true) {
  const effectiveMode = getEffectiveModelMode();

  if (slamMeshes.container) {
    slamMeshes.container.visible = (effectiveMode === 'slam');
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
  } else if (effectiveMode === 'slam') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (quardMeshes.container) quardMeshes.container.visible = false;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;
    if (slamMeshes.container) slamMeshes.container.visible = true;
    if (updateCamera && camera && controls) {
      camera.position.set(0, 240, 320);
      controls.target.set(0, 50, 60);
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

function animate() {
  requestAnimationFrame(animate);
  controls.update();

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
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// ── Left Control Panel GUI Generator ─────────────────────────────────────
function buildJointControls() {
  const container = document.getElementById('joints-list');
  container.innerHTML = '';

  JOINTS.forEach((j) => {
    jointState[j.id] = j.def;

    const card = document.createElement('div');
    card.className = 'joint-card';
    card.id = `card-${j.id}`;

    const radVal = degToRad(j.def, j.rad_min, j.rad_max);

    const syncOffset = getJointSyncOffset(j.id);
    const syncText = `${syncOffset >= 0 ? '+' : ''}${syncOffset.toFixed(1)}°`;

    card.innerHTML = `
      <div class="joint-card-header">
        <span class="joint-name">🦾 ${j.name}</span>
        <span class="joint-readout" id="readout-${j.id}">${j.def.toFixed(1)}° (${radVal.toFixed(3)} rad)</span>
      </div>
      <div class="slider-row">
        <input type="range" id="slider-${j.id}" min="0" max="1800" value="${j.def * 10}">
      </div>
      <div class="limits-row">
        <label>Min:</label>
        <input type="number" id="spin-min-${j.id}" class="spinbox spinbox-min" value="${j.min.toFixed(1)}" step="1.0" min="0" max="180">
        <button class="btn btn-outline btn-xs btn-set-low" data-joint="${j.id}">📍 Set Low</button>
        <div style="flex:1"></div>
        <label>Max:</label>
        <input type="number" id="spin-max-${j.id}" class="spinbox spinbox-max" value="${j.max.toFixed(1)}" step="1.0" min="0" max="180">
        <button class="btn btn-outline btn-xs btn-set-high" data-joint="${j.id}">📍 Set High</button>
      </div>
      <div class="sync-row" style="display:flex;align-items:center;gap:6px;margin-top:6px;padding-top:6px;border-top:1px dashed #cbd5e1;">
        <span style="font-weight:600;color:#475569;font-size:11px;" title="Fine-tune 3D model orientation offset relative to physical servo">🎯 Sync:</span>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="-5">-5°</button>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="-1">-1°</button>
        <span id="sync-val-${j.id}" class="readout-pill" style="font-size:11px;padding:2px 6px;min-width:44px;text-align:center;">${syncText}</span>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="1">+1°</button>
        <button class="btn btn-outline btn-xs btn-nudge" data-joint="${j.id}" data-nudge="5">+5°</button>
        <button class="btn btn-outline btn-xs btn-zero-sync" data-joint="${j.id}" title="Reset sync offset to 0°">0°</button>
      </div>
    `;

    container.appendChild(card);

    // Event listener for Slider changes
    const slider = card.querySelector(`#slider-${j.id}`);
    slider.addEventListener('input', (e) => {
      const val = parseFloat(e.target.value) / 10.0;
      onSliderChanged(j.id, val);
    });

    // Set Low / Set High click handlers
    card.querySelector('.btn-set-low').addEventListener('click', () => {
      const curr = jointState[j.id];
      card.querySelector(`#spin-min-${j.id}`).value = curr.toFixed(1);
    });

    card.querySelector('.btn-set-high').addEventListener('click', () => {
      const curr = jointState[j.id];
      card.querySelector(`#spin-max-${j.id}`).value = curr.toFixed(1);
    });

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

function onSliderChanged(jointId, deg) {
  jointState[jointId] = deg;

  const readout = document.getElementById(`readout-${jointId}`);
  const jointDef = JOINTS.find(j => j.id === jointId);
  const rad = degToRad(deg, jointDef ? jointDef.rad_min : -2.0, jointDef ? jointDef.rad_max : 2.0);

  if (readout) {
    readout.innerText = `${deg.toFixed(1)}° (${rad.toFixed(3)} rad)`;
  }

  // Push updated joint positions to server
  fetch('/api/update_joints', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ joints: jointState })
  });
}

function degToRad(deg, rMin, rMax) {
  const ratio = deg / 180.0;
  return rMin + ratio * (rMax - rMin);
}

// ── Setup Action Bar Event Listeners ─────────────────────────────────────
function setupEventListeners() {
  // Start Sim Toggle
  const btnSim = document.getElementById('btn-sim');
  btnSim.addEventListener('click', () => {
    isSimulating = !isSimulating;
    btnSim.innerText = isSimulating ? '⏸️ Stop Sim' : '▶️ Start Sim';
    btnSim.className = isSimulating ? 'btn btn-danger btn-sm' : 'btn btn-warning btn-sm';

    // Synchronize rendered angles so transition has zero initial displacement
    for (let k in jointState) {
      renderedJointState[k] = jointState[k];
    }

    fetch('/api/toggle_sim', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        simulating: isSimulating,
        current_joints: jointState
      })
    });
  });

  // Sim Speed Slider
  const sliderSpeed = document.getElementById('slider-speed');
  sliderSpeed.addEventListener('input', (e) => {
    const mult = (parseFloat(e.target.value) / 10.0).toFixed(1);
    document.getElementById('lbl-speed-val').innerText = `${mult}x`;

    fetch('/api/toggle_sim', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ speed: mult })
    });
  });

  // Go Home
  document.getElementById('btn-home').addEventListener('click', () => {
    JOINTS.forEach(j => {
      onSliderChanged(j.id, j.def);
      const slider = document.getElementById(`slider-${j.id}`);
      if (slider) slider.value = j.def * 10;
    });
  });

  // Center All
  document.getElementById('btn-center').addEventListener('click', () => {
    JOINTS.forEach(j => {
      onSliderChanged(j.id, 90.0);
      const slider = document.getElementById(`slider-${j.id}`);
      if (slider) slider.value = 900;
    });
  });

  // Save Config
  document.getElementById('btn-save').addEventListener('click', () => {
    const calib = {};
    JOINTS.forEach(j => {
      const minVal = parseFloat(document.getElementById(`spin-min-${j.id}`).value);
      const maxVal = parseFloat(document.getElementById(`spin-max-${j.id}`).value);
      calib[j.id] = {
        servo_min_deg: minVal,
        servo_max_deg: maxVal,
        home_deg: jointState[j.id]
      };
    });

    fetch('/api/save_calibration', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(calib)
    }).then(res => res.json()).then(data => {
      alert('💾 Homing & Safety Limits saved successfully to servo_calibration.json!');
    });
  });

  // 🎯 Sync Virtual to Physical Robot Arm
  const btnSyncArm = document.getElementById('btn-sync-all-arm');
  if (btnSyncArm) {
    btnSyncArm.addEventListener('click', () => {
      fetch('/api/tare_pose', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ target: 'arm' })
      })
      .then(res => res.json())
      .then(data => {
        if (data.status === 'ok') {
          syncCalibration = data.calibration || syncCalibration;
          updateSyncReadouts();
          alert('🎯 Virtual Twin synchronized with physical robot orientation!');
        }
      })
      .catch(() => alert('⚠️ Sync request failed! Check server connection.'));
    });
  }

  // 🎯 Tare IMU Horizon
  const btnTareImu = document.getElementById('btn-tare-imu');
  if (btnTareImu) {
    btnTareImu.addEventListener('click', () => {
      fetch('/api/tare_pose', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ target: 'imu' })
      })
      .then(res => res.json())
      .then(data => {
        if (data.status === 'ok') {
          syncCalibration = data.calibration || syncCalibration;
          const elP = document.getElementById('val-pitch');
          const elR = document.getElementById('val-roll');
          if (elP) elP.innerText = '0.0°';
          if (elR) elR.innerText = '0.0°';
          alert('🎯 IMU Horizon Tared! Virtual device is now leveled at 0.0° pitch & roll.');
        }
      })
      .catch(() => alert('⚠️ IMU Tare request failed!'));
    });
  }

  // 🎯 Quick Sync Button in 3D Viewport Header
  const btnQuickSync = document.getElementById('btn-quick-sync-vp');
  if (btnQuickSync) {
    btnQuickSync.addEventListener('click', () => {
      const mode = getEffectiveModelMode();
      const target = (mode === 'sensors') ? 'imu' : ((mode === 'quard') ? 'quard' : 'arm');
      fetch('/api/tare_pose', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ target: target })
      })
      .then(res => res.json())
      .then(data => {
        if (data.status === 'ok') {
          syncCalibration = data.calibration || syncCalibration;
          updateSyncReadouts();
          alert(`🎯 Digital Twin (${target.toUpperCase()}) synchronized with physical orientation!`);
        }
      })
      .catch(() => alert('⚠️ Quick sync request failed!'));
    });
  }

  // Comm Mode Toggle Switch Listener
  const toggleCommMode = document.getElementById('toggle-comm-mode');
  const lblWifi = document.getElementById('lbl-wifi');
  const lblSerial = document.getElementById('lbl-serial');

  if (toggleCommMode) {
    toggleCommMode.addEventListener('change', (e) => {
      const isWifi = e.target.checked;
      const mode = isWifi ? 'WIFI' : 'SERIAL';

      lblWifi.className = isWifi ? 'toggle-label active' : 'toggle-label';
      lblSerial.className = isWifi ? 'toggle-label' : 'toggle-label active';

      fetch('/api/comm_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: mode })
      }).then(res => res.json()).then(data => {
        console.log(`[Dashboard] Comm mode set to ${mode}:`, data);
      });
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
function startStatePolling() {
  setInterval(() => {
    fetch('/api/state')
      .then(res => res.json())
      .then(data => {
        if (data.joints) {
          for (let k in data.joints) {
            jointState[k] = data.joints[k];
            // Update UI sliders if simulating
            if (isSimulating) {
              const slider = document.getElementById(`slider-${k}`);
              const readout = document.getElementById(`readout-${k}`);
              if (slider) slider.value = data.joints[k] * 10;
              if (readout) readout.innerText = `${data.joints[k].toFixed(1)}°`;
            }
          }
        }
      });
  }, 40); // 25 Hz state sync
}

// ── Tab Management System ────────────────────────────────────────────────
let activeTab = 'arm';

function setupTabs() {
  const btnArm = document.getElementById('tab-btn-arm');
  const btnQuard = document.getElementById('tab-btn-quard');
  const btnSensors = document.getElementById('tab-btn-sensors');
  const btnSlam = document.getElementById('tab-btn-slam');

  const panelArm = document.getElementById('panel-arm-control');
  const panelQuard = document.getElementById('panel-quard-control');
  const panelSensors = document.getElementById('panel-sensors-control');
  const panelSlam = document.getElementById('panel-slam-control');

  function setActive(tabName, btn, panel) {
    activeTab = tabName;
    [btnArm, btnQuard, btnSensors, btnSlam].forEach(b => b && b.classList.remove('active'));
    [panelArm, panelQuard, panelSensors, panelSlam].forEach(p => p && p.classList.remove('active'));

    if (btn) btn.classList.add('active');
    if (panel) panel.classList.add('active');
  }

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
  if (btnSlam) {
    btnSlam.addEventListener('click', () => {
      setActive('slam', btnSlam, panelSlam);
      if (modelLoadSystem.mode === 'auto') {
        setModelMode('auto');
      }
    });
  }
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
        <span class="quard-channel-name">
          <span style="display:inline-block;width:10px;height:10px;border-radius:50%;background-color:${badgeColor};margin-right:6px;vertical-align:middle;box-shadow:0 0 4px ${badgeColor};"></span>
          <strong>Ch ${c.ch}:</strong> ${c.name}
        </span>
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
  quardState.channels[ch] = angle;
  const jInfo = CHANNEL_TO_JOINT[ch];
  if (jInfo) {
    quardState.joints[jInfo.id] = channelAngleToRad(ch, angle);
  }
  updateQuardKinematics();
  syncQuardJointsToServer();
  sendQuardCmd({ ch: ch, angle: angle });
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
}

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
    if (activeTab !== 'quard') return;
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

function sendQuardCmd(params) {
  const host = document.getElementById('input-quard-host') ? document.getElementById('input-quard-host').value.trim() : 'http://sesame-robot.local';

  fetch('/api/quardbot/cmd', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ host: host, params: params })
  })
  .then(res => res.json())
  .then(data => {
    updateQuardStatusPill(data.status === 'ok');
  })
  .catch(err => {
    console.warn('[Dashboard] Quard Bot command error:', err);
    updateQuardStatusPill(false);
  });
}

function pingQuardBot() {
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
  .catch(() => updateQuardStatusPill(false));
}

function updateQuardStatusPill(online) {
  const pill = document.getElementById('pill-quard-status');
  const dot = document.getElementById('dot-quardbot');
  const text = document.getElementById('status-quardbot-text');

  if (pill) {
    pill.className = online ? 'status-pill status-online' : 'status-pill status-offline';
    pill.innerText = online ? 'Connected' : 'Offline';
  }
  if (dot && text) {
    dot.parentElement.className = online ? 'node-badge status-online' : 'node-badge status-offline';
    text.innerText = online ? 'Connected' : 'Disconnected';
  }
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
      btnWifiConnect.innerText = '⏳ Connecting...';
      btnWifiConnect.disabled = true;

      fetch('/api/sensors/wifi_connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ip: ip })
      })
      .then(res => res.json())
      .then(data => {
        btnWifiConnect.innerText = '🌐 Connect Wi-Fi';
        btnWifiConnect.disabled = false;
        if (btnWifiDisconnect) btnWifiDisconnect.style.display = 'inline-block';
        const pill = document.getElementById('pill-sensor-wifi-status');
        if (pill) {
          pill.innerText = `Connected (${ip})`;
          pill.className = 'status-pill status-online';
        }
      })
      .catch(err => {
        btnWifiConnect.innerText = '🌐 Connect Wi-Fi';
        btnWifiConnect.disabled = false;
      });
    });
  }

  if (btnWifiDisconnect) {
    btnWifiDisconnect.addEventListener('click', () => {
      fetch('/api/sensors/wifi_disconnect', { method: 'POST' })
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
      .then(data => {
        alert(`🔌 Connecting to ESP32 on ${port} @ ${baud} baud...`);
      });
    });
  }

  if (btnSim) {
    btnSim.addEventListener('click', () => {
      sensorState.simulating = !sensorState.simulating;
      btnSim.innerText = sensorState.simulating ? '▶️ Simulating Data' : '⏸️ Stop Simulation';
      btnSim.className = sensorState.simulating ? 'btn btn-warning btn-sm' : 'btn btn-secondary btn-sm';

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
        if (fireBanner) {
          fireBanner.style.display = isFire ? 'block' : 'none';
        }

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

        if (badgeDht) {
          if (data.dht_error) {
            badgeDht.className = 'status-pill status-offline';
            badgeDht.innerText = 'DHT ERROR';
          } else {
            badgeDht.className = 'status-pill status-online';
            badgeDht.innerText = 'OK';
          }
        }

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
        if (badgeGas) {
          if (gasVal > 2500) {
            badgeGas.className = 'status-pill status-offline';
            badgeGas.innerText = '🔴 HAZARD!';
          } else if (gasVal > 1500) {
            badgeGas.className = 'status-pill';
            badgeGas.style.backgroundColor = '#78350f';
            badgeGas.style.color = '#fde047';
            badgeGas.innerText = '🟡 Moderate';
          } else {
            badgeGas.className = 'status-pill status-online';
            badgeGas.innerText = '🟢 Clean Air';
          }
        }

        if (elWater) elWater.innerText = `${waterVal} / 4095`;
        if (meterWater) {
          const wPct = Math.min(100, Math.max(0, (waterVal / 4095) * 100));
          meterWater.style.width = `${wPct.toFixed(1)}%`;
        }
        if (badgeWater) {
          if (waterVal > 2000) {
            badgeWater.className = 'status-pill';
            badgeWater.style.backgroundColor = '#1e3a8a';
            badgeWater.style.color = '#93c5fd';
            badgeWater.innerText = '🌊 Submerged';
          } else if (waterVal > 600) {
            badgeWater.className = 'status-pill';
            badgeWater.style.backgroundColor = '#0c4a6e';
            badgeWater.style.color = '#38bdf8';
            badgeWater.innerText = '💧 Moisture';
          } else {
            badgeWater.className = 'status-pill status-online';
            badgeWater.innerText = '🟢 Dry';
          }
        }

        if (elFlame) {
          elFlame.innerText = (flameVal === 0) ? 'Pin DO: LOW (🔥 Fire Detected!)' : 'Pin DO: HIGH (Clear)';
          elFlame.style.color = (flameVal === 0) ? '#ef4444' : '#10b981';
        }
        if (badgeFlame) {
          if (flameVal === 0) {
            badgeFlame.className = 'status-pill status-offline';
            badgeFlame.innerText = '🚨 FIRE ALERT!';
          } else {
            badgeFlame.className = 'status-pill status-online';
            badgeFlame.innerText = '✅ Clear';
          }
        }
        if (valAlert) {
          valAlert.innerText = `Alert: ${alertStr}`;
          valAlert.className = (alertStr === 'FIRE') ? 'status-pill status-offline' : 'status-pill status-online';
        }

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
          elGpsSats.innerText = `🛰️ ${sats} satellites locked`;
        }
        if (badgeGps) {
          badgeGps.className = gpsValid ? 'status-pill status-online' : 'status-pill status-offline';
          badgeGps.innerText = gpsValid ? '3D Fix' : 'No Fix';
        }

        const sdOk = data.sd_ok !== undefined ? data.sd_ok : true;
        if (elSdStatus) {
          elSdStatus.innerText = sdOk ? '💾 SD: Ready (/log.csv)' : '❌ No SD Card';
          elSdStatus.style.color = sdOk ? '#10b981' : '#ef4444';
        }
        if (elUptime) {
          const uptimeSec = Math.floor((data.uptime_ms || 0) / 1000);
          const mins = Math.floor(uptimeSec / 60);
          const secs = uptimeSec % 60;
          elUptime.innerText = `Uptime: ${mins}m ${secs}s`;
        }

        // Update Wi-Fi status pill
        const pillWifi = document.getElementById('pill-sensor-wifi-status');
        if (pillWifi) {
          if (data.wifi_connected) {
            pillWifi.className = 'status-pill status-online';
            pillWifi.innerText = `Wi-Fi: Online (${data.wifi_ip || ''})`;
          } else if (!data.connected && !data.simulating) {
            pillWifi.className = 'status-pill status-offline';
            pillWifi.innerText = 'Wi-Fi: Offline';
          }
        }

        // Update Console Log
        const elConsole = document.getElementById('sensor-console-log');
        if (elConsole && data.raw_log && data.raw_log.length > 0) {
          elConsole.innerText = data.raw_log.slice(-30).join('\n');
          elConsole.scrollTop = elConsole.scrollHeight;
        }

        // Update Top Header Badge
        const textNode = document.getElementById('status-sensornode-text');
        const dotNode = document.getElementById('dot-sensornode');
        if (textNode && dotNode) {
          if (data.wifi_connected) {
            textNode.innerText = 'Wi-Fi Online';
            dotNode.parentElement.className = 'node-badge status-online';
          } else if (data.connected) {
            textNode.innerText = 'Hardware Serial';
            dotNode.parentElement.className = 'node-badge status-online';
          } else if (data.simulating) {
            textNode.innerText = 'Simulating';
            dotNode.parentElement.className = 'node-badge status-online';
          } else {
            textNode.innerText = 'Disconnected';
            dotNode.parentElement.className = 'node-badge status-offline';
          }
        }
      })
      .catch(err => {});
  }, 200); // 5 Hz telemetry sync
}

// ── Visual SLAM Controls & Real-Time Sync System ──────────────────────────
const SLAM_SCALE = 250.0; // 1 meter = 250 units in Three.js viewport
let slamClientFrames = 0;
let slamClientTimer = Date.now();
let slamWebFps = 0.0;

function initSLAMControls() {
  // 1. IP Input & Stream Connect/Disconnect controls
  const inputIp = document.getElementById('input-slam-ip');
  const btnConnect = document.getElementById('btn-slam-connect');
  const btnDisconnect = document.getElementById('btn-slam-disconnect');

  if (btnConnect) {
    btnConnect.addEventListener('click', () => {
      const target = (inputIp ? inputIp.value.trim() : '') || '10.88.106.30';
      btnConnect.innerText = '⏳ Connecting...';
      btnConnect.disabled = true;
      fetch('/api/slam/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ target: target })
      })
        .then(res => res.json())
        .then(data => {
          btnConnect.innerText = '🔗 Connect';
          btnConnect.disabled = false;
          if (data.status === 'ok') {
            const pillStatus = document.getElementById('pill-slam-status');
            if (pillStatus) {
              pillStatus.innerText = 'Connecting (' + target + ')';
              pillStatus.className = 'status-pill status-online';
            }
            if (btnDisconnect) btnDisconnect.style.display = 'inline-block';
          }
        })
        .catch(err => {
          btnConnect.innerText = '🔗 Connect';
          btnConnect.disabled = false;
        });
    });
  }

  // Quick preset buttons (Wi-Fi, USB Serial, Auto-Detect)
  const btnQuickWifi = document.getElementById('btn-slam-quick-wifi');
  if (btnQuickWifi) {
    btnQuickWifi.addEventListener('click', () => {
      if (inputIp) inputIp.value = '10.88.106.30';
      if (btnConnect) btnConnect.click();
    });
  }

  const btnQuickSerial = document.getElementById('btn-slam-quick-serial');
  if (btnQuickSerial) {
    btnQuickSerial.addEventListener('click', () => {
      if (inputIp) inputIp.value = '/dev/ttyUSB0';
      if (btnConnect) btnConnect.click();
    });
  }

  const btnQuickAuto = document.getElementById('btn-slam-quick-auto');
  if (btnQuickAuto) {
    btnQuickAuto.addEventListener('click', () => {
      if (inputIp) inputIp.value = 'auto';
      if (btnConnect) btnConnect.click();
    });
  }

  if (inputIp) {
    inputIp.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') {
        if (btnConnect) btnConnect.click();
      }
    });
  }

  if (btnDisconnect) {
    btnDisconnect.addEventListener('click', () => {
      fetch('/api/slam/disconnect', { method: 'POST' })
        .then(res => res.json())
        .then(() => {
          btnDisconnect.style.display = 'none';
          const pillStatus = document.getElementById('pill-slam-status');
          if (pillStatus) {
            pillStatus.innerText = 'Disconnected';
            pillStatus.className = 'status-pill status-offline';
          }
        })
        .catch(() => {});
    });
  }

  // 2. Focus 3D SLAM Viewport button
  const btnLoadSlam = document.getElementById('btn-load-slam-view');
  if (btnLoadSlam) {
    btnLoadSlam.addEventListener('click', () => {
      setModelMode('slam');
    });
  }

  // 2. Reset Map button
  const btnReset = document.getElementById('btn-slam-reset');
  if (btnReset) {
    btnReset.addEventListener('click', () => {
      if (confirm('Reset 3D SLAM map, trajectory, and camera pose?')) {
        fetch('/api/slam/reset', { method: 'POST' })
          .then(res => res.json())
          .then(() => {
            if (slamMeshes.pointCloud) slamMeshes.pointCloud.geometry.setDrawRange(0, 0);
            if (slamMeshes.trajectoryLine) slamMeshes.trajectoryLine.geometry.setDrawRange(0, 0);
          })
          .catch(() => {});
      }
    });
  }

  // 4. Engine Toggle button
  const btnToggle = document.getElementById('btn-slam-toggle');
  if (btnToggle) {
    btnToggle.addEventListener('click', () => {
      fetch('/api/slam/toggle', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
          if (data.status === 'ok') {
            slamState.active = data.active;
            btnToggle.innerText = data.active ? '⏹️ Stop Engine' : '▶️ Start Engine';
          }
        })
        .catch(() => {});
    });
  }

  // 5. Video Feeds View Switcher
  const btnViewCam = document.getElementById('btn-slam-view-cam');
  const btnViewMap = document.getElementById('btn-slam-view-map');
  const btnViewDual = document.getElementById('btn-slam-view-dual');
  const viewContainer = document.getElementById('slam-viewports-container');
  const camViewport = document.getElementById('slam-cam-viewport');
  const mapViewport = document.getElementById('slam-map-viewport');

  function setSLAMView(mode) {
    slamState.viewMode = mode;
    [btnViewCam, btnViewMap, btnViewDual].forEach(b => b && b.classList.remove('active'));

    if (mode === 'dual') {
      if (btnViewDual) btnViewDual.classList.add('active');
      if (viewContainer) viewContainer.className = 'slam-viewports-dual';
      if (camViewport) camViewport.style.display = 'flex';
      if (mapViewport) mapViewport.style.display = 'flex';
    } else if (mode === 'map') {
      if (btnViewMap) btnViewMap.classList.add('active');
      if (viewContainer) viewContainer.className = 'slam-viewports-single';
      if (camViewport) camViewport.style.display = 'none';
      if (mapViewport) mapViewport.style.display = 'flex';
    } else {
      // cam
      if (btnViewCam) btnViewCam.classList.add('active');
      if (viewContainer) viewContainer.className = 'slam-viewports-single';
      if (camViewport) camViewport.style.display = 'flex';
      if (mapViewport) mapViewport.style.display = 'none';
    }
  }

  if (btnViewCam) btnViewCam.addEventListener('click', () => setSLAMView('cam'));
  if (btnViewMap) btnViewMap.addEventListener('click', () => setSLAMView('map'));
  if (btnViewDual) btnViewDual.addEventListener('click', () => setSLAMView('dual'));
}

function startSLAMPolling() {
  const imgCam = document.getElementById('img-slam-cam');
  const imgMap = document.getElementById('img-slam-map');

  // Fast frame updater for live video & 2D map feeds (12 Hz)
  setInterval(() => {
    if (activeTab !== 'slam') return;
    const now = Date.now();

    slamClientFrames++;
    if (now - slamClientTimer >= 1000) {
      slamWebFps = (slamClientFrames * 1000.0) / (now - slamClientTimer);
      slamClientFrames = 0;
      slamClientTimer = now;
      const elWebFps = document.getElementById('val-slam-web-fps');
      if (elWebFps) elWebFps.innerText = slamWebFps.toFixed(1);
    }

    if (slamState.viewMode === 'cam' || slamState.viewMode === 'dual') {
      if (imgCam) imgCam.src = '/api/slam/frame?t=' + now;
    }
    if (slamState.viewMode === 'map' || slamState.viewMode === 'dual') {
      if (imgMap) imgMap.src = '/api/slam/map_2d?t=' + now;
    }
  }, 80);

  // Telemetry Polling (10 Hz)
  setInterval(() => {
    fetch('/api/slam/state')
      .then(res => res.json())
      .then(data => {
        updateSLAMState(data);
      })
      .catch(() => {});
  }, 100);

  // 3D Point Cloud & Trajectory Polling (8 Hz)
  setInterval(() => {
    const effectiveMode = getEffectiveModelMode();
    if (effectiveMode !== 'slam' && activeTab !== 'slam') return;

    fetch('/api/slam/map')
      .then(res => res.json())
      .then(data => {
        updateSLAM3DMap(data);
      })
      .catch(() => {});
  }, 125);
}

function updateSLAMState(data) {
  if (!data) return;

  // 0. Disconnect button visibility & input target sync
  const btnDisconnect = document.getElementById('btn-slam-disconnect');
  if (btnDisconnect) {
    btnDisconnect.style.display = (data.connected || (data.status && data.status.startsWith('Connecting'))) ? 'inline-block' : 'none';
  }
  const inputIp = document.getElementById('input-slam-ip');
  if (inputIp && !inputIp.matches(':focus') && data.target && !inputIp.value) {
    inputIp.value = data.target;
  }
  const elIngestIp = document.getElementById('val-slam-ingest-ip');
  if (elIngestIp && data.local_ip) {
    elIngestIp.innerText = `${data.local_ip}:${data.udp_port || 5000} (UDP)`;
  }

  // 1. Engine Button state
  const btnToggle = document.getElementById('btn-slam-toggle');
  if (btnToggle) {
    btnToggle.innerText = data.active ? '⏹️ Stop Engine' : '▶️ Start Engine';
  }

  // 2. Status Pills
  const pillStatus = document.getElementById('pill-slam-status');
  if (pillStatus) {
    pillStatus.innerText = data.status || 'Standby';
    if (data.is_error || (data.status && data.status.includes('⚠️'))) {
      pillStatus.className = 'status-pill status-offline';
    } else if (data.status === 'Tracking' || data.connected) {
      pillStatus.className = 'status-pill status-online';
    } else if (data.active) {
      pillStatus.className = 'status-pill status-online';
    } else {
      pillStatus.className = 'status-pill status-offline';
    }
  }

  const elEngineState = document.getElementById('val-slam-engine-state');
  if (elEngineState) {
    elEngineState.innerText = data.active ? 'Active' : 'Halted';
  }

  // 3. FPS Metrics
  const elUdpFps = document.getElementById('val-slam-udp-fps');
  const elVoFps = document.getElementById('val-slam-vo-fps');
  if (elUdpFps) elUdpFps.innerText = (data.udp_fps || 0.0).toFixed(1);
  if (elVoFps) elVoFps.innerText = (data.vo_fps || 0.0).toFixed(1);

  // 4. VL53L1X ToF Distance
  const elDist = document.getElementById('val-slam-dist');
  const badgeTof = document.getElementById('badge-slam-tof');
  const meterDist = document.getElementById('meter-slam-dist');
  if (elDist) {
    const mm = data.distance_mm || 0;
    const m = (mm / 1000.0).toFixed(2);
    elDist.innerText = `${m} m (${mm} mm)`;
  }
  if (badgeTof) {
    badgeTof.innerText = data.is_tof_valid ? 'VALID' : 'OUT OF RANGE';
    badgeTof.className = data.is_tof_valid ? 'status-pill status-online' : 'status-pill status-offline';
  }
  if (meterDist) {
    const pct = Math.min(100, Math.max(0, ((data.distance_mm || 0) / 4000.0) * 100));
    meterDist.style.width = `${pct}%`;
  }

  // 5. Visual Odometry Metrics
  const elFeatures = document.getElementById('val-slam-features');
  const elMatches = document.getElementById('val-slam-matches');
  const elScale = document.getElementById('val-slam-scale');
  const elPoints = document.getElementById('val-slam-points');
  const badgeTracking = document.getElementById('badge-slam-tracking');

  if (elFeatures) elFeatures.innerText = data.num_features || 0;
  if (elMatches) elMatches.innerText = data.num_matches || 0;
  if (elScale) elScale.innerText = `${(data.metric_scale || 0.05).toFixed(3)} m`;
  if (elPoints) elPoints.innerText = data.point_count || 0;

  if (badgeTracking) {
    badgeTracking.innerText = data.status || 'Standby';
    badgeTracking.className = (data.status === 'Tracking')
      ? 'status-pill status-online'
      : (data.status === 'Tracking Lost' ? 'status-pill status-offline' : 'status-pill');
  }

  // 6. 6-DOF Pose & Heading
  const elPosX = document.getElementById('val-slam-pos-x');
  const elPosY = document.getElementById('val-slam-pos-y');
  const elPosZ = document.getElementById('val-slam-pos-z');
  const elHeading = document.getElementById('val-slam-heading');

  if (data.curr_pos && data.curr_pos.length >= 3) {
    if (elPosX) elPosX.innerText = data.curr_pos[0].toFixed(3);
    if (elPosY) elPosY.innerText = data.curr_pos[1].toFixed(3);
    if (elPosZ) elPosZ.innerText = data.curr_pos[2].toFixed(3);
  }
  if (elHeading) elHeading.innerText = `${(data.heading_deg || 0.0).toFixed(1)}°`;

  // 7. Header IoT Badge
  const statusSlamText = document.getElementById('status-slamnode-text');
  const dotSlam = document.getElementById('dot-slamnode');
  if (statusSlamText && dotSlam) {
    if (data.status === 'Tracking') {
      statusSlamText.innerText = 'Tracking';
      dotSlam.parentElement.className = 'node-badge status-online';
    } else if (data.active) {
      statusSlamText.innerText = 'Listening (Port 5000)';
      dotSlam.parentElement.className = 'node-badge status-online';
    } else {
      statusSlamText.innerText = 'Standby';
      dotSlam.parentElement.className = 'node-badge status-offline';
    }
  }
}

function updateSLAM3DMap(data) {
  if (!data || !slamMeshes.pointCloud) return;

  // 1. Update Sparse 3D Point Cloud
  const pts = data.points || [];
  const cols = data.colors || [];
  const ptGeom = slamMeshes.pointCloud.geometry;
  const posAttr = ptGeom.getAttribute('position');
  const colAttr = ptGeom.getAttribute('color');
  const maxPts = posAttr.count;

  const count = Math.min(pts.length, maxPts);
  for (let i = 0; i < count; i++) {
    const px = pts[i][0] * SLAM_SCALE;
    const py = -pts[i][1] * SLAM_SCALE + 60.0;
    const pz = pts[i][2] * SLAM_SCALE;

    posAttr.setXYZ(i, px, py, pz);

    if (cols[i]) {
      colAttr.setXYZ(i, cols[i][0], cols[i][1], cols[i][2]);
    } else {
      colAttr.setXYZ(i, 0.0, 0.85, 1.0);
    }
  }
  posAttr.needsUpdate = true;
  colAttr.needsUpdate = true;
  ptGeom.setDrawRange(0, count);

  // 2. Update Camera Trajectory
  const traj = data.trajectory || [];
  const trajGeom = slamMeshes.trajectoryLine.geometry;
  const trajAttr = trajGeom.getAttribute('position');
  const maxTraj = trajAttr.count;
  const trajCount = Math.min(traj.length, maxTraj);

  for (let i = 0; i < trajCount; i++) {
    const tx = traj[i][0] * SLAM_SCALE;
    const ty = -traj[i][1] * SLAM_SCALE + 60.0;
    const tz = traj[i][2] * SLAM_SCALE;
    trajAttr.setXYZ(i, tx, ty, tz);
  }
  trajAttr.needsUpdate = true;
  trajGeom.setDrawRange(0, trajCount);

  // 3. Update Camera Frustum & Heading
  if (data.pose && slamMeshes.cameraFrustum) {
    const P = data.pose;
    const camX = P[0][3] * SLAM_SCALE;
    const camY = -P[1][3] * SLAM_SCALE + 60.0;
    const camZ = P[2][3] * SLAM_SCALE;

    slamMeshes.cameraFrustum.position.set(camX, camY, camZ);

    const fwdX = P[0][2];
    const fwdY = -P[1][2];
    const fwdZ = P[2][2];

    const targetPos = new THREE.Vector3(camX + fwdX * 50.0, camY + fwdY * 50.0, camZ + fwdZ * 50.0);
    slamMeshes.cameraFrustum.lookAt(targetPos);

    // 4. Update ToF Laser Ray Line
    if (slamMeshes.tofRayLine && data.tof_ray) {
      const tofGeom = slamMeshes.tofRayLine.geometry;
      const tofAttr = tofGeom.getAttribute('position');
      const rayLen = (data.tof_ray.length || 0.0) * SLAM_SCALE;
      const isValid = data.tof_ray.valid;

      tofAttr.setXYZ(0, camX, camY, camZ);
      tofAttr.setXYZ(1, camX + fwdX * rayLen, camY + fwdY * rayLen, camZ + fwdZ * rayLen);
      tofAttr.needsUpdate = true;
      tofGeom.setDrawRange(0, 2);

      slamMeshes.tofRayLine.material.color.setHex(isValid ? 0x10b981 : 0xef4444);
    }
  }
}
