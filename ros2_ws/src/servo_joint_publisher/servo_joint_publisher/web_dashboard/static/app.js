/**
 * Light-Themed WebGL Robot Arm Dashboard
 * ---------------------------------------
 * Powered by Three.js (WebGL 3D Engine) & REST/WebSocket Sync
 */

// Joint definitions matching unnamed_gazebo.urdf
const JOINTS = [
  { id: 'turntable_link_joint_dup', name: 'Turntable Yaw', min: 0.0, max: 180.0, def: 90.0, rad_min: -3.0, rad_max: 3.0 },
  { id: 'turntable_link_joint', name: 'Shoulder Pitch', min: 16.6, max: 91.4, def: 59.0, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_1', name: 'Elbow Pitch 1', min: 30.4, max: 153.1, def: 153.1, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_2', name: 'Elbow Pitch 2', min: 0.0, max: 180.0, def: 116.0, rad_min: -2.0, rad_max: 2.0 },
  { id: 'turntable_link_joint_dup_3', name: 'Wrist Pitch', min: 0.0, max: 180.0, def: 90.0, rad_min: -2.0, rad_max: 2.0 },
  { id: 'wrist_twist_joint', name: 'Wrist Twist', min: 0.0, max: 180.0, def: 90.0, rad_min: -3.14159, rad_max: 3.14159 },
  { id: 'turntable_link_joint_dup_4', name: 'Gripper Linkage', min: 0.0, max: 180.0, def: 90.0, rad_min: -1.0, rad_max: 1.0 }
];

let scene, camera, renderer, controls;
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

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  initThreeJS();
  buildJointControls();
  setupTabs();
  initQuardBotControls();
  initSensorControls();
  setupEventListeners();
  startStatePolling();
  startSensorPolling();
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

  // Load Robot Model Meshes
  loadRobotModel();

  // Window Resize Listener
  window.addEventListener('resize', onWindowResize);

  // Animation Render Loop
  animate();
}

function loadRobotModel() {
  const loader = new THREE.STLLoader();

  // Create kinematic joint groups
  robotMeshes.base = new THREE.Group();
  robotMeshes.turntable = new THREE.Group();
  robotMeshes.shoulder = new THREE.Group();
  robotMeshes.forearm = new THREE.Group();
  robotMeshes.wrist = new THREE.Group();
  robotMeshes.gripper = new THREE.Group();

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

  // 1. Base Mesh
  loader.load('/meshes/part_9_obj9.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matBase);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    robotMeshes.base.add(mesh);
  });

  // 2. Turntable Mesh
  loader.load('/meshes/part_7_obj7.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matArm);
    mesh.castShadow = true;
    mesh.position.set(0, 45, 0);
    robotMeshes.turntable.add(mesh);
  });

  // 3. Shoulder Link Mesh
  loader.load('/meshes/part_44_obj44.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matLink);
    mesh.castShadow = true;
    mesh.position.set(0, 95, 0);
    robotMeshes.shoulder.add(mesh);
  });

  // 4. Forearm Link Mesh
  loader.load('/meshes/part_3_obj3.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matArm);
    mesh.castShadow = true;
    mesh.position.set(0, 160, 0);
    robotMeshes.forearm.add(mesh);
  });

  // 5. Wrist Mesh
  loader.load('/meshes/Robot_Arm_Wrist.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matLink);
    mesh.castShadow = true;
    mesh.position.set(0, 220, 0);
    robotMeshes.wrist.add(mesh);
  });

  // 6. Gripper Base Mesh
  loader.load('/meshes/mg996r-v17.005.stl', (geom) => {
    geom.computeVertexNormals();
    const mesh = new THREE.Mesh(geom, matGripper);
    mesh.castShadow = true;
    mesh.position.set(0, 260, 0);
    robotMeshes.gripper.add(mesh);
  });

  // 7. MPU6050 IMU Sensor Module Visual
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

function animate() {
  requestAnimationFrame(animate);
  controls.update();

  // Mode 1: Sensor Node active - show 3D IMU module tilting with MPU6050 pitch/roll
  if (activeTab === 'sensors') {
    if (robotMeshes.base) robotMeshes.base.visible = false;
    if (robotMeshes.sensorGroup) {
      robotMeshes.sensorGroup.visible = true;
      const pRad = (sensorState.pitch || 0) * (Math.PI / 180.0);
      const rRad = (sensorState.roll || 0) * (Math.PI / 180.0);
      robotMeshes.sensorGroup.rotation.x = pRad;
      robotMeshes.sensorGroup.rotation.z = rRad;
    }
  } else {
    // Mode 2: Arm / Quard Bot active - show robot arm meshes
    if (robotMeshes.base) robotMeshes.base.visible = true;
    if (robotMeshes.sensorGroup) robotMeshes.sensorGroup.visible = false;

    if (robotMeshes.turntable) {
      const yawDeg = jointState['turntable_link_joint_dup'] || 90.0;
      const shoulderDeg = jointState['turntable_link_joint'] || 59.0;
      const elbowDeg = jointState['turntable_link_joint_dup_1'] || 153.1;
      const wristDeg = jointState['turntable_link_joint_dup_3'] || 90.0;

      robotMeshes.turntable.rotation.y = (yawDeg - 90.0) * (Math.PI / 180.0);
      robotMeshes.shoulder.rotation.z = (shoulderDeg - 59.0) * (Math.PI / 180.0);
      robotMeshes.forearm.rotation.z = (elbowDeg - 153.1) * (Math.PI / 180.0);
      robotMeshes.wrist.rotation.x = (wristDeg - 90.0) * (Math.PI / 180.0);
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

    fetch('/api/toggle_sim', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ simulating: isSimulating })
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
    camera.position.set(350, 250, 350);
    controls.target.set(0, 80, 0);
    controls.update();
  });

  document.getElementById('btn-top-cam').addEventListener('click', () => {
    camera.position.set(0, 450, 0.1);
    controls.target.set(0, 80, 0);
    controls.update();
  });

  document.getElementById('btn-side-cam').addEventListener('click', () => {
    camera.position.set(450, 80, 0);
    controls.target.set(0, 80, 0);
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

  const panelArm = document.getElementById('panel-arm-control');
  const panelQuard = document.getElementById('panel-quard-control');
  const panelSensors = document.getElementById('panel-sensors-control');

  function setActive(tabName, btn, panel) {
    activeTab = tabName;
    [btnArm, btnQuard, btnSensors].forEach(b => b && b.classList.remove('active'));
    [panelArm, panelQuard, panelSensors].forEach(p => p && p.classList.remove('active'));

    if (btn) btn.classList.add('active');
    if (panel) panel.classList.add('active');
  }

  if (btnArm) {
    btnArm.addEventListener('click', () => setActive('arm', btnArm, panelArm));
  }
  if (btnQuard) {
    btnQuard.addEventListener('click', () => {
      setActive('quard', btnQuard, panelQuard);
      pingQuardBot();
    });
  }
  if (btnSensors) {
    btnSensors.addEventListener('click', () => setActive('sensors', btnSensors, panelSensors));
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
    const card = document.createElement('div');
    card.className = 'quard-channel-card';
    card.innerHTML = `
      <div class="quard-channel-header">
        <span class="quard-channel-name">⚙️ Ch ${c.ch}: ${c.name}</span>
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
      sendQuardCmd({ ch: c.ch, angle: angle });
    });
  });
}

function setupQuardEventListeners() {
  // Locomotion D-Pad Buttons
  const dpadButtons = document.querySelectorAll('.btn-dpad');
  dpadButtons.forEach(btn => {
    const cmd = btn.dataset.cmd;
    btn.addEventListener('click', () => {
      dpadButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      sendQuardCmd({ mode: cmd });
    });
  });

  // Preset Stances Buttons
  document.querySelectorAll('.btn-preset').forEach(btn => {
    btn.addEventListener('click', () => {
      const preset = btn.dataset.preset;
      sendQuardCmd({ mode: preset });
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
      QUARD_CHANNELS.forEach(c => {
        const slider = document.getElementById(`slider-quard-ch-${c.ch}`);
        const readout = document.getElementById(`readout-quard-ch-${c.ch}`);
        if (slider) slider.value = angle;
        if (readout) readout.innerText = `${angle}°`;
      });
      sendQuardCmd({ all: angle });
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

        // Update DHT11
        const tempC = data.temperature !== undefined ? data.temperature : 24.5;
        const tempF = (tempC * 9/5) + 32;
        const hum = data.humidity !== undefined ? data.humidity : 48.0;

        const elTemp = document.getElementById('val-temp');
        const elTempF = document.getElementById('val-temp-f');
        const elHum = document.getElementById('val-humidity');
        const meterTemp = document.getElementById('meter-temp');
        const meterHum = document.getElementById('meter-humidity');
        const badgeDht = document.getElementById('badge-dht-status');

        if (elTemp) elTemp.innerText = `${tempC.toFixed(2)} °C`;
        if (elTempF) elTempF.innerText = `(${tempF.toFixed(2)} °F)`;
        if (elHum) elHum.innerText = `${hum.toFixed(2)} %`;
        if (meterTemp) meterTemp.style.width = `${Math.min(100, Math.max(0, (tempC / 50) * 100))}%`;
        if (meterHum) meterHum.style.width = `${Math.min(100, Math.max(0, hum))}%`;

        if (badgeDht) {
          if (data.dht_error) {
            badgeDht.className = 'status-pill status-offline';
            badgeDht.innerText = 'ERROR';
          } else {
            badgeDht.className = 'status-pill status-online';
            badgeDht.innerText = 'OK';
          }
        }

        // Update MQ-5 Gas
        const gasVal = data.gas !== undefined ? data.gas : 415;
        const elGas = document.getElementById('val-gas');
        const meterGas = document.getElementById('meter-gas');
        const badgeGas = document.getElementById('badge-gas-status');

        if (elGas) elGas.innerText = `${gasVal} / 4095`;
        if (meterGas) {
          const pct = Math.min(100, Math.max(0, (gasVal / 4095) * 100));
          meterGas.style.width = `${pct.toFixed(1)}%`;
          if (gasVal > 1500) {
            meterGas.className = 'meter-fill meter-gas-danger';
          } else if (gasVal > 600) {
            meterGas.className = 'meter-fill meter-gas-warning';
          } else {
            meterGas.className = 'meter-fill meter-gas-fill';
          }
        }
        if (badgeGas) {
          if (data.air_quality === 'Danger') {
            badgeGas.className = 'status-pill status-offline';
            badgeGas.innerText = '⚠️ GAS ALERT!';
          } else if (data.air_quality === 'Moderate') {
            badgeGas.className = 'status-pill';
            badgeGas.style.backgroundColor = '#fef3c7';
            badgeGas.style.color = '#d97706';
            badgeGas.innerText = 'Moderate';
          } else {
            badgeGas.className = 'status-pill status-online';
            badgeGas.innerText = 'Clean Air';
          }
        }

        // Update MPU6050
        const accel = data.accel || { x: 0, y: 0, z: 9.81 };
        const gyro = data.gyro || { x: 0, y: 0, z: 0 };
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

        // Update Console Log
        const elConsole = document.getElementById('sensor-console-log');
        if (elConsole && data.raw_log && data.raw_log.length > 0) {
          elConsole.innerText = data.raw_log.slice(-30).join('\n');
          elConsole.scrollTop = elConsole.scrollHeight;
        }

        // Update Header Badge
        const textNode = document.getElementById('status-sensornode-text');
        const dotNode = document.getElementById('dot-sensornode');
        if (textNode && dotNode) {
          if (data.simulating) {
            textNode.innerText = 'Simulating';
            dotNode.parentElement.className = 'node-badge status-online';
          } else if (data.connected) {
            textNode.innerText = 'Hardware Serial';
            dotNode.parentElement.className = 'node-badge status-online';
          } else {
            textNode.innerText = 'Disconnected';
            dotNode.parentElement.className = 'node-badge status-offline';
          }
        }
      })
      .catch(err => {});
  }, 100); // 10 Hz telemetry sync
}
