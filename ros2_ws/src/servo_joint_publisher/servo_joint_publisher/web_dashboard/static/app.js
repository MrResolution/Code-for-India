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
  { id: 'turntable_link_joint_dup_4', name: 'Gripper Linkage', min: 0.0, max: 180.0, def: 90.0, rad_min: -1.0, rad_max: 1.0 }
];

let scene, camera, renderer, controls;
let robotMeshes = {};
let jointState = {};
let isSimulating = false;

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  initThreeJS();
  buildJointControls();
  setupEventListeners();
  startStatePolling();
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
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();

  // Apply real-time joint rotations to 3D meshes
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
