/**
 * AERIS Research Project Page - Interactive Controller & Flight Canvas
 */

document.addEventListener('DOMContentLoaded', () => {
  initArchitectureInspector();
  initPerceptionViewer();
  initMissionFilters();
  initFlightSimulator();
  initCopyBibtex();
  initLightbox();
});

/* ==========================================================================
   1. Interactive Architecture Inspector
   ========================================================================== */
const ARCH_COMPONENTS = {
  'intent-router': {
    title: 'Intent Router & Goal Extractor',
    layer: 'Task Layer',
    freq: '~0.1 - 0.2 Hz (Aperiodic)',
    file: 'backend/planning/planner.py',
    summary: 'Parses unstructured natural language instructions, detects geometric short-circuits (e.g. "fly in a square"), or routes object-seeking directives to the semantic reasoning engine.',
    math: 'C_{\\text{intent}} = \\operatorname{classify}(T_{\\text{prompt}}) \\in \\{\\text{GEOMETRIC}, \\text{VISUAL\\_SEARCH}, \\text{INSPECTION}, \\text{REACTIVE}\\}',
    inputs: 'Unstructured user prompt string',
    outputs: 'Target category label, metric intent type, inspection standoff distance'
  },
  'vlm-engine': {
    title: 'Vision-Language Model (VLM)',
    layer: 'Task Layer',
    freq: '~0.1 - 0.5 Hz (Prompt Latency: 3.4 - 12.5s)',
    file: 'backend/models/gemini_provider.py / qwen_provider.py',
    summary: 'Evaluates the scene using Gemini 2.5 Flash API or local Qwen2-VL-2B to generate 2D bounding boxes [ymin, xmin, ymax, xmax] around the candidate object referenced in the text.',
    math: 'B_{\\text{2D}} = \\operatorname{VLM}(I_{\\text{overhead}}, \\text{prompt}) \\quad \\text{normalized} \\in [0, 1000]^4',
    inputs: 'High-res overhead RGB frame (TCP 5007) + Target description',
    outputs: 'Normalized 2D bounding box coordinates'
  },
  'skill-planner': {
    title: 'Skill Sequence Planner & Safety Validator',
    layer: 'Task Layer',
    freq: '~0.1 - 0.2 Hz',
    file: 'backend/planning/planner.py & safety_validator.py',
    summary: 'Translates high-level goals into atomic skill primitives (TAKEOFF, MOVE_TO, HOVER, ROTATE, CIRCLE, INSPECT, LAND). The formal safety validator enforces altitude caps (0.3m real / 1.5m twin), speed limits (0.4 m/s), and battery guards before execution.',
    math: '\\mathcal{S} = \\{s_1, s_2, \\dots, s_k\\}, \\quad s_i \\in \\operatorname{SKILL\\_ENUM}, \\quad \\forall i, \\; z(s_i) \\le z_{\\max}, \\; \\|v(s_i)\\| \\le v_{\\max}',
    inputs: 'Grounded 3D spatial target (X, Y, Z) + Obstacle map',
    outputs: 'Validated Skill Sequence JSON payload'
  },
  'depth-proposals': {
    title: 'Depth-Verified Spatial Grounding',
    layer: 'Mission Layer',
    freq: '5 - 10 Hz',
    file: 'backend/vision/scene_proposals.py & fast_cv.py',
    summary: 'Solves the fatal VLM hallucination problem. Back-projects overhead depth into a metric point cloud, removes the floor plane via RANSAC, segments physical clusters into 3D bounding spheres, and strictly verifies that the VLM box coincides with a real physical mass.',
    math: 'P_w = R_{\\text{cam}}^{\\top} \\left( d(u,v) \\cdot K^{-1} [u, v, 1]^{\\top} - t_{\\text{cam}} \\right), \\quad \\text{reject if } \\operatorname{IoU}(B_{\\text{VLM}}, B_{\\text{physical}}) = 0',
    inputs: 'Overhead RGB-D stream (720x960), ArUco calibration matrix',
    outputs: 'Metric 3D coordinates (x, y, z) in world metres'
  },
  'replan-manager': {
    title: 'Dynamic Closed-Loop Replan Manager',
    layer: 'Mission Layer',
    freq: '5 - 10 Hz',
    file: 'backend/runtime/replan_manager.py',
    summary: 'Continuously monitors target centroids during hovering or standoff. If external disturbance displaces the target by more than 15 cm, the executor interrupts current flight setpoints, queries an updated trajectory, and re-engages without human intervention.',
    math: '\\Delta p = \\|p_{\\text{target}}(t) - p_{\\text{target}}(0)\\|, \\quad \\text{if } \\Delta p > 0.15\\text{ m} \\implies \\operatorname{TriggerReplan}()',
    inputs: 'Live target tracker state, UAV position estimate',
    outputs: 'Interrupt signal, updated waypoint setpoints'
  },
  'mission-executor': {
    title: 'Mission Executor & Route Planner',
    layer: 'Mission Layer',
    freq: '10 Hz',
    file: 'backend/runtime/mission_executor.py',
    summary: 'Dispatches active skills sequentially, generates obstacle-avoiding geometric paths with 3D clearance, interpolates smooth velocity profiles, and streams 10 Hz setpoints over UDP.',
    math: 'x_{\\text{cmd}}(t) = \\operatorname{SmoothWaypointProfile}(s_i, \\Delta t)',
    inputs: 'Validated skill list, live obstacle bounding cylinders',
    outputs: 'Position & Yaw setpoints (x, y, z, yaw) over UDP 5005'
  },
  'cascaded-controller': {
    title: 'Cascaded Flight Controller',
    layer: 'Flight Layer',
    freq: '500 - 1000 Hz',
    file: 'matlab/twin/aeris_controller.slx / backend/drone/litewing_link.py',
    summary: 'A 4-tier cascaded architecture: Position P loop -> Velocity PI loop with anti-windup -> Attitude P loop -> Angular Rate PID loop -> Motor Allocation PWM mixer. Guaranteed deterministic real-time stability.',
    math: 'u(t) = K_p e(t) + K_i \\int e(\\tau) d\\tau + K_d \\frac{de(t)}{dt}, \\quad \\omega_i^2 = M^{-1} [T_{\\text{cmd}}, \\tau_x, \\tau_y, \\tau_z]^{\\top}',
    inputs: 'Position setpoints @ 10-50 Hz, Estimator feedback @ 500 Hz',
    outputs: 'Four motor PWM duty cycles (0 - 100%)'
  },
  'state-estimator': {
    title: 'Onboard State Estimator & Sensor Fusion',
    layer: 'Flight Layer',
    freq: '500 Hz',
    file: 'backend/drone/sim_interface.py / LiteWing firmware',
    summary: 'Fuses high-speed MPU-6050 IMU accelerometer/gyroscope with PMW3901 downward optical flow (velocity) and VL53L1X Time-of-Flight laser (altitude) into an Extended Kalman Filter (EKF).',
    math: '\\dot{x} = f(x, u) + w, \\quad z_k = h(x_k) + v_k, \\quad K_k = P_k^- H^{\\top} (H P_k^- H^{\\top} + R)^{-1}',
    inputs: 'IMU (500 Hz), Optical Flow (50 Hz), ToF (50 Hz)',
    outputs: 'Full 12-state vector: [x, y, z, u, v, w, phi, theta, psi, p, q, r]'
  },
  'digital-twin': {
    title: 'Simscape Multibody & Unreal Digital Twin',
    layer: 'Flight Layer',
    freq: '1000 Hz Physics / 50 Hz Render',
    file: 'matlab/twin/run_aeris_twin.m',
    summary: 'Full 6-DOF multibody rigid body model with floor/wall contact dynamics, non-linear motor torque/thrust curves, battery discharge sag, sensor noise injection, and real-time TCP socket camera rendering.',
    math: 'm \\ddot{r} = \\sum F_{\\text{thrust}} + F_{\\text{contact}} + m g + F_{\\text{gust}}, \\quad J \\dot{\\omega} + \\omega \\times J \\omega = \\sum \\tau',
    inputs: 'Motor commands (UDP 5005)',
    outputs: 'Multi-camera feeds (TCP 5007), 50Hz telemetry (UDP 5006)'
  },
  'litewing-hw': {
    title: 'LiteWing ESP32-S3 Micro-Quadrotor',
    layer: 'Flight Layer',
    freq: 'Real-time Hardware (Wi-Fi UDP)',
    file: 'backend/drone/litewing_link.py',
    summary: '45-gram open-hardware nano-drone (100mm diagonal) equipped with ESP32-S3 dual-core 240MHz MCU, MPU-6050, PMW3901, VL53L1X, and coreless 720 brushed motors.',
    math: 'm = 45\\text{ g}, \\quad I_{xx} \\approx 1.8 \\times 10^{-5}\\text{ kg m}^2, \\quad T/W \\approx 1.85',
    inputs: 'Setpoints over 802.11b/g/n Wi-Fi UDP',
    outputs: 'Telemetry packets, optical flow & ToF sensor readings'
  }
};

function initArchitectureInspector() {
  const nodes = document.querySelectorAll('.arch-node');
  const layerCards = document.querySelectorAll('.arch-layer-card');
  const layerBtns = document.querySelectorAll('.layer-btn');
  
  const inspTitle = document.getElementById('insp-title');
  const inspFile = document.getElementById('insp-file');
  const inspDesc = document.getElementById('insp-desc');
  const inspLayer = document.getElementById('insp-layer');
  const inspFreq = document.getElementById('insp-freq');
  const inspInputs = document.getElementById('insp-inputs');
  const inspOutputs = document.getElementById('insp-outputs');
  const inspMath = document.getElementById('insp-math');

  function selectNode(id) {
    const data = ARCH_COMPONENTS[id];
    if (!data) return;

    nodes.forEach(n => n.classList.remove('active-node'));
    const targetNode = document.querySelector(`.arch-node[data-id="${id}"]`);
    if (targetNode) targetNode.classList.add('active-node');

    if (inspTitle) inspTitle.textContent = data.title;
    if (inspFile) inspFile.textContent = data.file;
    if (inspDesc) inspDesc.textContent = data.summary;
    if (inspLayer) inspLayer.textContent = data.layer;
    if (inspFreq) inspFreq.textContent = data.freq;
    if (inspInputs) inspInputs.textContent = data.inputs;
    if (inspOutputs) inspOutputs.textContent = data.outputs;
    if (inspMath) inspMath.textContent = data.math;
  }

  nodes.forEach(node => {
    node.addEventListener('click', () => {
      const id = node.getAttribute('data-id');
      selectNode(id);
    });
  });

  // Layer filter buttons
  layerBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      layerBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const targetLayer = btn.getAttribute('data-layer');

      layerCards.forEach(card => {
        const cardLayer = card.getAttribute('data-layer');
        if (targetLayer === 'all' || cardLayer === targetLayer) {
          card.classList.remove('dimmed');
          card.classList.add('highlighted');
        } else {
          card.classList.add('dimmed');
          card.classList.remove('highlighted');
        }
      });
    });
  });

  // Default selection
  selectNode('depth-proposals');
}

/* ==========================================================================
   2. Perception Carousel & Visual Tabs
   ========================================================================== */
function initPerceptionViewer() {
  const tabs = document.querySelectorAll('.perception-tab');
  const mainImg = document.getElementById('perception-img');
  const captionTitle = document.getElementById('perception-cap-title');
  const captionDesc = document.getElementById('perception-cap-desc');

  const VIEWS = {
    'proposals': {
      src: 'assets/img/perception_depth_proposals.jpg',
      title: 'Depth-Verified Physical Object Proposals & Grounded Target',
      desc: 'The overhead depth map is filtered by RANSAC ground subtraction and clustered into metric 3D bounding spheres (green/cyan). The VLM prediction is matched against real physical geometry to eliminate hallucinated boxes on empty floor.'
    },
    'depth': {
      src: 'assets/img/camera_overhead_depth.jpg',
      title: 'Overhead Metric Depth Map (720x960)',
      desc: 'Calibrated ceiling-mounted depth sensor covering the complete 4m x 3m laboratory workspace with 1.72m to 2.92m range resolution.'
    },
    'overhead': {
      src: 'assets/img/camera_overhead.jpg',
      title: 'Overhead Calibrated RGB Camera',
      desc: 'High-resolution room viewpoint calibrated via 6 ArUco ground fiducials, yielding a sub-millimeter 0.78 px reprojection accuracy.'
    },
    'chase': {
      src: 'assets/img/camera_chase_inflight.jpg',
      title: 'Third-Person Chase View (In-Flight)',
      desc: 'Observer perspective showing the LiteWing micro-quadrotor autonomously navigating toward the target object in the Simscape digital twin.'
    },
    'fpv': {
      src: 'assets/img/camera_fpv_inflight.jpg',
      title: 'Onboard FPV Camera Feed',
      desc: 'First-person perspective streamed from the vehicle nose camera during target inspection and close-proximity standoff.'
    }
  };

  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      tabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      const viewKey = tab.getAttribute('data-view');
      const data = VIEWS[viewKey];
      if (data && mainImg) {
        mainImg.src = data.src;
        if (captionTitle) captionTitle.textContent = data.title;
        if (captionDesc) captionDesc.textContent = data.desc;
      }
    });
  });
}

/* ==========================================================================
   3. Mission Gallery Filter & Lightbox
   ========================================================================== */
function initMissionFilters() {
  const filterBtns = document.querySelectorAll('.filter-btn');
  const cards = document.querySelectorAll('.mission-card');

  filterBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      filterBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      const category = btn.getAttribute('data-filter');

      cards.forEach(card => {
        const cardCats = card.getAttribute('data-category') || '';
        if (category === 'all' || cardCats.includes(category)) {
          card.style.display = 'flex';
        } else {
          card.style.display = 'none';
        }
      });
    });
  });
}

function initLightbox() {
  const modal = document.getElementById('lightbox-modal');
  const modalImg = document.getElementById('modal-img');
  const modalTitle = document.getElementById('modal-title');
  const modalDesc = document.getElementById('modal-desc');
  const closeBtn = document.getElementById('modal-close');
  const triggerBtns = document.querySelectorAll('.btn-inspect-path');

  if (!modal) return;

  triggerBtns.forEach(btn => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      const pathImg = btn.getAttribute('data-path-img');
      const title = btn.getAttribute('data-title');
      const desc = btn.getAttribute('data-desc');

      if (modalImg) modalImg.src = pathImg;
      if (modalTitle) modalTitle.textContent = title;
      if (modalDesc) modalDesc.textContent = desc;
      modal.classList.add('open');
    });
  });

  if (closeBtn) {
    closeBtn.addEventListener('click', () => {
      modal.classList.remove('open');
    });
  }

  modal.addEventListener('click', (e) => {
    if (e.target === modal) modal.classList.remove('open');
  });
}

/* ==========================================================================
   4. Interactive Flight Trajectory Simulator (HTML5 Canvas)
   ========================================================================== */
function initFlightSimulator() {
  const canvas = document.getElementById('flight-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');

  // Trajectory datasets for replay
  const TRAJECTORIES = {
    'case02': {
      title: 'Case 02: "Find red bottle and hover above it"',
      start: { x: -1.8, y: -1.4, z: 0.0 },
      target: { x: -0.81, y: -0.61, z: 0.72 },
      obstacles: [{ x: -1.4, y: -0.9, r: 0.25, name: 'Drum' }],
      waypoints: [
        { x: -1.8, y: -1.4, z: 0.0, label: 'TAKEOFF' },
        { x: -1.8, y: -1.4, z: 0.8, label: 'CLIMB' },
        { x: -1.3, y: -1.1, z: 0.8, label: 'WAYPOINT 1' },
        { x: -0.81, y: -0.61, z: 0.72, label: 'HOVER TARGET' }
      ],
      duration: 18.7,
      errorCm: '0.8 cm'
    },
    'case09': {
      title: 'Case 09: Dynamic Replan on Moved Target',
      start: { x: -1.8, y: -1.4, z: 0.0 },
      target: { x: -0.80, y: -0.60, z: 0.72 },
      targetMoved: { x: -0.45, y: -0.55, z: 0.72 },
      obstacles: [{ x: -1.2, y: -0.8, r: 0.2, name: 'Crate' }],
      waypoints: [
        { x: -1.8, y: -1.4, z: 0.0, label: 'TAKEOFF' },
        { x: -0.80, y: -0.60, z: 0.72, label: 'INITIAL HOVER' },
        { x: -0.45, y: -0.55, z: 0.72, label: 'REPLAN WAYPOINT' }
      ],
      duration: 22.0,
      errorCm: '5.0 cm'
    },
    'case17': {
      title: 'Case 17: Star Pattern Aerobatics',
      start: { x: 0.0, y: 0.0, z: 0.0 },
      target: { x: 0.0, y: 0.0, z: 0.8 },
      obstacles: [],
      waypoints: [
        { x: 0.0, y: 0.8, z: 0.8, label: 'POINT 1' },
        { x: 0.25, y: -0.7, z: 0.8, label: 'POINT 2' },
        { x: -0.75, y: 0.25, z: 0.8, label: 'POINT 3' },
        { x: 0.75, y: 0.25, z: 0.8, label: 'POINT 4' },
        { x: -0.25, y: -0.7, z: 0.8, label: 'POINT 5' },
        { x: 0.0, y: 0.8, z: 0.8, label: 'CLOSE' }
      ],
      duration: 33.5,
      errorCm: '1.2 cm'
    }
  };

  let activeCaseKey = 'case02';
  let simTime = 0;
  let isPlaying = true;
  let speedMultiplier = 1;
  let animFrameId = null;

  // DOM elements for telemetry HUD
  const hudX = document.getElementById('hud-x');
  const hudY = document.getElementById('hud-y');
  const hudZ = document.getElementById('hud-z');
  const hudSpeed = document.getElementById('hud-spd');
  const hudYaw = document.getElementById('hud-yaw');
  const hudError = document.getElementById('hud-err');
  const hudStatus = document.getElementById('hud-status');

  // World bounds in metres: x from -2.2 to +1.8, y from -1.8 to +1.6
  const W_XMIN = -2.2, W_XMAX = 1.8;
  const W_YMIN = -1.8, W_YMAX = 1.6;

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * window.devicePixelRatio || 800;
    canvas.height = rect.height * window.devicePixelRatio || 400;
  }
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  function worldToCanvas(wx, wy) {
    const cw = canvas.width;
    const ch = canvas.height;
    const padding = 40;
    const sx = (wx - W_XMIN) / (W_XMAX - W_XMIN);
    const sy = 1.0 - (wy - W_YMIN) / (W_YMAX - W_YMIN);
    return {
      x: padding + sx * (cw - padding * 2),
      y: padding + sy * (ch - padding * 2)
    };
  }

  function getDroneStateAtTime(traj, t) {
    const wps = traj.waypoints;
    const totalWps = wps.length;
    const dur = traj.duration;
    const progress = Math.min(1.0, Math.max(0.0, t / dur));
    
    const segCount = totalWps - 1;
    const segFloat = progress * segCount;
    const segIdx = Math.min(segCount - 1, Math.floor(segFloat));
    const segT = segFloat - segIdx;

    // Smooth hermite interpolation
    const easeT = segT * segT * (3 - 2 * segT);
    const p1 = wps[segIdx];
    const p2 = wps[segIdx + 1];

    const curX = p1.x + (p2.x - p1.x) * easeT;
    const curY = p1.y + (p2.y - p1.y) * easeT;
    const curZ = p1.z + (p2.z - p1.z) * easeT;

    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const yaw = Math.atan2(dy, dx) * (180 / Math.PI);
    const spd = Math.sqrt(dx * dx + dy * dy) * 0.45;

    return { x: curX, y: curY, z: curZ, yaw: yaw, speed: spd, progress: progress };
  }

  function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const traj = TRAJECTORIES[activeCaseKey];
    if (!traj) return;

    // 1. Draw Grid
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    for (let x = -2.0; x <= 1.5; x += 0.5) {
      const p1 = worldToCanvas(x, -1.8);
      const p2 = worldToCanvas(x, 1.6);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    for (let y = -1.5; y <= 1.5; y += 0.5) {
      const p1 = worldToCanvas(-2.2, y);
      const p2 = worldToCanvas(1.8, y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }

    // 2. Draw Workspace Boundary
    const b1 = worldToCanvas(-2.0, -1.6);
    const b2 = worldToCanvas(1.6, 1.4);
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.strokeRect(b1.x, b2.y, b2.x - b1.x, b1.y - b2.y);
    ctx.setLineDash([]);

    // 3. Draw Obstacles
    if (traj.obstacles) {
      traj.obstacles.forEach(obs => {
        const pt = worldToCanvas(obs.x, obs.y);
        ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, obs.r * 80, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = '#f87171';
        ctx.font = '11px monospace';
        ctx.fillText(obs.name, pt.x - 14, pt.y - obs.r * 80 - 6);
      });
    }

    // 4. Draw Planned Trajectory Line
    ctx.strokeStyle = 'rgba(2, 132, 199, 0.4)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    traj.waypoints.forEach((wp, idx) => {
      const pt = worldToCanvas(wp.x, wp.y);
      if (idx === 0) ctx.moveTo(pt.x, pt.y);
      else ctx.lineTo(pt.x, pt.y);
    });
    ctx.stroke();

    // Draw waypoints
    traj.waypoints.forEach((wp) => {
      const pt = worldToCanvas(wp.x, wp.y);
      ctx.fillStyle = '#38bdf8';
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, 4, 0, Math.PI * 2);
      ctx.fill();
    });

    // 5. Draw Target Object
    const curTarget = (activeCaseKey === 'case09' && simTime > 12 && traj.targetMoved) 
                      ? traj.targetMoved 
                      : traj.target;
    const tgtPt = worldToCanvas(curTarget.x, curTarget.y);
    ctx.fillStyle = 'rgba(234, 179, 8, 0.3)';
    ctx.strokeStyle = '#eab308';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tgtPt.x, tgtPt.y, 14, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#fbbf24';
    ctx.font = '11px sans-serif';
    ctx.fillText('Target (Metric)', tgtPt.x + 18, tgtPt.y + 4);

    // 6. Draw Drone Current State
    const state = getDroneStateAtTime(traj, simTime);
    const dronePt = worldToCanvas(state.x, state.y);

    // Flown path trail
    ctx.strokeStyle = '#06b6d4';
    ctx.lineWidth = 3;
    ctx.beginPath();
    for (let t = 0; t <= simTime; t += 0.2) {
      const st = getDroneStateAtTime(traj, t);
      const p = worldToCanvas(st.x, st.y);
      if (t === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();

    // Quadrotor Icon
    ctx.save();
    ctx.translate(dronePt.x, dronePt.y);
    ctx.rotate((state.yaw * Math.PI) / 180);

    // Quad arms
    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-16, -16);
    ctx.lineTo(16, 16);
    ctx.moveTo(16, -16);
    ctx.lineTo(-16, 16);
    ctx.stroke();

    // Rotors
    ctx.fillStyle = '#38bdf8';
    [[-16, -16], [16, -16], [16, 16], [-16, 16]].forEach(([rx, ry]) => {
      ctx.beginPath();
      ctx.arc(rx, ry, 6, 0, Math.PI * 2);
      ctx.fill();
    });

    // Heading nose
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(0, -18, 4, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();

    // Update Telemetry HUD
    if (hudX) hudX.textContent = state.x.toFixed(2) + ' m';
    if (hudY) hudY.textContent = state.y.toFixed(2) + ' m';
    if (hudZ) hudZ.textContent = state.z.toFixed(2) + ' m';
    if (hudSpeed) hudSpeed.textContent = (state.speed || 0.35).toFixed(2) + ' m/s';
    if (hudYaw) hudYaw.textContent = Math.round(state.yaw) + '°';
    if (hudError) hudError.textContent = traj.errorCm;
    if (hudStatus) {
      if (state.progress >= 0.98) {
        hudStatus.textContent = 'HOVER / GOAL REACHED';
        hudStatus.style.color = '#34d399';
      } else if (activeCaseKey === 'case09' && simTime > 12) {
        hudStatus.textContent = 'REPLANNING ACTIVE';
        hudStatus.style.color = '#f59e0b';
      } else {
        hudStatus.textContent = 'EXECUTING SKILL';
        hudStatus.style.color = '#38bdf8';
      }
    }

    // Step time forward
    if (isPlaying) {
      simTime += 0.035 * speedMultiplier;
      if (simTime > traj.duration) {
        simTime = 0; // Loop seamlessly
      }
    }

    animFrameId = requestAnimationFrame(render);
  }

  // Trajectory selector buttons
  const simCaseBtns = document.querySelectorAll('.btn-sim-case');
  simCaseBtns.forEach(btn => {
    btn.addEventListener('click', () => {
      simCaseBtns.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      activeCaseKey = btn.getAttribute('data-case');
      simTime = 0;
    });
  });

  // Play / Pause / Reset controls
  const btnPlay = document.getElementById('btn-sim-play');
  const btnReset = document.getElementById('btn-sim-reset');
  const btnSpeed = document.getElementById('btn-sim-speed');

  if (btnPlay) {
    btnPlay.addEventListener('click', () => {
      isPlaying = !isPlaying;
      btnPlay.textContent = isPlaying ? '⏸ Pause' : '▶ Play';
    });
  }

  if (btnReset) {
    btnReset.addEventListener('click', () => {
      simTime = 0;
    });
  }

  if (btnSpeed) {
    btnSpeed.addEventListener('click', () => {
      if (speedMultiplier === 1) speedMultiplier = 2;
      else if (speedMultiplier === 2) speedMultiplier = 4;
      else speedMultiplier = 1;
      btnSpeed.textContent = speedMultiplier + 'x Speed';
    });
  }

  // Start animation loop
  render();
}

/* ==========================================================================
   5. BibTeX Copy to Clipboard
   ========================================================================== */
function initCopyBibtex() {
  const btn = document.getElementById('btn-copy-bib');
  const bibCode = document.getElementById('bibtex-code');

  if (btn && bibCode) {
    btn.addEventListener('click', () => {
      const text = bibCode.innerText;
      navigator.clipboard.writeText(text).then(() => {
        const originalText = btn.innerHTML;
        btn.innerHTML = '✓ Copied!';
        setTimeout(() => {
          btn.innerHTML = originalText;
        }, 2000);
      }).catch(err => {
        console.error('Failed to copy BibTeX: ', err);
      });
    });
  }
}
