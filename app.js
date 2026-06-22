/* ==========================================================================
   RVCE MECH 4TH SEMESTER PROJECT - SMART CONVEYOR HMI & SIMULATOR
   ENGINE: app.js
   
   Contains the Physics loop, Kinematic Calculations, WebRTC controller,
   Chart.js Analog Plotter, and SCADA state machine.
   ========================================================================== */

// --- Global Constants & Configurations ---
const CANVAS_WIDTH = 800;
const CANVAS_HEIGHT = 280;

// X-coordinates of key stations on the conveyor belt (in pixels)
const SPAWN_X = -60;
const VISION_CAMERA_X = 220;
const PROX_SENSOR_X = 420;
const REJECT_ACTUATOR_X = 620;

const BELT_Y = 110;
const BELT_HEIGHT = 60;
const ITEM_SIZE = 40;

// Kinematic distance between proximity sensor and pneumatic actuator
// Mapping: 1 pixel = 1 mm (Hence, distance = 200 mm)
const PHYSICAL_DISTANCE_MM = REJECT_ACTUATOR_X - PROX_SENSOR_X; 

// --- Application State ---
let sysState = {
  running: true,
  eStopActive: false,
  hz: 30.0,              // VFD Frequency (0 - 60 Hz)
  beltSpeed: 2.5,        // Speed in px/frame (calculated from Hz)
  time: 0,
  sensorVoltage: 0.15,   // Simulated inductive sensor analog signal (0-10V)
  totalItemsScanned: 0,
  metalsRejected: 0,
  plasticsPassed: 0
};

// Array to hold simulated items
let conveyorItems = [];

// Pusher state machine
let pneumaticPiston = {
  extending: false,
  retracting: false,
  extension: 0,          // current extension in pixels (0 to maxExtension)
  maxExtension: 70,      // maximum reach to push items off the belt
  speed: 6,              // extension speed per frame
  width: 32,
  height: 40
};

// Timing/Queue for pneumatic reject commands (simulating PLC output queue)
let plcRejectQueue = [];

// --- WebRTC Camera Initialization ---
const webcamVideo = document.getElementById('webcamFeed');
const mockFeed = document.getElementById('mockFeed');
const cameraStatusPill = document.getElementById('statusCamera');

function initWebRTC() {
  if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
    navigator.mediaDevices.getUserMedia({ 
      video: { 
        width: { ideal: 640 }, 
        height: { ideal: 480 },
        facingMode: "user" 
      } 
    })
    .then(function(stream) {
      webcamVideo.srcObject = stream;
      webcamVideo.play();
      mockFeed.classList.add('hidden');
      cameraStatusPill.classList.add('active-green');
      cameraStatusPill.classList.remove('active-red');
      cameraStatusPill.querySelector('span:nth-child(2)').innerText = "VISION CAM: ONLINE";
    })
    .catch(function(error) {
      console.warn("Webcam access denied or unavailable. Running in mock simulation mode:", error);
      runMockCameraFeedback();
    });
  } else {
    runMockCameraFeedback();
  }
}

function runMockCameraFeedback() {
  mockFeed.classList.remove('hidden');
  cameraStatusPill.classList.add('active-cyan');
  cameraStatusPill.classList.remove('active-red', 'active-green');
  cameraStatusPill.querySelector('span:nth-child(2)').innerText = "VISION CAM: EMULATED";
}

// --- Chart.js Setup (Cathode Ray Oscilloscope style) ---
const chartContext = document.getElementById('sensorChart').getContext('2d');
const chartBufferLength = 60;
let chartLabels = Array(chartBufferLength).fill('');
let chartDataPoints = Array(chartBufferLength).fill(0.15);

const oscilloscopeChart = new Chart(chartContext, {
  type: 'line',
  data: {
    labels: chartLabels,
    datasets: [{
      label: 'Inductive Sensor Output (0-10V)',
      data: chartDataPoints,
      borderColor: '#00f0ff',
      borderWidth: 2,
      pointRadius: 0,
      fill: true,
      backgroundColor: 'rgba(0, 240, 255, 0.08)',
      tension: 0.2
    }]
  },
  options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    scales: {
      x: { display: false },
      y: {
        min: 0,
        max: 10,
        grid: {
          color: '#1e293b'
        },
        ticks: {
          color: '#94a3b8',
          font: { family: 'Share Tech Mono', size: 10 }
        }
      }
    },
    plugins: {
      legend: { display: false }
    }
  }
});

// --- Dynamic HMI Telemetry Updates ---
const frequencySlider = document.getElementById('speedSlider');
const displayHz = document.getElementById('valHz');
const displayRpm = document.getElementById('valRpm');
const displayVel = document.getElementById('valVel');
const displayTorque = document.getElementById('valTorque');
const displayCurrent = document.getElementById('valCurrent');
const displayDelay = document.getElementById('valDelay');

const statScans = document.getElementById('statScans');
const statRejects = document.getElementById('statRejects');
const statPassed = document.getElementById('statPassed');

const plcStatusPill = document.getElementById('statusPlc');
const vfdStatusPill = document.getElementById('statusVfd');

// Math functions matching standard VFD (Variable Frequency Drive) physics
function updateVFDPhysics() {
  if (sysState.eStopActive) {
    sysState.beltSpeed = 0;
    displayHz.innerText = "0.0 Hz";
    displayRpm.innerText = "0 RPM";
    displayVel.innerText = "0.00 m/s";
    displayTorque.innerText = "0.0 Nm";
    displayCurrent.innerText = "0.0 A";
    displayDelay.innerText = "INFINITE (HALTED)";
    
    vfdStatusPill.classList.remove('active-green');
    vfdStatusPill.classList.add('active-red');
    vfdStatusPill.querySelector('span:nth-child(2)').innerText = "VFD: TRIPPED (ESTOP)";
    return;
  }

  sysState.hz = parseFloat(frequencySlider.value);
  
  // Calculate speed proportional to motor frequency:
  // Let 60Hz = 3.0 m/s belt velocity, 0Hz = 0 m/s
  const physicalVelocity = (sysState.hz / 60.0) * 3.0; 
  sysState.beltSpeed = (sysState.hz / 60.0) * 5.0; // scale to canvas pixels per frame

  // Standard induction motor RPM: N = (120 * f) / P (Assume 4-pole motor with slip)
  const syncSpeed = (120 * sysState.hz) / 4;
  const motorRpm = Math.max(0, Math.round(syncSpeed * 0.96)); // 4% slip factor

  // Torque load simulation: torque remains constant in constant torque band (0-50Hz), falls in field weakening zone
  let torque = 0.0;
  let current = 0.0;
  if (sysState.hz > 0.5) {
    torque = sysState.hz <= 50 ? 28.5 + Math.random() * 0.4 : (28.5 * (50 / sysState.hz)) + Math.random() * 0.3;
    // Current is proportional to torque & active speed range
    current = (torque * 0.45) + (sysState.hz * 0.1) + Math.random() * 0.2;
  }

  // Kinematic calculations for Pneumatic Actuator Delay:
  // Time = Distance / Velocity (t = d / v)
  // Distance = PHYSICAL_DISTANCE_MM (200 mm)
  // Velocity = physicalVelocity (m/s). We convert to mm/s (1 m/s = 1000 mm/s)
  let delayMs = 0;
  if (physicalVelocity > 0.01) {
    const velocityMms = physicalVelocity * 1000;
    delayMs = (PHYSICAL_DISTANCE_MM / velocityMms) * 1000; // time in ms
    displayDelay.innerText = `${Math.round(delayMs)} ms`;
  } else {
    displayDelay.innerText = "INFINITE (HALTED)";
  }

  // Update DOM HMI Telemetry
  displayHz.innerText = `${sysState.hz.toFixed(1)} Hz`;
  displayRpm.innerText = `${motorRpm} RPM`;
  displayVel.innerText = `${physicalVelocity.toFixed(2)} m/s`;
  displayTorque.innerText = `${torque.toFixed(1)} Nm`;
  displayCurrent.innerText = `${current.toFixed(1)} A`;

  if (sysState.hz > 0.5) {
    vfdStatusPill.classList.add('active-green');
    vfdStatusPill.classList.remove('active-red');
    vfdStatusPill.querySelector('span:nth-child(2)').innerText = "VFD: FREQ CTRL MODE";
  } else {
    vfdStatusPill.classList.remove('active-green');
    vfdStatusPill.classList.add('active-cyan');
    vfdStatusPill.querySelector('span:nth-child(2)').innerText = "VFD: ZERO VELOCITY";
  }
}

frequencySlider.addEventListener('input', updateVFDPhysics);

// --- 2D CANVAS PHYSICS SIMULATOR ---
const canvas = document.getElementById('conveyorCanvas');
const ctx = canvas.getContext('2d');

function spawnItem(forcedType = null) {
  if (sysState.eStopActive) return;
  
  const type = forcedType || (Math.random() > 0.4 ? 'metal' : 'plastic');
  const itemId = ++sysState.totalItemsScanned;
  statScans.innerText = itemId;

  conveyorItems.push({
    id: itemId,
    x: SPAWN_X,
    y: BELT_Y + (BELT_HEIGHT - ITEM_SIZE) / 2, // perfectly centered on belt
    w: ITEM_SIZE,
    h: ITEM_SIZE,
    type: type,
    visionScanned: false,
    inductiveScanned: false,
    peakVoltage: 0.15,
    pushed: false,
    pushDistance: 0,
    discarded: false,
    speedFactor: 1.0
  });
}

// Bind spawn button
document.getElementById('btnSpawnMetal').addEventListener('click', () => spawnItem('metal'));
document.getElementById('btnSpawnPlastic').addEventListener('click', () => spawnItem('plastic'));

// --- EMERGENCY STOP CONTROL ---
const eStopBtn = document.getElementById('btnEstop');
const estopWrapper = document.getElementById('estopWrapper');

eStopBtn.addEventListener('click', () => {
  sysState.eStopActive = !sysState.eStopActive;

  if (sysState.eStopActive) {
    // Halting system
    sysState.running = false;
    estopWrapper.classList.add('halted');
    eStopBtn.innerText = "RESET SYSTEM (E-STOP ACTIVE)";
    
    plcStatusPill.classList.remove('active-green');
    plcStatusPill.classList.add('active-red');
    plcStatusPill.querySelector('span:nth-child(2)').innerText = "PLC: EMERGENCY STOP";
  } else {
    // Resuming system
    sysState.running = true;
    estopWrapper.classList.remove('halted');
    eStopBtn.innerText = "EMERGENCY STOP (E-STOP)";
    
    plcStatusPill.classList.add('active-green');
    plcStatusPill.classList.remove('active-red');
    plcStatusPill.querySelector('span:nth-child(2)').innerText = "PLC: LOGIC SOLVER RUNNING";
  }
  updateVFDPhysics();
});

// --- RENDER & PHYSICS LOOP ---
function drawConveyorBelt() {
  // Clear canvas
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // 1. Draw Roller Drums (Head & Tail pulleys)
  ctx.fillStyle = '#334155'; // Clean steel drum
  // Tail drum (Left)
  ctx.beginPath();
  ctx.arc(30, BELT_Y + BELT_HEIGHT/2, BELT_HEIGHT/2, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#475569';
  ctx.lineWidth = 3;
  ctx.stroke();

  // Head drum (Right)
  ctx.beginPath();
  ctx.arc(CANVAS_WIDTH - 30, BELT_Y + BELT_HEIGHT/2, BELT_HEIGHT/2, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // Draw spokes on the roller drums to show spin
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 4;
  let angle = (sysState.time * (sysState.beltSpeed * 0.05)) % (Math.PI * 2);
  
  // Left spokes
  ctx.beginPath();
  ctx.moveTo(30, BELT_Y + BELT_HEIGHT/2);
  ctx.lineTo(30 + Math.cos(angle) * (BELT_HEIGHT/2 - 2), BELT_Y + BELT_HEIGHT/2 + Math.sin(angle) * (BELT_HEIGHT/2 - 2));
  ctx.moveTo(30, BELT_Y + BELT_HEIGHT/2);
  ctx.lineTo(30 - Math.cos(angle) * (BELT_HEIGHT/2 - 2), BELT_Y + BELT_HEIGHT/2 - Math.sin(angle) * (BELT_HEIGHT/2 - 2));
  ctx.stroke();

  // Right spokes
  ctx.beginPath();
  ctx.moveTo(CANVAS_WIDTH - 30, BELT_Y + BELT_HEIGHT/2);
  ctx.lineTo(CANVAS_WIDTH - 30 + Math.cos(angle) * (BELT_HEIGHT/2 - 2), BELT_Y + BELT_HEIGHT/2 + Math.sin(angle) * (BELT_HEIGHT/2 - 2));
  ctx.moveTo(CANVAS_WIDTH - 30, BELT_Y + BELT_HEIGHT/2);
  ctx.lineTo(CANVAS_WIDTH - 30 - Math.cos(angle) * (BELT_HEIGHT/2 - 2), BELT_Y + BELT_HEIGHT/2 - Math.sin(angle) * (BELT_HEIGHT/2 - 2));
  ctx.stroke();

  // 2. Draw Conveyor Belt Bed
  ctx.fillStyle = '#0f172a'; // Belt base frame
  ctx.fillRect(30, BELT_Y, CANVAS_WIDTH - 60, BELT_HEIGHT);
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 2;
  ctx.strokeRect(30, BELT_Y, CANVAS_WIDTH - 60, BELT_HEIGHT);

  // Moving texture lines on the belt
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 2;
  let lineSpacing = 40;
  let offset = (sysState.time * sysState.beltSpeed) % lineSpacing;
  for (let x = 30 + offset; x < CANVAS_WIDTH - 30; x += lineSpacing) {
    ctx.beginPath();
    ctx.moveTo(x, BELT_Y);
    ctx.lineTo(x, BELT_Y + BELT_HEIGHT);
    ctx.stroke();
  }

  // 3. Draw Station Overlays (Sensors & Actuators)
  
  // A. Optical Vision Inspection Zone
  ctx.fillStyle = 'rgba(0, 240, 255, 0.05)';
  ctx.fillRect(VISION_CAMERA_X - 25, BELT_Y - 5, 50, BELT_HEIGHT + 10);
  ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
  ctx.lineWidth = 1;
  ctx.strokeRect(VISION_CAMERA_X - 25, BELT_Y - 5, 50, BELT_HEIGHT + 10);
  
  // Camera symbol
  ctx.fillStyle = '#00f0ff';
  ctx.fillRect(VISION_CAMERA_X - 8, BELT_Y - 30, 16, 12);
  ctx.beginPath();
  ctx.arc(VISION_CAMERA_X, BELT_Y - 24, 4, 0, Math.PI * 2);
  ctx.fillStyle = '#0b0f19';
  ctx.fill();

  // B. Inductive Proximity Sensor Zone
  ctx.fillStyle = 'rgba(234, 179, 8, 0.05)';
  ctx.fillRect(PROX_SENSOR_X - 15, BELT_Y - 5, 30, BELT_HEIGHT + 10);
  ctx.strokeStyle = 'rgba(234, 179, 8, 0.3)';
  ctx.lineWidth = 1;
  ctx.strokeRect(PROX_SENSOR_X - 15, BELT_Y - 5, 30, BELT_HEIGHT + 10);
  
  // Proximity sensor casing (yellow tip)
  ctx.fillStyle = '#eab308';
  ctx.fillRect(PROX_SENSOR_X - 8, BELT_Y - 20, 16, 12);
  ctx.fillStyle = '#475569';
  ctx.fillRect(PROX_SENSOR_X - 6, BELT_Y - 35, 12, 15);

  // C. Pneumatic Actuator (Push Plate and Piston Rod)
  // Draw Cylinder body (top of the belt, pushing downwards)
  ctx.fillStyle = '#475569'; // Cylinder grey
  ctx.fillRect(REJECT_ACTUATOR_X - pneumaticPiston.width/2, BELT_Y - 50, pneumaticPiston.width, 35);
  ctx.strokeStyle = '#334155';
  ctx.strokeRect(REJECT_ACTUATOR_X - pneumaticPiston.width/2, BELT_Y - 50, pneumaticPiston.width, 35);
  
  // Draw Cylinder rod
  ctx.fillStyle = '#cbd5e1'; // Chrome rod
  ctx.fillRect(REJECT_ACTUATOR_X - 4, BELT_Y - 15, 8, pneumaticPiston.extension);
  
  // Draw Pusher plate
  ctx.fillStyle = '#f43f5e'; // Red pusher plate
  ctx.fillRect(REJECT_ACTUATOR_X - 20, BELT_Y - 15 + pneumaticPiston.extension, 40, 8);

  // D. Metal Reject Bin (bottom of the belt)
  ctx.fillStyle = 'rgba(244, 63, 94, 0.08)';
  ctx.fillRect(REJECT_ACTUATOR_X - 35, BELT_Y + BELT_HEIGHT + 20, 70, 70);
  ctx.strokeStyle = '#f43f5e';
  ctx.lineWidth = 1;
  ctx.strokeRect(REJECT_ACTUATOR_X - 35, BELT_Y + BELT_HEIGHT + 20, 70, 70);
  ctx.fillStyle = '#f43f5e';
  ctx.font = '10px Share Tech Mono';
  ctx.fillText("REJECT BIN", REJECT_ACTUATOR_X - 26, BELT_Y + BELT_HEIGHT + 40);
  ctx.fillText("(Fe ONLY)", REJECT_ACTUATOR_X - 24, BELT_Y + BELT_HEIGHT + 55);

  // E. Passed Material Bin (at the right edge)
  ctx.fillStyle = 'rgba(16, 185, 129, 0.08)';
  ctx.fillRect(CANVAS_WIDTH - 60, BELT_Y + BELT_HEIGHT + 20, 50, 70);
  ctx.strokeStyle = '#10b981';
  ctx.strokeRect(CANVAS_WIDTH - 60, BELT_Y + BELT_HEIGHT + 20, 50, 70);
  ctx.fillStyle = '#10b981';
  ctx.fillText("PASS", CANVAS_WIDTH - 48, BELT_Y + BELT_HEIGHT + 45);
  ctx.fillText("BIN", CANVAS_WIDTH - 44, BELT_Y + BELT_HEIGHT + 60);
}

// Draw items, process sensor physics, run scheduling logic
function updateItems() {
  let activeVoltageValue = 0.15 + (Math.random() * 0.05); // Base background electrical noise
  let detectionBox = document.getElementById('visionDetectionBox');
  let visionWrapper = document.getElementById('visionFeedWrapper');
  let hasDetectedObject = false;

  for (let i = conveyorItems.length - 1; i >= 0; i--) {
    let item = conveyorItems[i];

    // 1. Update Physics Coordinates
    if (sysState.running) {
      if (!item.pushed) {
        // Move items forward with belt speed
        item.x += sysState.beltSpeed * item.speedFactor;
      } else {
        // Pushed downwards by pneumatic actuator
        item.y += 4;
        item.x += sysState.beltSpeed * 0.1; // minor forward friction momentum
      }
    }

    // 2. Machine Vision webcam overlay detection box trigger (X = 220)
    // Matches where camera scans items
    const centerOfItem = item.x + item.w/2;
    if (centerOfItem > VISION_CAMERA_X - 25 && centerOfItem < VISION_CAMERA_X + 25) {
      hasDetectedObject = true;
      item.visionScanned = true;

      // Map item's position on canvas to webcam HTML overlay bounds
      const canvasRect = canvas.getBoundingClientRect();
      const visionRect = visionWrapper.getBoundingClientRect();

      // Horizontal percentage on canvas, scaled to vision panel
      const horizontalPct = (item.x - (VISION_CAMERA_X - 45)) / 90.0;
      
      const overlayWidth = 60;
      const overlayHeight = 60;
      const overlayLeft = (visionRect.width / 2 - overlayWidth / 2) + (horizontalPct - 0.5) * 140;
      const overlayTop = (visionRect.height / 2 - overlayHeight / 2) - 10;

      detectionBox.style.display = 'flex';
      detectionBox.style.left = `${overlayLeft}px`;
      detectionBox.style.top = `${overlayTop}px`;
      detectionBox.style.width = `${overlayWidth}px`;
      detectionBox.style.height = `${overlayHeight}px`;

      // Update detection status label
      if (item.type === 'metal') {
        detectionBox.className = 'vision-detection-box detected-metal';
        detectionBox.innerHTML = `<span class="label">Fe Detected</span><span>Conf: 99.4%</span>`;
      } else {
        detectionBox.className = 'vision-detection-box detected-plastic';
        detectionBox.innerHTML = `<span class="label">Polymer</span><span>Conf: 98.1%</span>`;
      }
    }

    // 3. Inductive Proximity Sensor Electrical Logic (X = 420)
    const sensorCenterDistance = Math.abs(centerOfItem - PROX_SENSOR_X);
    if (sensorCenterDistance < 40) {
      item.inductiveScanned = true;
      // Calculate normal distribution curve for sensor output voltage
      // Max voltage for metal = 9.5V, for plastic = 0.8V
      const maxDetectionVolts = item.type === 'metal' ? 9.5 : 0.8;
      const voltageFormulaVal = maxDetectionVolts * Math.exp(-Math.pow(sensorCenterDistance / 20.0, 2));
      
      // Update instantaneous analog signal
      activeVoltageValue = Math.max(activeVoltageValue, voltageFormulaVal);

      // Record peak voltage read
      if (voltageFormulaVal > item.peakVoltage) {
        item.peakVoltage = voltageFormulaVal;
      }
    }

    // 4. PLC Decision Engine: Action Scheduling
    // When metal object passes proximity sensor center point, calculate queue delay to trigger ejector
    if (item.type === 'metal' && item.inductiveScanned && centerOfItem >= PROX_SENSOR_X && !item.pushed && !item.discarded) {
      
      // Determine physical time delay (in frames):
      // Distance from Proximity Sensor to Actuator center is 200px.
      // Speed is sysState.beltSpeed px/frame.
      // Delay in frames = 200 / beltSpeed.
      const delayInFrames = PHYSICAL_DISTANCE_MM / sysState.beltSpeed;
      
      // Check if this item already has a scheduled trigger
      const alreadyScheduled = plcRejectQueue.some(evt => evt.itemId === item.id);
      
      if (!alreadyScheduled && sysState.beltSpeed > 0.01) {
        plcRejectQueue.push({
          itemId: item.id,
          triggerFrame: sysState.time + delayInFrames
        });
      }
    }

    // 5. Draw the item on Canvas
    // Design details: Rounded containers, visual labeling, shadows
    ctx.shadowBlur = 4;
    ctx.shadowColor = 'rgba(0,0,0,0.5)';

    if (item.type === 'metal') {
      // Cylinder design for metal
      ctx.fillStyle = '#94a3b8'; // Steel color
      ctx.beginPath();
      ctx.roundRect(item.x, item.y, item.w, item.h, 6);
      ctx.fill();

      // Top steel shine line
      ctx.fillStyle = '#e2e8f0';
      ctx.fillRect(item.x + 2, item.y + 4, item.w - 4, 3);
      
      // Metallic ribs
      ctx.strokeStyle = '#475569';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(item.x + 10, item.y + 10);
      ctx.lineTo(item.x + 10, item.y + item.h - 10);
      ctx.moveTo(item.x + 20, item.y + 10);
      ctx.lineTo(item.x + 20, item.y + item.h - 10);
      ctx.moveTo(item.x + 30, item.y + 10);
      ctx.lineTo(item.x + 30, item.y + item.h - 10);
      ctx.stroke();

      ctx.fillStyle = '#0f172a';
      ctx.font = '8px Share Tech Mono';
      ctx.fillText("Fe", item.x + item.w/2 - 5, item.y + item.h/2 + 3);
    } else {
      // Brick/Box design for polymer
      ctx.fillStyle = '#f59e0b'; // Amber / polymer
      ctx.beginPath();
      ctx.roundRect(item.x, item.y, item.w, item.h, 4);
      ctx.fill();

      // Inner mold marking
      ctx.strokeStyle = '#b45309';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(item.x + 6, item.y + 6, item.w - 12, item.h - 12);
      
      ctx.fillStyle = '#78350f';
      ctx.font = '8px Share Tech Mono';
      ctx.fillText("POLY", item.x + item.w/2 - 10, item.y + item.h/2 + 3);
    }

    // Reset shadow for subsequent drawing
    ctx.shadowBlur = 0;

    // 6. Cleanup items passing into bins
    // A. Metal successfully rejected into reject bin
    if (item.pushed && item.y > BELT_Y + BELT_HEIGHT + 20) {
      if (!item.discarded) {
        item.discarded = true;
        sysState.metalsRejected++;
        statRejects.innerText = sysState.metalsRejected;
        logEvent(item, "REJECTED (Fe-ACTUATOR)");
      }
      conveyorItems.splice(i, 1);
    } 
    // B. Polymer passing successfully to the end
    else if (!item.pushed && item.x > CANVAS_WIDTH - 60) {
      if (!item.discarded) {
        item.discarded = true;
        sysState.plasticsPassed++;
        statPassed.innerText = sysState.plasticsPassed;
        logEvent(item, "PASS (NON-MAGNETIC)");
      }
      conveyorItems.splice(i, 1);
    }
  }

  // Hide bounding overlay on camera feed if no items are under scanner
  if (!hasDetectedObject) {
    detectionBox.style.display = 'none';
  }

  sysState.sensorVoltage = activeVoltageValue;
}

// --- PLC SCHEDULER & ACTUATOR ACTUATION ---
function processPLCOutputs() {
  if (!sysState.running) return;

  // 1. Scan reject queue to check if time delay has elapsed
  for (let k = plcRejectQueue.length - 1; k >= 0; k--) {
    let actionItem = plcRejectQueue[k];

    if (sysState.time >= actionItem.triggerFrame) {
      // Find the corresponding item on canvas
      let targetObj = conveyorItems.find(obj => obj.id === actionItem.itemId);

      if (targetObj && !targetObj.pushed) {
        // Fire pneumatic actuator!
        targetObj.pushed = true;
        pneumaticPiston.extending = true;
      }
      
      // Remove command from PLC queue
      plcRejectQueue.splice(k, 1);
    }
  }

  // 2. Control Actuator Extension/Retraction physics
  if (pneumaticPiston.extending) {
    pneumaticPiston.extension += pneumaticPiston.speed;
    if (pneumaticPiston.extension >= pneumaticPiston.maxExtension) {
      pneumaticPiston.extending = false;
      pneumaticPiston.retracting = true;
    }
  } else if (pneumaticPiston.retracting) {
    pneumaticPiston.extension -= pneumaticPiston.speed - 2; // retract slightly slower
    if (pneumaticPiston.extension <= 0) {
      pneumaticPiston.extension = 0;
      pneumaticPiston.retracting = false;
    }
  }
}

// --- LOGGING PLC ACTIONS ---
function logEvent(item, action) {
  const tableBody = document.getElementById('logBody');
  const tableRow = document.createElement('tr');
  
  const d = new Date();
  const timestamp = `${d.getHours().toString().padStart(2, '0')}:${d.getMinutes().toString().padStart(2, '0')}:${d.getSeconds().toString().padStart(2, '0')}:${d.getMilliseconds().toString().padStart(3, '0')}`;
  
  tableRow.className = item.type === 'metal' ? 'log-row-metal' : 'log-row-plastic';
  
  const isRejected = action.includes("REJECTED");
  const badgeClass = isRejected ? "badge-sorting reject" : "badge-sorting pass";
  const badgeLabel = isRejected ? "REJECT" : "PASS";

  tableRow.innerHTML = `
    <td>${timestamp}</td>
    <td>${item.id}</td>
    <td style="text-transform: uppercase; font-weight: 600;">${item.type}</td>
    <td>${item.peakVoltage.toFixed(2)} V</td>
    <td><span class="${badgeClass}">${badgeLabel}</span></td>
  `;

  tableBody.prepend(tableRow);

  // Keep max 20 logs to save performance
  if (tableBody.children.length > 20) {
    tableBody.removeChild(tableBody.lastChild);
  }
}

// --- CORE SYSTEM LOOP ---
function runSimulationLoop() {
  if (sysState.running) {
    sysState.time++;
    
    // Update canvas elements & physical models
    drawConveyorBelt();
    updateItems();
    processPLCOutputs();

    // Oscilloscope update
    if (sysState.time % 2 === 0) {
      chartDataPoints.push(sysState.sensorVoltage);
      chartDataPoints.shift();
      oscilloscopeChart.update('none'); // Update without full recalculation lag
    }
  } else if (sysState.eStopActive) {
    // If E-Stop active, flash warn signs on canvas
    drawConveyorBelt();
    updateItems();
    
    if (Math.floor(Date.now() / 500) % 2 === 0) {
      ctx.fillStyle = 'rgba(244, 63, 94, 0.4)';
      ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
      ctx.fillStyle = '#ffffff';
      ctx.font = '24px Share Tech Mono';
      ctx.textAlign = 'center';
      ctx.fillText("SYSTEM E-STOP TRIPPED", CANVAS_WIDTH/2, CANVAS_HEIGHT/2 + 8);
      ctx.textAlign = 'start'; // restore
    }
  }

  // Update clock UI
  const now = new Date();
  document.getElementById('clock').innerText = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}:${now.getMilliseconds().toString().padStart(3, '0')}`;

  requestAnimationFrame(runSimulationLoop);
}

// --- SETUP INITIALIZERS ---
initWebRTC();
updateVFDPhysics();
requestAnimationFrame(runSimulationLoop);

// Periodically feed material profiles automatically (Simulate random production line)
setInterval(() => {
  if (sysState.running && conveyorItems.length < 4) {
    spawnItem();
  }
}, 2400);
