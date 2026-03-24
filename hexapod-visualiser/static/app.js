const LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"];
const MOUNT_ANGLES = {
  LF: deg2rad(40),
  LM: deg2rad(90),
  LR: deg2rad(140),
  RF: deg2rad(-40),
  RM: deg2rad(-90),
  RR: deg2rad(-140),
};

const STALE_AFTER_MS = 1200;
const ROLLING_LIMIT = 8;

const CAMERA_PRESETS = {
  reset: { yaw: deg2rad(45), pitch: deg2rad(30), zoom: 1, panX: 0, panY: 0 },
  top: { yaw: deg2rad(0), pitch: deg2rad(-89), zoom: 1, panX: 0, panY: 0 },
  front: { yaw: deg2rad(0), pitch: deg2rad(8), zoom: 1, panX: 0, panY: 0 },
  side: { yaw: deg2rad(90), pitch: deg2rad(8), zoom: 1, panX: 0, panY: 0 },
};

const ZOOM_MIN = 0.35;
const ZOOM_MAX = 3.5;
const ORBIT_SENSITIVITY = 0.0085;
const PAN_SENSITIVITY = 1.0;
const PITCH_MIN = deg2rad(-89);
const PITCH_MAX = deg2rad(89);

const statusEl = document.getElementById("status");
const metaEl = document.getElementById("meta");
const rollingMetricsEl = document.getElementById("rolling-metrics");
const badgeModeEl = document.getElementById("badge-mode");
const badgeFaultEl = document.getElementById("badge-fault");
const badgeBusEl = document.getElementById("badge-bus");
const badgeEstimatorEl = document.getElementById("badge-estimator");
const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
const viewButtons = document.querySelectorAll("[data-view]");

let model = {
  geometry: { coxa: 35, femur: 70, tibia: 110, body_radius: 60 },
  angles_deg: {
    LF: [0, 10, -25], LM: [0, 10, -25], LR: [0, 10, -25],
    RF: [0, 10, -25], RM: [0, 10, -25], RR: [0, 10, -25],
  },
  timestamp_ms: null,
};

const telemetry = {
  active_mode: null,
  active_fault: null,
  bus_ok: null,
  estimator_valid: null,
  loop_counter: null,
  voltage: null,
  current: null,
  lastPayloadAtMs: 0,
  lastModelTimestampMs: null,
  lastAdvancingTimestampAtMs: 0,
  rolling: [],
};

const camera = { ...CAMERA_PRESETS.reset };
const interaction = {
  pointerId: null,
  mode: null,
  lastX: 0,
  lastY: 0,
};

function deg2rad(v) { return (v * Math.PI) / 180; }

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

function applyCameraPreset(name) {
  const preset = CAMERA_PRESETS[name];
  if (!preset) return;
  Object.assign(camera, preset);
}

function resize() {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * ratio;
  canvas.height = canvas.clientHeight * ratio;
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}
window.addEventListener("resize", resize);
resize();

function project(p, scale, centerX, centerY) {
  const cosYaw = Math.cos(camera.yaw);
  const sinYaw = Math.sin(camera.yaw);
  const cosPitch = Math.cos(camera.pitch);
  const sinPitch = Math.sin(camera.pitch);

  const yawX = p.x * cosYaw - p.y * sinYaw;
  const yawY = p.x * sinYaw + p.y * cosYaw;
  const yawZ = p.z;

  const pitchY = yawY * cosPitch - yawZ * sinPitch;

  return {
    x: centerX + yawX * scale,
    y: centerY + pitchY * scale,
  };
}

function fkForLeg(legName, anglesDeg, geometry) {
  const [coxaDeg, femurDeg, tibiaDeg] = anglesDeg;
  const mountYaw = MOUNT_ANGLES[legName];
  const yaw = mountYaw + deg2rad(coxaDeg);

  const coxa = geometry.coxa;
  const femur = geometry.femur;
  const tibia = geometry.tibia;
  const bodyR = geometry.body_radius;

  const anchor = {
    x: bodyR * Math.cos(mountYaw),
    y: bodyR * Math.sin(mountYaw),
    z: 0,
  };

  const shoulder = {
    x: anchor.x + coxa * Math.cos(yaw),
    y: anchor.y + coxa * Math.sin(yaw),
    z: 0,
  };

  const femurPitch = deg2rad(femurDeg);
  const tibiaPitch = deg2rad(tibiaDeg);

  const femurR = femur * Math.cos(femurPitch);
  const knee = {
    x: shoulder.x + femurR * Math.cos(yaw),
    y: shoulder.y + femurR * Math.sin(yaw),
    z: shoulder.z + femur * Math.sin(femurPitch),
  };

  const tibiaTotal = femurPitch + tibiaPitch;
  const tibiaR = tibia * Math.cos(tibiaTotal);
  const foot = {
    x: knee.x + tibiaR * Math.cos(yaw),
    y: knee.y + tibiaR * Math.sin(yaw),
    z: knee.z + tibia * Math.sin(tibiaTotal),
  };

  return { anchor, shoulder, knee, foot };
}

function setBadge(el, label, value, stateClass = "") {
  const shown = value === null || value === undefined || value === "" ? "n/a" : value;
  el.textContent = `${label}: ${shown}`;
  el.classList.remove("good", "bad", "warn");
  if (stateClass) el.classList.add(stateClass);
}

function formatAge(ageMs) {
  if (!Number.isFinite(ageMs)) return "n/a";
  if (ageMs < 1000) return `${Math.round(ageMs)}ms`;
  return `${(ageMs / 1000).toFixed(2)}s`;
}

function updateTelemetryUI(nowMs) {
  const modelAgeMs = telemetry.lastModelTimestampMs === null ? NaN : nowMs - telemetry.lastModelTimestampMs;
  const transportAgeMs = telemetry.lastPayloadAtMs ? nowMs - telemetry.lastPayloadAtMs : NaN;
  const staleData = !Number.isFinite(modelAgeMs) || modelAgeMs > STALE_AFTER_MS;

  setBadge(badgeModeEl, "mode", telemetry.active_mode, staleData ? "warn" : "good");

  const faultClass = telemetry.active_fault ? "bad" : staleData ? "warn" : "good";
  const faultText = telemetry.active_fault || "none";
  setBadge(badgeFaultEl, "fault", faultText, faultClass);

  let busLabel = telemetry.bus_ok;
  let busClass = "warn";
  if (telemetry.bus_ok === true) {
    busLabel = "ok";
    busClass = staleData ? "warn" : "good";
  } else if (telemetry.bus_ok === false) {
    busLabel = "down";
    busClass = "bad";
  }
  setBadge(badgeBusEl, "bus", busLabel, busClass);

  let estimatorLabel = telemetry.estimator_valid;
  let estimatorClass = "warn";
  if (telemetry.estimator_valid === true) {
    estimatorLabel = "valid";
    estimatorClass = staleData ? "warn" : "good";
  } else if (telemetry.estimator_valid === false) {
    estimatorLabel = "invalid";
    estimatorClass = "bad";
  }
  setBadge(badgeEstimatorEl, "estimator", estimatorLabel, estimatorClass);

  const freshness = staleData ? "STALE" : "LIVE";
  statusEl.textContent = `Connected | ${freshness} telemetry`;
  statusEl.style.color = staleData ? "#fbbf24" : "#34d399";
  statusEl.classList.toggle("stale", staleData);

  const rollingSample = `loop_age=${formatAge(modelAgeMs)} · telemetry_age=${formatAge(transportAgeMs)}`;
  if (telemetry.rolling[0] !== rollingSample) {
    telemetry.rolling.unshift(rollingSample);
    if (telemetry.rolling.length > ROLLING_LIMIT) {
      telemetry.rolling.length = ROLLING_LIMIT;
    }
  }
  rollingMetricsEl.textContent = telemetry.rolling.join("   |   ");

  const ts = model.timestamp_ms ? new Date(model.timestamp_ms).toISOString() : "n/a";
  const voltage = telemetry.voltage == null ? "n/a" : `${Number(telemetry.voltage).toFixed(2)}V`;
  const current = telemetry.current == null ? "n/a" : `${Number(telemetry.current).toFixed(2)}A`;
  const counter = telemetry.loop_counter == null ? "n/a" : telemetry.loop_counter;
  metaEl.textContent = `coxa:${model.geometry.coxa.toFixed(1)} femur:${model.geometry.femur.toFixed(1)} tibia:${model.geometry.tibia.toFixed(1)} | counter:${counter} | V:${voltage} I:${current} | timestamp:${ts}`;
}

function draw() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);

  const scale = (Math.min(width, height) / 420) * camera.zoom;
  const centerX = width * 0.5 + camera.panX;
  const centerY = height * 0.58 + camera.panY;

  // Ground grid.
  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 1;
  for (let i = -4; i <= 4; i += 1) {
    const a = project({ x: -220, y: i * 55, z: -120 }, scale, centerX, centerY);
    const b = project({ x: 220, y: i * 55, z: -120 }, scale, centerX, centerY);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  const bodyPts = LEG_ORDER.map((name) => {
    const mount = MOUNT_ANGLES[name];
    return {
      x: model.geometry.body_radius * Math.cos(mount),
      y: model.geometry.body_radius * Math.sin(mount),
      z: 0,
    };
  });

  // Body polygon.
  ctx.fillStyle = "rgba(59,130,246,0.25)";
  ctx.strokeStyle = "#60a5fa";
  ctx.lineWidth = 2;
  ctx.beginPath();
  bodyPts.forEach((p, index) => {
    const screen = project(p, scale, centerX, centerY);
    if (index === 0) ctx.moveTo(screen.x, screen.y);
    else ctx.lineTo(screen.x, screen.y);
  });
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  LEG_ORDER.forEach((legName) => {
    const points = fkForLeg(
      legName,
      model.angles_deg[legName] ?? [0, 0, 0],
      model.geometry,
    );
    const segments = [points.anchor, points.shoulder, points.knee, points.foot]
      .map((p) => project(p, scale, centerX, centerY));

    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(segments[0].x, segments[0].y);
    ctx.lineTo(segments[1].x, segments[1].y);
    ctx.stroke();

    ctx.strokeStyle = "#34d399";
    ctx.beginPath();
    ctx.moveTo(segments[1].x, segments[1].y);
    ctx.lineTo(segments[2].x, segments[2].y);
    ctx.lineTo(segments[3].x, segments[3].y);
    ctx.stroke();

    ctx.fillStyle = "#e5e7eb";
    segments.forEach((p) => {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
      ctx.fill();
    });

    ctx.fillStyle = "#9ca3af";
    ctx.font = "11px sans-serif";
    ctx.fillText(legName, segments[0].x + 5, segments[0].y - 5);
  });

  updateTelemetryUI(Date.now());
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function applyPayload(payload) {
  telemetry.lastPayloadAtMs = Date.now();

  if (payload.type === "state") {
    model = {
      ...model,
      geometry: { ...model.geometry, ...(payload.geometry || {}) },
      angles_deg: { ...model.angles_deg, ...(payload.angles_deg || {}) },
      timestamp_ms: payload.timestamp_ms ?? model.timestamp_ms,
    };

    if (typeof payload.timestamp_ms === "number") {
      const prev = telemetry.lastModelTimestampMs;
      telemetry.lastModelTimestampMs = payload.timestamp_ms;
      if (prev === null || payload.timestamp_ms > prev) {
        telemetry.lastAdvancingTimestampAtMs = Date.now();
      }
    }

    if (payload.active_mode !== undefined) telemetry.active_mode = payload.active_mode;
    if (payload.active_fault !== undefined) telemetry.active_fault = payload.active_fault;
    if (payload.bus_ok !== undefined) telemetry.bus_ok = payload.bus_ok;
    if (payload.estimator_valid !== undefined) telemetry.estimator_valid = payload.estimator_valid;
    if (payload.loop_counter !== undefined) telemetry.loop_counter = payload.loop_counter;
    if (payload.voltage !== undefined) telemetry.voltage = payload.voltage;
    if (payload.current !== undefined) telemetry.current = payload.current;
  }
}

function connect() {
  const wsProtocol = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${wsProtocol}://${location.host}/ws`);

  ws.addEventListener("open", () => {
    statusEl.textContent = "Connected";
    statusEl.style.color = "#34d399";
  });

  ws.addEventListener("message", (event) => {
    try {
      applyPayload(JSON.parse(event.data));
    } catch (err) {
      console.warn("invalid payload", err);
    }
  });

  ws.addEventListener("close", () => {
    statusEl.textContent = "Disconnected (retrying)";
    statusEl.style.color = "#f87171";
    setTimeout(connect, 1000);
  });
}

viewButtons.forEach((button) => {
  button.addEventListener("click", () => {
    applyCameraPreset(button.dataset.view);
  });
});

canvas.addEventListener("contextmenu", (event) => {
  event.preventDefault();
});

canvas.addEventListener("pointerdown", (event) => {
  interaction.pointerId = event.pointerId;
  interaction.lastX = event.clientX;
  interaction.lastY = event.clientY;

  if (event.button === 2 || event.button === 1 || event.shiftKey) {
    interaction.mode = "pan";
  } else {
    interaction.mode = "orbit";
  }

  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener("pointermove", (event) => {
  if (interaction.pointerId !== event.pointerId || !interaction.mode) return;

  const dx = event.clientX - interaction.lastX;
  const dy = event.clientY - interaction.lastY;
  interaction.lastX = event.clientX;
  interaction.lastY = event.clientY;

  if (interaction.mode === "orbit") {
    camera.yaw += dx * ORBIT_SENSITIVITY;
    camera.pitch = clamp(camera.pitch + dy * ORBIT_SENSITIVITY, PITCH_MIN, PITCH_MAX);
    return;
  }

  camera.panX += dx * PAN_SENSITIVITY;
  camera.panY += dy * PAN_SENSITIVITY;
});

function clearInteraction(event) {
  if (interaction.pointerId !== event.pointerId) return;
  interaction.pointerId = null;
  interaction.mode = null;
}

canvas.addEventListener("pointerup", clearInteraction);
canvas.addEventListener("pointercancel", clearInteraction);

canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const zoomFactor = Math.exp(-event.deltaY * 0.0015);
  camera.zoom = clamp(camera.zoom * zoomFactor, ZOOM_MIN, ZOOM_MAX);
}, { passive: false });

connect();
