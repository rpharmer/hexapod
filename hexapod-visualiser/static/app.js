const LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"];
const MOUNT_ANGLES = {
  LF: deg2rad(40),
  LM: deg2rad(90),
  LR: deg2rad(140),
  RF: deg2rad(-40),
  RM: deg2rad(-90),
  RR: deg2rad(-140),
};

const statusEl = document.getElementById("status");
const metaEl = document.getElementById("meta");
const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");

let model = {
  geometry: { coxa: 35, femur: 70, tibia: 110, body_radius: 60 },
  angles_deg: {
    LF: [0, 10, -25], LM: [0, 10, -25], LR: [0, 10, -25],
    RF: [0, 10, -25], RM: [0, 10, -25], RR: [0, 10, -25],
  },
  timestamp_ms: null,
};

function deg2rad(v) { return (v * Math.PI) / 180; }

function resize() {
  const ratio = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * ratio;
  canvas.height = canvas.clientHeight * ratio;
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}
window.addEventListener("resize", resize);
resize();

function project(p, scale, centerX, centerY) {
  const x = (p.x - p.y) * 0.88;
  const y = (p.x + p.y) * 0.45 - p.z;
  return { x: centerX + x * scale, y: centerY + y * scale };
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

function draw() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);

  const scale = Math.min(width, height) / 420;
  const centerX = width * 0.5;
  const centerY = height * 0.58;

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

  const ts = model.timestamp_ms ? new Date(model.timestamp_ms).toISOString() : "n/a";
  metaEl.textContent = `coxa:${model.geometry.coxa.toFixed(1)} femur:${model.geometry.femur.toFixed(1)} tibia:${model.geometry.tibia.toFixed(1)} | timestamp: ${ts}`;
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function connect() {
  const wsProtocol = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${wsProtocol}://${location.host}/ws`);

  ws.addEventListener("open", () => {
    statusEl.textContent = "Connected";
    statusEl.style.color = "#34d399";
  });

  ws.addEventListener("message", (event) => {
    try {
      const payload = JSON.parse(event.data);
      if (payload.type === "state") {
        model = {
          ...model,
          geometry: { ...model.geometry, ...(payload.geometry || {}) },
          angles_deg: { ...model.angles_deg, ...(payload.angles_deg || {}) },
          timestamp_ms: payload.timestamp_ms ?? model.timestamp_ms,
        };
      }
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

connect();
