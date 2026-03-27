import {
  LEG_ORDER,
  MOUNT_ANGLES,
  STALE_AFTER_MS,
  ROLLING_LIMIT,
  deg2rad,
} from "./constants.js";

function formatAge(ageMs) {
  if (!Number.isFinite(ageMs)) return "n/a";
  if (ageMs < 1000) return `${Math.round(ageMs)}ms`;
  return `${(ageMs / 1000).toFixed(2)}s`;
}

function project(point, camera, scale, centerX, centerY) {
  const cosYaw = Math.cos(camera.yaw);
  const sinYaw = Math.sin(camera.yaw);
  const cosPitch = Math.cos(camera.pitch);
  const sinPitch = Math.sin(camera.pitch);

  const yawX = point.x * cosYaw - point.y * sinYaw;
  const yawY = point.x * sinYaw + point.y * cosYaw;
  const yawZ = point.z;

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

function resolveModelAgeMs(telemetry, nowMs) {
  if (!Number.isFinite(telemetry.lastModelTimestampMs)) {
    return NaN;
  }

  const ageMs = nowMs - telemetry.lastModelTimestampMs;
  const maxExpectedAgeMs = 24 * 60 * 60 * 1000;
  if (ageMs < -STALE_AFTER_MS || ageMs > maxExpectedAgeMs) {
    return NaN;
  }

  return ageMs;
}

function updateTelemetryUI({ statusEl, metaEl, rollingMetricsEl, telemetry, model }, nowMs) {
  const transportAgeMs = telemetry.lastPayloadAtMs ? nowMs - telemetry.lastPayloadAtMs : NaN;
  const modelAgeCandidateMs = resolveModelAgeMs(telemetry, nowMs);
  const modelAgeMs = Number.isFinite(modelAgeCandidateMs) ? modelAgeCandidateMs : transportAgeMs;
  const staleData = !Number.isFinite(transportAgeMs) || transportAgeMs > STALE_AFTER_MS;

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
  const missionProgress = resolveMissionProgress(model);
  const activeDistanceM = resolveActiveWaypointDistanceM(model);
  const targetDistance =
    activeDistanceM === null ? "target:n/a" : `target_dist:${activeDistanceM.toFixed(2)}m`;
  metaEl.textContent = `coxa:${model.geometry.coxa.toFixed(1)} femur:${model.geometry.femur.toFixed(1)} tibia:${model.geometry.tibia.toFixed(1)} | ${missionProgress} ${targetDistance} | timestamp:${ts}`;
}

function resolveMissionProgress(model) {
  const autonomy = model.autonomy_debug;
  if (!autonomy || !Array.isArray(autonomy.waypoints) || autonomy.waypoints.length === 0) {
    return "mission:n/a";
  }
  const idx = Number.isInteger(autonomy.active_waypoint_index) ? autonomy.active_waypoint_index : 0;
  const clampedIdx = Math.max(0, Math.min(idx, autonomy.waypoints.length));
  return `mission:${clampedIdx}/${autonomy.waypoints.length}`;
}

function resolveActiveWaypointDistanceM(model) {
  const autonomy = model.autonomy_debug;
  const pose = autonomy?.current_pose;
  const waypoints = autonomy?.waypoints;
  if (!pose || !Array.isArray(waypoints) || waypoints.length === 0) {
    return null;
  }
  const idx = Number.isInteger(autonomy.active_waypoint_index) ? autonomy.active_waypoint_index : 0;
  const active = waypoints[Math.max(0, Math.min(idx, waypoints.length - 1))];
  if (
    !Number.isFinite(pose.x_m) ||
    !Number.isFinite(pose.y_m) ||
    !Number.isFinite(active?.x_m) ||
    !Number.isFinite(active?.y_m)
  ) {
    return null;
  }
  return Math.hypot(active.x_m - pose.x_m, active.y_m - pose.y_m);
}

function resolvePoseOffsetMm(model) {
  const pose = model.autonomy_debug?.current_pose;
  const x = Number.isFinite(pose?.x_m) ? pose.x_m * 1000 : 0;
  const y = Number.isFinite(pose?.y_m) ? pose.y_m * 1000 : 0;
  const yaw = Number.isFinite(pose?.yaw_rad) ? pose.yaw_rad : 0;
  return { x, y, yaw };
}

function transformBodyToWorld(point, poseOffset) {
  const cosYaw = Math.cos(poseOffset.yaw);
  const sinYaw = Math.sin(poseOffset.yaw);
  return {
    x: poseOffset.x + point.x * cosYaw - point.y * sinYaw,
    y: poseOffset.y + point.x * sinYaw + point.y * cosYaw,
    z: point.z,
  };
}

function drawAutonomyDebugOverlay({ ctx, camera, scale, centerX, centerY, model, poseOffset }) {
  const autonomy = model.autonomy_debug;
  if (!autonomy || !Array.isArray(autonomy.waypoints) || autonomy.waypoints.length === 0) {
    return;
  }

  const waypointPoints = autonomy.waypoints
    .filter((waypoint) => Number.isFinite(waypoint?.x_m) && Number.isFinite(waypoint?.y_m))
    .map((waypoint) => ({
      x: waypoint.x_m * 1000,
      y: waypoint.y_m * 1000,
      z: -115,
    }));

  if (waypointPoints.length === 0) {
    return;
  }

  const activeIndex = Number.isInteger(autonomy.active_waypoint_index) ? autonomy.active_waypoint_index : 0;
  const completedCount = Math.max(0, Math.min(activeIndex, waypointPoints.length));

  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(99, 102, 241, 0.85)";
  ctx.beginPath();
  waypointPoints.forEach((point, index) => {
    const screen = project(point, camera, scale, centerX, centerY);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
      return;
    }
    ctx.lineTo(screen.x, screen.y);
  });
  ctx.stroke();

  waypointPoints.forEach((point, index) => {
    const screen = project(point, camera, scale, centerX, centerY);
    const isCompleted = index < completedCount;
    const isActive = completedCount < waypointPoints.length && index === completedCount;
    ctx.fillStyle = isActive ? "#f59e0b" : isCompleted ? "#34d399" : "#60a5fa";
    const radius = isActive ? 6 : 4;
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#cbd5e1";
    ctx.font = "10px sans-serif";
    ctx.fillText(`W${index}`, screen.x + 6, screen.y - 6);

    const yawRad = autonomy.waypoints[index]?.yaw_rad;
    if (Number.isFinite(yawRad)) {
      const headingPoint = {
        x: point.x + Math.cos(yawRad) * 22,
        y: point.y + Math.sin(yawRad) * 22,
        z: point.z,
      };
      const headingScreen = project(headingPoint, camera, scale, centerX, centerY);
      ctx.strokeStyle = isActive ? "#f59e0b" : "#93c5fd";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(screen.x, screen.y);
      ctx.lineTo(headingScreen.x, headingScreen.y);
      ctx.stroke();
    }
  });

  if (autonomy.current_pose && Number.isFinite(autonomy.current_pose.x_m) && Number.isFinite(autonomy.current_pose.y_m)) {
    const posePoint = { x: poseOffset.x, y: poseOffset.y, z: -115 };
    const heading = poseOffset.yaw;
    const nosePoint = {
      x: posePoint.x + Math.cos(heading) * 45,
      y: posePoint.y + Math.sin(heading) * 45,
      z: posePoint.z,
    };

    const poseScreen = project(posePoint, camera, scale, centerX, centerY);
    const noseScreen = project(nosePoint, camera, scale, centerX, centerY);

    ctx.strokeStyle = "#fbbf24";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(poseScreen.x, poseScreen.y);
    ctx.lineTo(noseScreen.x, noseScreen.y);
    ctx.stroke();

    ctx.fillStyle = "#fbbf24";
    ctx.beginPath();
    ctx.arc(poseScreen.x, poseScreen.y, 5, 0, Math.PI * 2);
    ctx.fill();

    const activeWaypoint = waypointPoints[Math.max(0, Math.min(activeIndex, waypointPoints.length - 1))];
    if (activeWaypoint) {
      const activeScreen = project(activeWaypoint, camera, scale, centerX, centerY);
      ctx.strokeStyle = "rgba(245, 158, 11, 0.75)";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.moveTo(poseScreen.x, poseScreen.y);
      ctx.lineTo(activeScreen.x, activeScreen.y);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  ctx.fillStyle = "#cbd5e1";
  ctx.font = "11px sans-serif";
  ctx.fillText("Autonomy overlay: green=completed, amber=active, blue=pending", 12, 22);
}

export function createSceneRenderer({ canvas, ctx, camera, statusEl, metaEl, rollingMetricsEl, modelRef, telemetry }) {
  function resize() {
    const ratio = window.devicePixelRatio || 1;
    canvas.width = canvas.clientWidth * ratio;
    canvas.height = canvas.clientHeight * ratio;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  }

  function drawFrame() {
    const model = modelRef.current;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    ctx.clearRect(0, 0, width, height);

    const scale = (Math.min(width, height) / 420) * camera.zoom;
    const centerX = width * 0.5 + camera.panX;
    const centerY = height * 0.58 + camera.panY;
    const poseOffset = resolvePoseOffsetMm(model);
    const gridSpacingMm = 55;
    const gridExtentMm = 220;

    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 1;
    for (let i = -5; i <= 5; i += 1) {
      const worldY = i * gridSpacingMm;
      const a = project({ x: -gridExtentMm, y: worldY, z: -120 }, camera, scale, centerX, centerY);
      const b = project({ x: gridExtentMm, y: worldY, z: -120 }, camera, scale, centerX, centerY);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
    for (let i = -5; i <= 5; i += 1) {
      const worldX = i * gridSpacingMm;
      const a = project({ x: worldX, y: -gridExtentMm, z: -120 }, camera, scale, centerX, centerY);
      const b = project({ x: worldX, y: gridExtentMm, z: -120 }, camera, scale, centerX, centerY);
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

    ctx.fillStyle = "rgba(59,130,246,0.25)";
    ctx.strokeStyle = "#60a5fa";
    ctx.lineWidth = 2;
    ctx.beginPath();
    bodyPts.forEach((p, index) => {
      const screen = project(transformBodyToWorld(p, poseOffset), camera, scale, centerX, centerY);
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
        .map((point) => transformBodyToWorld(point, poseOffset))
        .map((point) => project(point, camera, scale, centerX, centerY));

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
      segments.forEach((point) => {
        ctx.beginPath();
        ctx.arc(point.x, point.y, 3, 0, Math.PI * 2);
        ctx.fill();
      });

      ctx.fillStyle = "#9ca3af";
      ctx.font = "11px sans-serif";
      ctx.fillText(legName, segments[0].x + 5, segments[0].y - 5);
    });

    drawAutonomyDebugOverlay({ ctx, camera, scale, centerX, centerY, model, poseOffset });

    updateTelemetryUI({ statusEl, metaEl, rollingMetricsEl, telemetry, model }, Date.now());
    requestAnimationFrame(drawFrame);
  }

  return {
    start() {
      window.addEventListener("resize", resize);
      resize();
      requestAnimationFrame(drawFrame);
    },
    resize,
  };
}
