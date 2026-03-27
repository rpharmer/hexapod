import {
  LEG_ORDER,
  MOUNT_ANGLES,
  STALE_AFTER_MS,
  ROLLING_LIMIT,
  deg2rad,
} from "./constants.js";

const RIGHT_SIDE_LEGS = new Set(["RF", "RM", "RR"]);

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

  const sideSign = RIGHT_SIDE_LEGS.has(legName) ? -1 : 1;
  const femurPitch = deg2rad(femurDeg * sideSign);
  const tibiaPitch = deg2rad(tibiaDeg * sideSign);

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

function normalizePoseCandidate(candidate) {
  if (!candidate) {
    return null;
  }
  const x = Number.isFinite(candidate.x_m) ? candidate.x_m : candidate.x;
  const y = Number.isFinite(candidate.y_m) ? candidate.y_m : candidate.y;
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return null;
  }

  const yaw = Number.isFinite(candidate.yaw_rad)
    ? candidate.yaw_rad
    : Number.isFinite(candidate.yaw)
      ? candidate.yaw
      : candidate.z;

  return { x_m: x, y_m: y, yaw_rad: Number.isFinite(yaw) ? yaw : 0 };
}

function resolveFinitePoseCandidate(...candidates) {
  for (const candidate of candidates) {
    const normalized = normalizePoseCandidate(candidate);
    if (normalized) {
      return normalized;
    }
  }
  return null;
}

function resolvePoseCandidateFromTranslation(translation, orientation) {
  if (!translation) {
    return null;
  }
  const candidate = normalizePoseCandidate(translation);
  if (!candidate) {
    return null;
  }
  const yaw = Number.isFinite(orientation?.yaw_rad)
    ? orientation.yaw_rad
    : Number.isFinite(orientation?.yaw)
      ? orientation.yaw
      : orientation?.z;
  return { x_m: candidate.x_m, y_m: candidate.y_m, yaw_rad: Number.isFinite(yaw) ? yaw : candidate.yaw_rad };
}

function resolveVelocityTwist(model) {
  const linear = model.dynamic_gait?.body_linear_velocity_mps;
  const angular = model.dynamic_gait?.body_angular_velocity_radps;
  if (!linear && !angular) {
    return null;
  }
  return {
    vx: Number.isFinite(linear?.x) ? linear.x : 0,
    vy: Number.isFinite(linear?.y) ? linear.y : 0,
    wz: Number.isFinite(angular?.z) ? angular.z : 0,
  };
}

export function resolvePoseOffsetMm(model, deadReckoning, nowMs = Date.now()) {
  const absolutePose = resolveFinitePoseCandidate(
    model.autonomy_debug?.current_pose,
    model.autonomy_debug?.localization?.current_pose,
    model.dynamic_gait?.current_pose,
  );
  if (absolutePose) {
    const resolved = {
      x: absolutePose.x_m * 1000,
      y: absolutePose.y_m * 1000,
      yaw: Number.isFinite(absolutePose.yaw_rad) ? absolutePose.yaw_rad : 0,
    };
    deadReckoning.pose = { ...resolved };
    deadReckoning.lastTimestampMs = Number.isFinite(model.timestamp_ms)
      ? model.timestamp_ms
      : (Number.isFinite(nowMs) ? nowMs : null);
    return resolved;
  }

  const translationPose = resolveFinitePoseCandidate(
    resolvePoseCandidateFromTranslation(
      model.dynamic_gait?.body_translation_m,
      model.dynamic_gait?.body_orientation_rad,
    ),
    resolvePoseCandidateFromTranslation(
      model.dynamic_gait?.body_translation_m,
      model.dynamic_gait?.orientation_rad,
    ),
  );
  const velocity = resolveVelocityTwist(model);
  const timestampMs = Number.isFinite(model.timestamp_ms)
    ? model.timestamp_ms
    : (Number.isFinite(nowMs) ? nowMs : null);
  if (velocity && timestampMs !== null) {
    if (deadReckoning.lastTimestampMs === null && translationPose) {
      deadReckoning.pose = {
        x: translationPose.x_m * 1000,
        y: translationPose.y_m * 1000,
        yaw: Number.isFinite(translationPose.yaw_rad) ? translationPose.yaw_rad : deadReckoning.pose.yaw,
      };
    }

    if (deadReckoning.lastTimestampMs !== null) {
      const dtSeconds = Math.max(0, Math.min((timestampMs - deadReckoning.lastTimestampMs) / 1000, 0.25));
      if (dtSeconds > 0) {
        const yaw = deadReckoning.pose.yaw;
        const worldVx = velocity.vx * Math.cos(yaw) - velocity.vy * Math.sin(yaw);
        const worldVy = velocity.vx * Math.sin(yaw) + velocity.vy * Math.cos(yaw);
        deadReckoning.pose.x += worldVx * dtSeconds * 1000;
        deadReckoning.pose.y += worldVy * dtSeconds * 1000;
        deadReckoning.pose.yaw += velocity.wz * dtSeconds;
      }
    }
    deadReckoning.lastTimestampMs = timestampMs;
    return deadReckoning.pose;
  }

  if (translationPose) {
    const translated = {
      x: translationPose.x_m * 1000,
      y: translationPose.y_m * 1000,
      yaw: Number.isFinite(translationPose.yaw_rad) ? translationPose.yaw_rad : 0,
    };
    deadReckoning.pose = { ...translated };
    deadReckoning.lastTimestampMs = timestampMs;
    return translated;
  }

  return deadReckoning.pose;
}

function transformWorldToBodyAnchored(point, poseOffset) {
  const dx = point.x - poseOffset.x;
  const dy = point.y - poseOffset.y;
  const cosYaw = Math.cos(poseOffset.yaw);
  const sinYaw = Math.sin(poseOffset.yaw);
  return {
    x: dx * cosYaw + dy * sinYaw,
    y: -dx * sinYaw + dy * cosYaw,
    z: point.z,
  };
}

export function buildGridProbePoints({
  width,
  height,
  scale,
  poseOffset,
  gridSpacingMm = 55,
  groundPlaneZ = -120,
}) {
  const visibleRadiusMm = Math.max(width, height) / Math.max(scale, 1e-6);
  const gridExtentMm = visibleRadiusMm * 1.2;
  const minWorldX = poseOffset.x - gridExtentMm;
  const maxWorldX = poseOffset.x + gridExtentMm;
  const minWorldY = poseOffset.y - gridExtentMm;
  const maxWorldY = poseOffset.y + gridExtentMm;
  const firstWorldY = Math.floor(minWorldY / gridSpacingMm) * gridSpacingMm;
  const firstWorldX = Math.floor(minWorldX / gridSpacingMm) * gridSpacingMm;

  const horizontalStart = transformWorldToBodyAnchored(
    { x: minWorldX, y: firstWorldY, z: groundPlaneZ },
    poseOffset,
  );
  const horizontalEnd = transformWorldToBodyAnchored(
    { x: maxWorldX, y: firstWorldY, z: groundPlaneZ },
    poseOffset,
  );
  const verticalStart = transformWorldToBodyAnchored(
    { x: firstWorldX, y: minWorldY, z: groundPlaneZ },
    poseOffset,
  );
  const verticalEnd = transformWorldToBodyAnchored(
    { x: firstWorldX, y: maxWorldY, z: groundPlaneZ },
    poseOffset,
  );
  return { horizontalStart, horizontalEnd, verticalStart, verticalEnd };
}

function drawAutonomyDebugOverlay({ ctx, camera, scale, centerX, centerY, model, poseOffset, groundPlaneZ }) {
  const autonomy = model.autonomy_debug;
  if (!autonomy || !Array.isArray(autonomy.waypoints) || autonomy.waypoints.length === 0) {
    return;
  }

  const waypointPoints = autonomy.waypoints
    .filter((waypoint) => Number.isFinite(waypoint?.x_m) && Number.isFinite(waypoint?.y_m))
    .map((waypoint) => ({
      x: waypoint.x_m * 1000,
      y: waypoint.y_m * 1000,
      z: groundPlaneZ,
    }))
    .map((point) => transformWorldToBodyAnchored(point, poseOffset));

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
        x: point.x + Math.cos(yawRad - poseOffset.yaw) * 22,
        y: point.y + Math.sin(yawRad - poseOffset.yaw) * 22,
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
    const posePoint = { x: 0, y: 0, z: groundPlaneZ };
    const heading = 0;
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
  let stableGroundPlaneZ = Number.NaN;
  const deadReckoning = {
    pose: { x: 0, y: 0, yaw: 0 },
    lastTimestampMs: null,
  };

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
    const poseOffset = resolvePoseOffsetMm(model, deadReckoning, Date.now());
    const gridSpacingMm = 55;
    const kinematics = LEG_ORDER.map((legName) => ({
      legName,
      points: fkForLeg(
        legName,
        model.angles_deg[legName] ?? [0, 0, 0],
        model.geometry,
      ),
    }));
    const footHeights = kinematics.map((entry) => entry.points.foot.z).filter(Number.isFinite);
    if (footHeights.length > 0 && !Number.isFinite(stableGroundPlaneZ)) {
      stableGroundPlaneZ = Math.min(...footHeights);
    }
    const groundPlaneZ = Number.isFinite(stableGroundPlaneZ) ? stableGroundPlaneZ : -120;

    ctx.strokeStyle = "#1f2937";
    ctx.lineWidth = 1;
    const visibleRadiusMm = Math.max(width, height) / Math.max(scale, 1e-6);
    const gridExtentMm = visibleRadiusMm * 1.2;
    const minWorldX = poseOffset.x - gridExtentMm;
    const maxWorldX = poseOffset.x + gridExtentMm;
    const minWorldY = poseOffset.y - gridExtentMm;
    const maxWorldY = poseOffset.y + gridExtentMm;
    const firstWorldY = Math.floor(minWorldY / gridSpacingMm) * gridSpacingMm;
    const firstWorldX = Math.floor(minWorldX / gridSpacingMm) * gridSpacingMm;

    for (let worldY = firstWorldY; worldY <= maxWorldY + gridSpacingMm; worldY += gridSpacingMm) {
      const a = transformWorldToBodyAnchored({ x: minWorldX, y: worldY, z: groundPlaneZ }, poseOffset);
      const b = transformWorldToBodyAnchored({ x: maxWorldX, y: worldY, z: groundPlaneZ }, poseOffset);
      const ap = project(a, camera, scale, centerX, centerY);
      const bp = project(b, camera, scale, centerX, centerY);
      ctx.beginPath();
      ctx.moveTo(ap.x, ap.y);
      ctx.lineTo(bp.x, bp.y);
      ctx.stroke();
    }

    for (let worldX = firstWorldX; worldX <= maxWorldX + gridSpacingMm; worldX += gridSpacingMm) {
      const a = transformWorldToBodyAnchored({ x: worldX, y: minWorldY, z: groundPlaneZ }, poseOffset);
      const b = transformWorldToBodyAnchored({ x: worldX, y: maxWorldY, z: groundPlaneZ }, poseOffset);
      const ap = project(a, camera, scale, centerX, centerY);
      const bp = project(b, camera, scale, centerX, centerY);
      ctx.beginPath();
      ctx.moveTo(ap.x, ap.y);
      ctx.lineTo(bp.x, bp.y);
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
      const screen = project(p, camera, scale, centerX, centerY);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    kinematics.forEach(({ legName, points }) => {

      const segments = [points.anchor, points.shoulder, points.knee, points.foot]
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

    drawAutonomyDebugOverlay({ ctx, camera, scale, centerX, centerY, model, poseOffset, groundPlaneZ });

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
