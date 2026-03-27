import test from "node:test";
import assert from "node:assert/strict";

import { buildGridProbePoints, resolvePoseOffsetMm } from "../static/scene-renderer.js";

function baseModel(overrides = {}) {
  return {
    timestamp_ms: 1000,
    autonomy_debug: null,
    dynamic_gait: null,
    ...overrides,
  };
}

test("resolvePoseOffsetMm accepts x/y pose fields from dynamic gait current_pose", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    dynamic_gait: {
      current_pose: { x: 1.25, y: -0.5, yaw: 0.75 },
    },
  });

  const pose = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(pose, { x: 1250, y: -500, yaw: 0.75 });
});

test("resolvePoseOffsetMm accepts x/y pose fields from autonomy current_pose", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    autonomy_debug: {
      current_pose: { x: -0.3, y: 0.2, z: -0.4 },
    },
  });

  const pose = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(pose, { x: -300, y: 200, yaw: -0.4 });
});

test("resolvePoseOffsetMm ignores unsupported localization frame ids", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    autonomy_debug: {
      localization: {
        frame_id: "base_link",
        current_pose: { x_m: 2.0, y_m: 3.0, yaw_rad: 0.25 },
      },
    },
    dynamic_gait: {
      current_pose: { x_m: 0.4, y_m: -0.2, yaw_rad: 0.1 },
    },
  });

  const pose = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(pose, { x: 400, y: -200, yaw: 0.1 });
  assert.equal(deadReckoning.poseSource, "dynamic_gait_current_pose");
});

test("resolvePoseOffsetMm dead reckons with wall clock when telemetry timestamp is missing", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    timestamp_ms: null,
    dynamic_gait: {
      body_linear_velocity_mps: { x: 1.0, y: 0.0, z: 0.0 },
      body_angular_velocity_radps: { x: 0.0, y: 0.0, z: 0.0 },
    },
  });

  const poseA = resolvePoseOffsetMm(model, deadReckoning, 1_000);
  assert.deepEqual(poseA, { x: 0, y: 0, yaw: 0 });

  const poseB = resolvePoseOffsetMm(model, deadReckoning, 1_500);
  assert.deepEqual(poseB, { x: 250, y: 0, yaw: 0 });
});

test("resolvePoseOffsetMm prefers velocity integration when only static body_translation is available", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    timestamp_ms: 1_000,
    dynamic_gait: {
      body_translation_m: { x: 0, y: 0, z: 0 },
      body_orientation_rad: { x: 0, y: 0, z: 0 },
      body_linear_velocity_mps: { x: 0.2, y: 0.0, z: 0.0 },
      body_angular_velocity_radps: { x: 0.0, y: 0.0, z: 0.0 },
    },
  });

  const poseA = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(poseA, { x: 0, y: 0, yaw: 0 });

  const poseB = resolvePoseOffsetMm({ ...model, timestamp_ms: 1_500 }, deadReckoning);
  assert.deepEqual(poseB, { x: 50, y: 0, yaw: 0 });
});

test("resolvePoseOffsetMm falls back to commanded body velocity when estimated XY velocity is absent", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    timestamp_ms: 1_000,
    dynamic_gait: {
      body_translation_m: { x: 0, y: 0, z: 0 },
      body_linear_velocity_mps: { x: 0, y: 0, z: 0 },
      body_angular_velocity_radps: { x: 0, y: 0, z: 0 },
      commanded_body_velocity_mps: { x: 0.3, y: 0, z: 0 },
      commanded_body_angular_velocity_radps: { x: 0, y: 0, z: 0 },
    },
  });

  const poseA = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(poseA, { x: 0, y: 0, yaw: 0 });

  const poseB = resolvePoseOffsetMm({ ...model, timestamp_ms: 1_500 }, deadReckoning);
  assert.deepEqual(poseB, { x: 75, y: 0, yaw: 0 });
});

test("resolvePoseOffsetMm falls back to commanded speed+heading when no XY velocity vectors are present", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const model = baseModel({
    timestamp_ms: 2_000,
    dynamic_gait: {
      body_translation_m: { x: 0, y: 0, z: 0 },
      body_linear_velocity_mps: { x: 0, y: 0, z: 0 },
      body_angular_velocity_radps: { x: 0, y: 0, z: 0 },
      commanded_speed_mps: 0.2,
      commanded_heading_rad: Math.PI / 2,
    },
  });

  const poseA = resolvePoseOffsetMm(model, deadReckoning);
  assert.deepEqual(poseA, { x: 0, y: 0, yaw: 0 });

  const poseB = resolvePoseOffsetMm({ ...model, timestamp_ms: 2_500 }, deadReckoning);
  assert.ok(Math.abs(poseB.x) < 0.001);
  assert.equal(Math.round(poseB.y), 50);
});

test("resolvePoseOffsetMm integrates straight-line odometry over multiple telemetry frames", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const makeFrame = (timestampMs) => baseModel({
    timestamp_ms: timestampMs,
    dynamic_gait: {
      body_linear_velocity_mps: { x: 0.4, y: 0.0, z: 0.0 },
      body_angular_velocity_radps: { x: 0.0, y: 0.0, z: 0.0 },
    },
  });

  const p0 = { ...resolvePoseOffsetMm(makeFrame(2_000), deadReckoning) };
  const p1 = { ...resolvePoseOffsetMm(makeFrame(2_100), deadReckoning) };
  const p2 = { ...resolvePoseOffsetMm(makeFrame(2_250), deadReckoning) };

  assert.deepEqual(p0, { x: 0, y: 0, yaw: 0 });
  assert.equal(Math.round(p1.x), 40);
  assert.equal(Math.round(p2.x), 100);
  assert.equal(Math.round(p2.y), 0);
  assert.equal(p2.yaw, 0);
});

test("resolvePoseOffsetMm applies body twist while translating in world frame", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: Math.PI / 2 }, lastTimestampMs: 10_000 };
  const model = baseModel({
    timestamp_ms: 10_200,
    dynamic_gait: {
      body_linear_velocity_mps: { x: 0.3, y: 0.2, z: 0.0 },
      body_angular_velocity_radps: { x: 0.0, y: 0.0, z: 0.5 },
    },
  });

  const pose = resolvePoseOffsetMm(model, deadReckoning);

  // At yaw=pi/2, body +x maps to world +y and body +y maps to world -x.
  assert.equal(Math.round(pose.x), -40);
  assert.equal(Math.round(pose.y), 60);
  assert.ok(Math.abs(pose.yaw - (Math.PI / 2 + 0.1)) < 1e-9);
});

test("resolvePoseOffsetMm tracks waypoint progress from absolute autonomy poses in map frame", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const waypoints = [
    { x_m: 0.5, y_m: 0.0, yaw_rad: 0.0 },
    { x_m: 1.0, y_m: 0.4, yaw_rad: 0.5 },
  ];
  const makeFrame = (timestampMs, x, y, yaw, activeIdx) => baseModel({
    timestamp_ms: timestampMs,
    autonomy_debug: {
      active_waypoint_index: activeIdx,
      waypoints,
      localization: {
        frame_id: "map",
        current_pose: { x_m: x, y_m: y, yaw_rad: yaw },
      },
    },
  });

  const poseA = resolvePoseOffsetMm(makeFrame(5_000, 0.1, 0.0, 0.0, 0), deadReckoning);
  const poseB = resolvePoseOffsetMm(makeFrame(5_300, 0.52, 0.01, 0.1, 1), deadReckoning);
  const poseC = resolvePoseOffsetMm(makeFrame(5_600, 1.0, 0.4, 0.5, 2), deadReckoning);

  assert.deepEqual(poseA, { x: 100, y: 0, yaw: 0 });
  assert.deepEqual(poseB, { x: 520, y: 10, yaw: 0.1 });
  assert.deepEqual(poseC, { x: 1000, y: 400, yaw: 0.5 });
});

test("resolvePoseOffsetMm accepts odom localization poses while missions advance", () => {
  const deadReckoning = { pose: { x: -200, y: 300, yaw: -0.2 }, lastTimestampMs: 7_000 };
  const model = baseModel({
    timestamp_ms: 7_200,
    autonomy_debug: {
      localization: {
        frame_id: "odom",
        current_pose: { x_m: -0.05, y_m: 0.12, yaw_rad: -0.35 },
      },
      active_waypoint_index: 1,
      waypoints: [
        { x_m: -0.1, y_m: 0.1, yaw_rad: -0.3 },
        { x_m: 0.2, y_m: 0.2, yaw_rad: 0.0 },
      ],
    },
  });

  const pose = resolvePoseOffsetMm(model, deadReckoning);

  assert.deepEqual(pose, { x: -50, y: 120, yaw: -0.35 });
  assert.equal(deadReckoning.lastTimestampMs, 7_200);
});

test("buildGridProbePoints shifts rendered grid anchors as pose offset changes", () => {
  const width = 800;
  const height = 600;
  const scale = 2;
  const poseA = { x: 0, y: 0, yaw: 0 };
  const poseB = { x: 18, y: -12, yaw: 0.15 };

  const probeA = buildGridProbePoints({ width, height, scale, poseOffset: poseA, groundPlaneZ: -100 });
  const probeB = buildGridProbePoints({ width, height, scale, poseOffset: poseB, groundPlaneZ: -100 });

  // Helpful diagnostic dump for investigation/reproduction logs.
  console.log("grid_probe_frame_a", JSON.stringify(probeA));
  console.log("grid_probe_frame_b", JSON.stringify(probeB));

  assert.notDeepEqual(probeA.horizontalStart, probeB.horizontalStart);
  assert.notDeepEqual(probeA.verticalStart, probeB.verticalStart);
});

test("grid probe dumps show pose increases under forward command while rendered world anchors move opposite", () => {
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };
  const makeFrame = (timestampMs) => baseModel({
    timestamp_ms: timestampMs,
    dynamic_gait: {
      body_translation_m: { x: 0, y: 0, z: 0 },
      body_linear_velocity_mps: { x: 0, y: 0, z: 0 },
      body_angular_velocity_radps: { x: 0, y: 0, z: 0 },
      commanded_speed_mps: 0.15,
      commanded_heading_rad: 0,
    },
  });

  const dump = [];
  for (const timestampMs of [1_000, 1_250, 1_500, 1_750]) {
    const poseOffset = { ...resolvePoseOffsetMm(makeFrame(timestampMs), deadReckoning) };
    const probe = buildGridProbePoints({
      width: 800,
      height: 600,
      scale: 2,
      poseOffset,
      groundPlaneZ: -100,
    });
    dump.push({ timestampMs, poseOffset, probe });
  }

  // Helpful reproduction dump requested during observability diagnosis.
  console.log("grid_probe_time_series", JSON.stringify(dump));

  assert.ok(dump[3].poseOffset.x > dump[0].poseOffset.x, "forward command should increase dead-reckoned world pose x");
  assert.ok(
    dump[3].probe.verticalStart.x < dump[0].probe.verticalStart.x,
    "world-anchored grid x should move opposite the forward pose direction in body frame",
  );
});
