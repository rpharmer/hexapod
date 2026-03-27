import test from "node:test";
import assert from "node:assert/strict";

import { resolvePoseOffsetMm } from "../static/scene-renderer.js";

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
