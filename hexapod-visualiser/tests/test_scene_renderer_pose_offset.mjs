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

