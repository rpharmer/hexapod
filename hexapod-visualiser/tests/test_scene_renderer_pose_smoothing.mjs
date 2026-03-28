import test from "node:test";
import assert from "node:assert/strict";

import { smoothPoseOffsetForRender } from "../static/scene-renderer.js";

test("smoothPoseOffsetForRender eases translation deltas across frames", () => {
  const renderPoseState = { pose: null, lastRenderAtMs: Number.NaN };
  const first = smoothPoseOffsetForRender({
    targetPose: { x: 0, y: 0, yaw: 0 },
    renderPoseState,
    nowMs: 0,
  });
  assert.deepEqual(first, { x: 0, y: 0, yaw: 0 });

  const second = smoothPoseOffsetForRender({
    targetPose: { x: 1000, y: -500, yaw: 0.5 },
    renderPoseState,
    nowMs: 16,
  });

  assert.ok(second.x > 0 && second.x < 1000, "x should move toward target without snapping");
  assert.ok(second.y < 0 && second.y > -500, "y should move toward target without snapping");
  assert.ok(second.yaw > 0 && second.yaw < 0.5, "yaw should move toward target without snapping");
});

test("smoothPoseOffsetForRender uses shortest yaw path across angle wrap", () => {
  const renderPoseState = { pose: { x: 0, y: 0, yaw: 3.12 }, lastRenderAtMs: 0 };

  const next = smoothPoseOffsetForRender({
    targetPose: { x: 0, y: 0, yaw: -3.12 },
    renderPoseState,
    nowMs: 16,
  });

  assert.ok(next.yaw > 3.0 || next.yaw < -3.0, "yaw should stay near the wrap boundary");
});
