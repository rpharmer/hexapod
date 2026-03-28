import test from "node:test";
import assert from "node:assert/strict";

import {
  resolvePoseOffsetWithSource,
  smoothPoseOffsetForRender,
} from "../static/scene-renderer.js";

function buildModel({ currentPose = null, localizationPose = null, localizationFrameId = "map" } = {}) {
  return {
    autonomy_debug: {
      current_pose: currentPose,
      localization: {
        frame_id: localizationFrameId,
        current_pose: localizationPose,
      },
    },
  };
}

test("resolvePoseOffsetWithSource keeps active source sticky on large alternate jumps", () => {
  const poseState = {
    pose: { x: 0, y: 0, yaw: 0 },
    poseSource: "missing_pose",
    renderPoseState: { pose: null, lastRenderAtMs: Number.NaN },
  };

  const first = resolvePoseOffsetWithSource(
    buildModel({ localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 } }),
    poseState,
    0,
  );
  assert.equal(first.poseSource, "localization_pose");
  assert.equal(first.pose.x, 0);

  const largeDeltaAutonomy = resolvePoseOffsetWithSource(
    buildModel({
      localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 },
      currentPose: { x_m: 1.0, y_m: 0.0, yaw_rad: 0.0 },
    }),
    poseState,
    16,
  );

  assert.equal(largeDeltaAutonomy.poseSource, "localization_pose");
  assert.equal(largeDeltaAutonomy.pose.x, 0);
});

test("resolvePoseOffsetWithSource switches after sustained invalid frames and reseeds render pose", () => {
  const poseState = {
    pose: { x: 0, y: 0, yaw: 0 },
    poseSource: "missing_pose",
    renderPoseState: { pose: null, lastRenderAtMs: Number.NaN },
  };

  resolvePoseOffsetWithSource(
    buildModel({ localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 } }),
    poseState,
    0,
  );

  resolvePoseOffsetWithSource(
    buildModel({ currentPose: { x_m: 1.0, y_m: 0.1, yaw_rad: 0.2 }, localizationPose: null }),
    poseState,
    50,
  );
  resolvePoseOffsetWithSource(
    buildModel({ currentPose: { x_m: 1.0, y_m: 0.1, yaw_rad: 0.2 }, localizationPose: null }),
    poseState,
    100,
  );
  const switched = resolvePoseOffsetWithSource(
    buildModel({ currentPose: { x_m: 1.0, y_m: 0.1, yaw_rad: 0.2 }, localizationPose: null }),
    poseState,
    150,
  );

  assert.equal(switched.poseSource, "autonomy_current_pose");
  assert.deepEqual(switched.pose, { x: 1000, y: 100, yaw: 0.2 });
  assert.deepEqual(
    poseState.renderPoseState.pose,
    switched.pose,
    "switch should reseed render pose to avoid interpolation artifacts",
  );
});

test("alternating availability does not produce large discontinuous smoothed jumps", () => {
  const poseState = {
    pose: { x: 0, y: 0, yaw: 0 },
    poseSource: "missing_pose",
    renderPoseState: { pose: null, lastRenderAtMs: Number.NaN },
  };

  const samples = [
    { nowMs: 0, model: buildModel({ localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 } }) },
    { nowMs: 16, model: buildModel({ localizationPose: null, currentPose: { x_m: 0.9, y_m: 0.0, yaw_rad: 0.0 } }) },
    { nowMs: 32, model: buildModel({ localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 }, currentPose: { x_m: 0.9, y_m: 0.0, yaw_rad: 0.0 } }) },
    { nowMs: 48, model: buildModel({ localizationPose: null, currentPose: { x_m: 0.9, y_m: 0.0, yaw_rad: 0.0 } }) },
    { nowMs: 64, model: buildModel({ localizationPose: { x_m: 0.0, y_m: 0.0, yaw_rad: 0.0 }, currentPose: { x_m: 0.9, y_m: 0.0, yaw_rad: 0.0 } }) },
  ];

  let previousRender = null;
  for (const sample of samples) {
    const resolved = resolvePoseOffsetWithSource(sample.model, poseState, sample.nowMs);
    const renderPose = smoothPoseOffsetForRender({
      targetPose: resolved.pose,
      renderPoseState: poseState.renderPoseState,
      nowMs: sample.nowMs,
    });

    if (previousRender) {
      const jump = Math.hypot(renderPose.x - previousRender.x, renderPose.y - previousRender.y);
      assert.ok(jump < 450, `unexpected discontinuous jump detected: ${jump.toFixed(2)}mm`);
    }
    previousRender = renderPose;
  }
});
