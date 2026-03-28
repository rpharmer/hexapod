import test from "node:test";
import assert from "node:assert/strict";

import {
  buildGridProbePoints,
  resolveCurrentPoseHeadingRad,
  resolvePoseOffsetMm,
} from "../static/scene-renderer.js";

function baseModel(overrides = {}) {
  return {
    timestamp_ms: 1000,
    autonomy_debug: null,
    dynamic_gait: null,
    ...overrides,
  };
}

test("resolvePoseOffsetMm uses autonomy_debug.current_pose when provided", () => {
  const poseState = { pose: { x: 0, y: 0, yaw: 0 }, poseSource: "missing_pose" };
  const model = baseModel({
    autonomy_debug: {
      current_pose: { x_m: 1.2, y_m: -0.4, yaw_rad: 0.75 },
    },
  });

  const pose = resolvePoseOffsetMm(model, poseState);
  assert.deepEqual(pose, { x: 1200, y: -400, yaw: 0.75 });
  assert.equal(poseState.poseSource, "autonomy_current_pose");
});

test("resolvePoseOffsetMm uses localization pose only for map/odom frames", () => {
  const poseState = { pose: { x: 100, y: 200, yaw: -0.2 }, poseSource: "seed" };

  const ignored = resolvePoseOffsetMm(
    baseModel({
      autonomy_debug: {
        localization: {
          frame_id: "base_link",
          current_pose: { x_m: 9.0, y_m: 9.0, yaw_rad: 1.0 },
        },
      },
    }),
    poseState,
  );
  assert.deepEqual(ignored, { x: 100, y: 200, yaw: -0.2 });
  assert.equal(poseState.poseSource, "missing_pose");

  const accepted = resolvePoseOffsetMm(
    baseModel({
      autonomy_debug: {
        localization: {
          frame_id: "map",
          current_pose: { x_m: 0.5, y_m: 0.1, yaw_rad: 0.3 },
        },
      },
    }),
    poseState,
  );
  assert.deepEqual(accepted, { x: 500, y: 100, yaw: 0.3 });
  assert.equal(poseState.poseSource, "localization_pose");
});

test("resolvePoseOffsetMm does not dead-reckon from velocity-only payloads", () => {
  const poseState = { pose: { x: 42, y: -24, yaw: 0.1 }, poseSource: "seed" };
  const pose = resolvePoseOffsetMm(
    baseModel({
      dynamic_gait: {
        body_linear_velocity_mps: { x: 0.5, y: 0.0, z: 0.0 },
        body_angular_velocity_radps: { x: 0.0, y: 0.0, z: 0.2 },
      },
    }),
    poseState,
  );

  assert.deepEqual(pose, { x: 42, y: -24, yaw: 0.1 });
  assert.equal(poseState.poseSource, "missing_pose");
});

test("buildGridProbePoints still shifts world anchors as pose offset changes", () => {
  const probeA = buildGridProbePoints({
    width: 800,
    height: 600,
    scale: 2,
    poseOffset: { x: 0, y: 0, yaw: 0 },
    groundPlaneZ: -100,
  });
  const probeB = buildGridProbePoints({
    width: 800,
    height: 600,
    scale: 2,
    poseOffset: { x: 25, y: -10, yaw: 0.2 },
    groundPlaneZ: -100,
  });

  assert.notDeepEqual(probeA.horizontalStart, probeB.horizontalStart);
  assert.notDeepEqual(probeA.verticalStart, probeB.verticalStart);
});

test("resolveCurrentPoseHeadingRad aligns nose direction with waypoint frame transform", () => {
  const poseOffset = { x: 0, y: 0, yaw: 0.7 };
  const headingA = resolveCurrentPoseHeadingRad({ current_pose: { yaw_rad: 0.7 } }, poseOffset);
  const headingB = resolveCurrentPoseHeadingRad({ current_pose: { yaw_rad: 1.2 } }, poseOffset);

  assert.equal(headingA, 0);
  assert.equal(headingB, 0.5);

  const noseA = { x: Math.cos(headingA), y: Math.sin(headingA) };
  const noseB = { x: Math.cos(headingB), y: Math.sin(headingB) };
  assert.ok(noseB.y > noseA.y, "nose direction should change as yaw changes");
  assert.equal(headingB, 1.2 - poseOffset.yaw, "must use same yaw - poseOffset.yaw convention as waypoint arrows");
});

test("resolveCurrentPoseHeadingRad falls back to zero when current pose yaw is invalid", () => {
  assert.equal(resolveCurrentPoseHeadingRad({ current_pose: { yaw_rad: Number.NaN } }, { yaw: 2.0 }), 0);
  assert.equal(resolveCurrentPoseHeadingRad({ current_pose: {} }, { yaw: 2.0 }), 0);
  assert.equal(resolveCurrentPoseHeadingRad(null, { yaw: 2.0 }), 0);
});
