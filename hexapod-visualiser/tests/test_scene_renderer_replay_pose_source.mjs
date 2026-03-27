import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

import { DEFAULT_ANGLES_DEG, DEFAULT_GEOMETRY } from "../static/constants.js";
import { buildGridProbePoints, resolvePoseOffsetMm } from "../static/scene-renderer.js";
import { applyStatePayload } from "../static/transport.js";

const fixturePath = path.join(
  path.dirname(fileURLToPath(import.meta.url)),
  "fixtures",
  "scene_renderer_replay_pose_source.ndjson",
);

function makeBaseModel() {
  return {
    geometry: { ...DEFAULT_GEOMETRY },
    angles_deg: { ...DEFAULT_ANGLES_DEG },
    timestamp_ms: null,
    active_mode: null,
    active_fault: null,
    bus_ok: null,
    estimator_valid: null,
    loop_counter: null,
    voltage: null,
    current: null,
    dynamic_gait: null,
    autonomy_debug: null,
  };
}

function loadFixture() {
  const raw = readFileSync(fixturePath, "utf8");
  return raw
    .split("\n")
    .map((line) => line.trim())
    .filter(Boolean)
    .map((line) => JSON.parse(line));
}

test("replay datagrams preserves pose-source precedence without dead-reckoning fallback", () => {
  let model = makeBaseModel();
  const telemetry = { lastPayloadAtMs: 0, lastModelTimestampMs: null, rolling: [] };
  const poseState = { pose: { x: 0, y: 0, yaw: 0 }, poseSource: "missing_pose" };

  const replay = [];
  for (const payload of loadFixture()) {
    model = applyStatePayload(model, telemetry, payload, payload.timestamp_ms ?? 0);
    const poseOffset = { ...resolvePoseOffsetMm(model, poseState) };
    const probe = buildGridProbePoints({
      width: 800,
      height: 600,
      scale: 2,
      poseOffset,
      groundPlaneZ: -100,
    });
    replay.push({ label: payload.label, type: payload.type, poseOffset, probe, poseSource: poseState.poseSource });
  }

  const byLabel = Object.fromEntries(replay.map((entry) => [entry.label, entry]));

  assert.equal(byLabel.loc_map.poseOffset.x, 1000, "map localization pose should set the offset");
  assert.equal(byLabel.loc_map.poseSource, "localization_pose");

  assert.equal(byLabel.autonomy_pose.poseOffset.x, 450, "autonomy current_pose should override localization");
  assert.equal(byLabel.autonomy_pose.poseSource, "autonomy_current_pose");

  assert.deepEqual(
    byLabel.missing_pose.poseOffset,
    byLabel.autonomy_pose.poseOffset,
    "missing required pose fields should keep last known offset",
  );
  assert.equal(byLabel.missing_pose.poseSource, "missing_pose");

  assert.notDeepEqual(
    byLabel.loc_map.probe.verticalStart,
    byLabel.autonomy_pose.probe.verticalStart,
    "grid probe should still respond to world pose changes",
  );
});
