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

test("replay datagrams preserves pose-source precedence and directional grid invariant", () => {
  let model = makeBaseModel();
  const telemetry = { lastPayloadAtMs: 0, lastModelTimestampMs: null, rolling: [] };
  const deadReckoning = { pose: { x: 0, y: 0, yaw: 0 }, lastTimestampMs: null };

  const replay = [];
  for (const payload of loadFixture()) {
    model = applyStatePayload(model, telemetry, payload, payload.timestamp_ms ?? 0);
    const poseOffset = { ...resolvePoseOffsetMm(model, deadReckoning, payload.timestamp_ms ?? 0) };
    const probe = buildGridProbePoints({
      width: 800,
      height: 600,
      scale: 2,
      poseOffset,
      groundPlaneZ: -100,
    });
    replay.push({ label: payload.label, type: payload.type, poseOffset, probe });
  }

  if (process.env.DEBUG_SCENE_REPLAY === "1") {
    const bounded = replay.slice(-4).map(({ label, type, poseOffset, probe }) => ({
      label,
      type,
      poseOffset,
      verticalStart: probe.verticalStart,
    }));
    console.log("scene_replay_tail", JSON.stringify(bounded));
  }

  const byLabel = Object.fromEntries(replay.map((entry) => [entry.label, entry]));

  assert.equal(byLabel.loc_map.poseOffset.x, 1000, "map localization pose should override dynamic gait pose");
  assert.equal(byLabel.loc_map.poseOffset.y, 100, "map localization pose should preserve Y pose");

  assert.equal(
    byLabel.unsupported_localization.poseOffset.x,
    450,
    "unsupported localization frame should be ignored in favor of dynamic gait pose",
  );

  assert.ok(
    byLabel.fallback_progress.poseOffset.x > byLabel.fallback_start.poseOffset.x,
    "fallback integration should advance world pose over time",
  );
  assert.notDeepEqual(
    byLabel.fallback_progress.probe.verticalStart,
    byLabel.fallback_start.probe.verticalStart,
    "grid probe should update as dead-reckoned pose changes",
  );

  const toBodyFrame = (worldPoint, poseOffset) => {
    const dx = worldPoint.x - poseOffset.x;
    const dy = worldPoint.y - poseOffset.y;
    const cosYaw = Math.cos(poseOffset.yaw);
    const sinYaw = Math.sin(poseOffset.yaw);
    return {
      x: dx * cosYaw + dy * sinYaw,
      y: -dx * sinYaw + dy * cosYaw,
    };
  };

  const fixedWorldAnchor = { x: 1000, y: 0 };
  const bodyAnchorAtStart = toBodyFrame(fixedWorldAnchor, byLabel.fallback_start.poseOffset);
  const bodyAnchorAtProgress = toBodyFrame(fixedWorldAnchor, byLabel.fallback_progress.poseOffset);

  assert.ok(
    bodyAnchorAtProgress.x < bodyAnchorAtStart.x,
    "forward pose progression should move a fixed world anchor opposite in body frame",
  );
});
