import test from "node:test";
import assert from "node:assert/strict";

import { applyStatePayload } from "../static/transport.js";

function makeModel() {
  return {
    geometry: {},
    angles_deg: {},
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

test("equal timestamp payloads require monotonic sample_id to avoid regression/jitter", () => {
  let model = makeModel();
  const telemetry = { lastPayloadAtMs: 0, lastModelTimestampMs: null, lastModelSampleId: null };

  model = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1000, sample_id: 1, active_mode: "walking", loop_counter: 10 },
    1000,
  );
  assert.equal(model.loop_counter, 10);
  assert.equal(telemetry.lastModelTimestampMs, 1000);
  assert.equal(telemetry.lastModelSampleId, 1);

  const noTiebreaker = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1000, loop_counter: 5, active_mode: "idle" },
    1001,
  );
  assert.equal(noTiebreaker.loop_counter, 10, "equal timestamp without sample_id must be dropped");
  assert.equal(noTiebreaker.active_mode, "walking");

  const nonMonotonicTiebreaker = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1000, sample_id: 1, loop_counter: 6, active_mode: "idle" },
    1002,
  );
  assert.equal(nonMonotonicTiebreaker.loop_counter, 10, "equal timestamp with non-increasing sample_id must be dropped");
  assert.equal(nonMonotonicTiebreaker.active_mode, "walking");

  const monotonicTiebreaker = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1000, sample_id: 2, loop_counter: 11, active_mode: "trotting" },
    1003,
  );
  assert.equal(monotonicTiebreaker.loop_counter, 11);
  assert.equal(monotonicTiebreaker.active_mode, "trotting");
  assert.equal(telemetry.lastModelSampleId, 2);
});

test("strictly increasing timestamps continue to apply normally", () => {
  let model = makeModel();
  const telemetry = { lastPayloadAtMs: 0, lastModelTimestampMs: null, lastModelSampleId: null };

  model = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1000, sample_id: 10, loop_counter: 1, active_mode: "stand" },
    1000,
  );
  model = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1010, loop_counter: 2, active_mode: "walk" },
    1010,
  );
  model = applyStatePayload(
    model,
    telemetry,
    { type: "state", timestamp_ms: 1020, loop_counter: 3, active_mode: "trot" },
    1020,
  );

  assert.equal(model.loop_counter, 3);
  assert.equal(model.active_mode, "trot");
  assert.equal(telemetry.lastModelTimestampMs, 1020);
});
