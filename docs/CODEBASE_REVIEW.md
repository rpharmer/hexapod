# Hexapod Codebase Review (March 24, 2026)

This review focuses on architecture, maintainability, testability, and roadmap direction across:

- `hexapod-server` (host runtime/control loop)
- `hexapod-client` (firmware)
- `hexapod-common` (shared protocol)
- `hexapod-visualiser` (UDP telemetry + browser rendering)

> Priority note: this update places extra attention on the visualiser path and the server→visualiser telemetry contract.

## Executive summary

### Strengths
- The repository has a clear product split: host runtime, firmware runtime, and shared protocol/framing primitives.
- Verification entrypoints are straightforward (`scripts/verify.sh`, visualiser smoke, parser tests).
- The visualiser contract is explicitly documented (`docs/VISUALISER_TELEMETRY.md`) with schema versioning and unit rules.

### Top risks / refactor hotspots
1. **Visualiser backend startup path regression risk**: listener wiring and callback signatures in `hexapod-visualiser/server.py` should be simplified and guarded by an integration test for process boot.
2. **Visualiser frontend monolith**: `static/app.js` mixes network state, telemetry freshness logic, camera controls, and rendering in one file.
3. **Large server bridge surface area**: host hardware bridge modules still carry repeated command boilerplate and broad responsibility.
4. **Config parser duplication**: parsing and validation logic repeats common conversion/default/range patterns.
5. **Contract drift risk**: schema docs and producers/consumers are strong today, but there is no automated compatibility matrix for versioned producer/consumer pairs.

---

## Architecture review (cross-cutting)

## 1) Module boundaries are mostly sound
- Current top-level module decomposition is coherent and makes ownership clear.
- Shared protocol types live in `hexapod-common`, reducing accidental divergence between host and firmware.

### Opportunity
- Introduce a short, enforced architecture decision record (ADR) for each cross-component contract (`transport protocol`, `visualiser telemetry`, `config schema`).
- Tie each ADR to a test path so contract changes require both doc + test updates.

## 2) Runtime observability is improving but fragmented
- Diagnostics exist in multiple places (visualiser logs, host telemetry JSON, test suites), but there is no single operator-focused runtime dashboard artifact.

### Opportunity
- Standardize on a small “runtime health schema” for host + visualiser.
- Expose a compact machine-readable endpoint from `hexapod-server` similar to visualiser `/healthz` and consume both in one dev dashboard.

---

## In-depth visualiser review (priority)

## A) Backend (`hexapod-visualiser/server.py`)

### What is good
- Schema-gated UDP parsing prevents silent acceptance of incompatible payloads.
- Partial-merge semantics for geometry and per-leg angles are practical for incremental updates.
- Diagnostics counters (`udp_received`, `udp_rejected`, `ws_send_failures`, etc.) are useful and low-cost.
- Coalescing scheduler is directionally correct for bursty telemetry.

### Refactoring opportunities
1. **Stabilize startup and listener wiring**
   - Keep exactly one UDP listener registration path and one callback flow.
   - Add a dedicated startup test that launches `server.py` and verifies health endpoint + one telemetry round-trip.

2. **Separate concerns in parser path**
   - Split packet handling into pure functions:
     - `decode_json_datagram(bytes) -> Result`
     - `validate_schema(payload) -> Result`
     - `merge_state(state, payload) -> changed`
   - This shrinks `datagram_received()` complexity and improves test precision.

3. **Make rate controls explicit and configurable**
   - `publish_hz` is currently embedded in scheduler setup.
   - Promote this to CLI (`--publish-hz`) and enforce bounds (`1..120`) with clear logs.

4. **Improve broadcast robustness**
   - Handle additional websocket exceptions (`RuntimeError`, closed socket state) and collect failure reasons.
   - Consider per-client backpressure handling (drop stale frames vs queue limit).

5. **Add contract assertions on outbound state**
   - Validate that outbound payload still matches documented browser contract before broadcast in debug mode.

## B) Frontend (`hexapod-visualiser/static/app.js`)

### What is good
- Deterministic camera presets are practical for debugging.
- Freshness UI (`LIVE/STALE`) gives immediate operator feedback.
- Rendering math is understandable and low dependency.

### Refactoring opportunities
1. **Split the monolith into modules**
   - `telemetry_store.js` (payload merge + freshness)
   - `camera_controller.js` (orbit/pan/zoom + presets)
   - `renderer.js` (FK + draw pipeline)
   - `ws_client.js` (reconnect/backoff)

2. **Harden inbound payload sanitization at UI boundary**
   - Even with backend validation, frontend should guard shape/range before mutating render model.

3. **Tune reconnection policy**
   - Replace fixed 1s reconnect with bounded exponential backoff + jitter to reduce thundering-herd behavior.

4. **Add feature flags for debug overlays**
   - Optional overlays: leg workspace circles, foot trajectory traces, frame-time graph.

5. **Rendering performance envelope**
   - Track FPS and draw time moving average; automatically degrade optional overlays when frame budget is exceeded.

## C) Server→Visualiser producer (`hexapod-server/src/control/visualiser_telemetry.cpp`)

### What is good
- Explicit unit conversion (meters→mm, rad→deg) is clean and documented.
- Leg key mapping is deterministic and test-covered.

### Refactoring opportunities
1. **Make geometry source explicit**
   - Current serialization reaches into active geometry singleton; consider injecting geometry context to avoid hidden global coupling.

2. **Schema evolution hooks**
   - Reserve optional metadata envelope for additive fields (`producer_id`, `sequence`, `mode`) while preserving schema v1 compatibility.

3. **Sampling strategy controls**
   - Add configurable publish decimation independent from control loop to avoid unnecessary network load.

---

## Test strategy review

## Strengths
- Server test surface is broad and domain-oriented.
- Visualiser has parser-focused unit tests and a smoke workflow.

## Gaps to close
1. **Visualiser process boot test in CI**
   - Ensure startup wiring regressions are caught immediately.
2. **Contract compatibility tests**
   - Validate `hexapod-server` producer payloads directly against visualiser parser fixtures.
3. **Golden telemetry fixtures**
   - Add versioned fixtures (`schema_v1_nominal`, `schema_v1_partial`, `schema_v1_malformed`) and replay in tests.
4. **Negative testing around timestamps**
   - Include future timestamps, stale timestamps, and non-monotonic updates.

---

## Prioritized execution plan

## Next 1-2 weeks (high ROI)
1. Refactor visualiser backend startup/listener flow and add process-level startup smoke test.
2. Split `app.js` into smaller modules without changing visual behavior.
3. Add CLI knobs for visualiser publish rate and reconnect/backoff settings.

## Next 1-2 months (maintainability)
1. Introduce schema descriptors for config parsing in `hexapod-server`.
2. Continue decomposing bridge command plumbing into reusable helpers.
3. Add compatibility fixture tests between producer and consumer telemetry paths.

## Next quarter (product maturity)
1. Build operator dashboard for unified host + visualiser health.
2. Add session recording/playback UI controls in visualiser (seek, speed, bookmark events).
3. Add terrain/contact debug overlays and gait phase annotations in visualiser.

---

## Feature directions / ideas

## Visualiser-focused ideas
1. **Ghost pose comparison**
   - Overlay “planned” vs “measured” leg poses to diagnose estimator lag and calibration drift.
2. **Footfall timeline strip**
   - Per-leg contact state strip chart for gait debugging.
3. **Safety/fault event markers**
   - Annotate timeline and 3D scene when safety supervisor changes state.
4. **Latency lens**
   - Show transport age, model age, and render age as separate metrics.
5. **Replay notebook mode**
   - Load NDJSON captures and attach operator notes/bookmarks to timestamps.

## Broader platform ideas
1. **Protocol compatibility checker CLI**
   - Validate producer output against schema contracts offline.
2. **Calibration confidence scorecards**
   - Persist quality metrics by joint and show change over time.
3. **Scenario mutation testing**
   - Auto-perturb scenario inputs to discover brittle safety logic.
4. **Release readiness gates**
   - Add performance/error-budget thresholds (loop jitter, command failures, parser rejects).

---

## Suggested objective metrics

Track these continuously and gate releases with thresholds:

- Visualiser UDP accept ratio (`accepted / received`) and rejection reason breakdown.
- WebSocket client stability (connect/disconnect rate, send failure rate).
- End-to-end telemetry latency distribution (`timestamp_ms` to render time).
- Host control-loop p95/p99 jitter under simulation and hardware profiles.
- Scenario suite pass rate and median execution time trend.

These metrics make refactoring impact measurable and keep feature growth aligned with reliability.
