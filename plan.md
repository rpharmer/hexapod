## Detailed Plan

### Progress snapshot (updated March 23, 2026)

- ✅ **Phase 1:** Substantially complete in current tree (shared bridge transaction/decode helpers, freshness gate extraction, focused freshness edge tests present in `hexapod-server/tests`).
- 🔄 **Phase 2:** In progress.
  - ✅ Route-wrapper boilerplate reduction completed.
  - ✅ Shared decode/NACK helper path applied across motion/sensing/power handlers.
  - ⚠️ Host-facing malformed-payload checks expanded; server-side CTest integration exists, but equivalent client-side automation path still needs wiring.
- 🔄 **Phase 3:** Partially started.
  - ✅ Shared command metadata source now exists in `hexapod-common`.
  - ✅ Host command-name lookup now uses shared metadata-driven mapping.
  - ✅ Firmware route payload policies now derive from shared metadata contracts.
  - ✅ Added compile-time/runtime drift checks for firmware-dispatch coverage and payload-contract alignment.
  - ⚠️ Additional end-to-end runtime coverage across all command paths is still needed.
- ⏳ **Phase 4:** Not yet started (design spike + staged implementation still pending).

### Remaining priority points to address next

1. **Phase 2.3 test execution path**
   - Ensure host-facing malformed payload checks run in CI/build defaults (not only via ad-hoc execution).
2. **Phase 3.3 consistency validation depth**
   - Expand beyond metadata/dispatch contract checks into end-to-end transport-path assertions for every command category.
3. **Phase 4 design spike + safety workflow**
   - Implement guided probe mode skeleton, telemetry schema extensions, and abort interlocks before fitting/calibration rollout.

### Phase 1 — Core maintainability and safety logic hardening (Week 1)

1. **Refactor server command transaction/decode duplication**
   - Target: `hexapod-server` hardware bridge layer.
   - Consolidate repeated request/ACK/decode patterns into shared typed helpers.
   - Standardize error outcome handling and logging fields across all command methods.
   - Migrate high-frequency query commands first (current, voltage, sensor, calibrations), then remaining command methods.

2. **Decompose runtime control gating**
   - Target: `RobotRuntime::controlStep`.
   - Extract pure decision helpers for:
     - freshness gate evaluation,
     - stale-status synthesis,
     - nominal-vs-safe-idle decision.
   - Keep orchestration side effects (state write-back, command emission, diagnostics) in the top-level control step.

3. **Expand focused test coverage for gate edge cases**
   - Add/adjust unit tests for stale estimator, stale intent, and combined stale conditions.
   - Verify safe-idle fallback and status precedence behaviors remain unchanged.

4. **Phase-1 acceptance criteria**
   - Existing host tests stay green.
   - No protocol behavior changes.
   - Control-step code path is simpler and easier to unit test.

---

### Phase 2 — Firmware command dispatch cleanup (Week 2)

1. **Reduce route wrapper boilerplate**
   - Target: `hexapod-client/command_dispatch.cpp`.
   - Replace thin forwarding wrappers with direct bindings or minimal reusable adapters.
   - Preserve declarative route table style and payload policy enforcement.

2. **Unify decode and NACK behavior**
   - Add shared decode/validate utility used by motion, sensing, and power handlers.
   - Ensure consistent error responses for invalid payload lengths and out-of-range arguments.

3. **Strengthen host-facing behavior checks**
   - Add/extend tests for malformed payload lengths and invalid indices.
   - Confirm command response semantics are unchanged for valid requests.

4. **Phase-2 acceptance criteria**
   - Reduced firmware dispatch duplication.
   - Uniform invalid-payload handling across command domains.
   - No regressions in host-firmware command compatibility.

---

### Phase 3 — Shared protocol metadata unification (Week 3)

1. **Create a single command metadata source**
   - Target: `hexapod-common`.
   - Define authoritative command metadata: ID, canonical name, payload contract/policy.

2. **Migrate host and firmware consumers**
   - Replace host command-name lookup/switch logic with shared metadata-driven mapping.
   - Reuse metadata in firmware route declarations where practical.

3. **Add consistency validation**
   - Introduce tests/checks to detect drift between IDs, names, and payload contracts.

4. **Phase-3 acceptance criteria**
   - One authoritative command metadata source used by both sides.
   - New command additions require fewer synchronized edits.
   - Drift is caught automatically by tests.

---

### Phase 4 — Calibration and servo-dynamics probe (Design spike + staged implementation)

1. **Design and implement guided flat-surface probe mode**
   - Add host-driven calibration routine with one-leg-at-a-time probing and stable support polygon.
   - Capture structured probe segments (`approach -> touchdown -> hold -> unload`).

2. **Enhance telemetry schema**
   - Record command timestamps, command/measured joint trajectories, contact edge timing, estimator pose/velocity context.
   - Persist probe runs in analysis-friendly format.

3. **Fit calibration corrections from contact residuals**
   - Use ground-plane touchdown residual minimization with robust outlier rejection.
   - Weight samples by contact confidence and persistence.
   - Produce review-first calibration delta reports before applying changes.

4. **Identify servo speed dynamics**
   - Fit per-joint dynamic parameters (e.g., response time constant, speed limit), including load-direction effects.
   - Compute touchdown prediction residuals to flag lag/friction/asymmetry issues.

5. **Define destination/contact completion logic**
   - For descending probes, require both geometric proximity and debounced contact for “reached”.
   - Classify early-contact and no-contact timeout as distinct outcomes for diagnostics and fitting quality.

6. **Add safety interlocks and validation flow**
   - Abort on over-current, timeout, contact anomalies, or IK infeasibility; return to neutral safely.
   - Validate in simulation with injected offsets/noise/latency, then repeatability checks on hardware.

7. **Phase-4 acceptance criteria**
   - Probe workflow is safe, deterministic, and operator-reviewable.
   - Calibration proposals and servo dynamics reports are reproducible.
   - Edge conditions (early/no contact, abort) are covered by scenarios/tests.

---

### Ongoing (all phases)

1. Run simulator scenario sweeps before hardware trials.
2. Track command-failure and freshness-stale counters over longer runs.
3. Keep docs synchronized with each completed phase.
4. Use conservative hardware bring-up (low amplitude, unloaded, supervised power cutoff).

---

### Suggested execution timeline

- **Week 1:** Phase 1 complete.
- **Week 2:** Phase 2 complete.
- **Week 3:** Phase 3 complete.
- **Week 4+:** Phase 4 design spike, then incremental implementation and validation.
