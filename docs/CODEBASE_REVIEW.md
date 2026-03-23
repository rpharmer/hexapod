# Codebase Review (March 23, 2026)

## Executive summary

The repository is in a healthy state overall: architecture boundaries are clear, protocol ownership is centralized in `hexapod-common`, and host-side tests are extensive and currently green.

The strongest improvement opportunities are not correctness bugs, but **maintainability and scaling risks**:

1. **Server transport API duplication** in `SimpleHardwareBridge`.
2. **Control-step responsibility overload** in `RobotRuntime::controlStep`.
3. **Firmware dispatch wrapper boilerplate** in `command_dispatch.cpp`.
4. **Protocol metadata duplication** across host/firmware command-name and payload policies.
5. **Docs drift** at repo root (`README.md` references missing docs).

---

## Review scope and method

- Read core architecture and runtime flow docs (`README.md`, `hexapod-server/README.md`, `hexapod-client/README.md`).
- Inspected high-churn/control-critical implementation files in host runtime, transport, and firmware command dispatch.
- Built and ran host tests via CMake presets to validate current baseline.

---

## What is working well

### 1) Separation of concerns is explicit

- `hexapod-server` cleanly separates transport, estimation, safety, control pipeline, and diagnostics responsibilities.
- `hexapod-common` is a single source for command IDs/constants used on both sides.
- Simulator support and scenario files provide an effective risk-reduction path before hardware motion.

### 2) Host regression suite is meaningful

- 13 server tests compile and pass with a broad spread (runtime loop, transport components, parser sections, safety transitions, freshness policy).
- This gives good confidence for refactors in host-side control/transport layers.

### 3) Freshness/timeout safety posture is thoughtful

- Runtime uses freshness contracts (timestamp/sample monotonicity/age) and hard-gates control output to SAFE_IDLE on stale data.
- This is exactly the kind of guardrail expected in robot safety loops.

---

## Refactoring opportunities

## A) Consolidate server command transaction helpers (high impact)

### Observations

`SimpleHardwareBridge` has repeated request/decode patterns (`get_current`, `get_voltage`, `get_sensor`, calibration paths, servo state paths) that differ mostly by command ID and decode function.

### Risk

- Boilerplate encourages drift in logging, error taxonomy, and retry behavior.
- Feature additions require touching many callsites.

### Recommendation

Introduce a typed command helper layer (for example `CommandTraits<TCmd>` + `execute<TCmd>(...)`) that standardizes:

- command ID
- request payload encoding
- response decode contract
- failure log labels

### Acceptance criteria

- At least 5 existing command methods migrated.
- Error log format becomes uniform for all command paths.
- Existing transport tests remain green.

---

## B) Split `RobotRuntime::controlStep` into pure decision stages (high impact)

### Observations

`controlStep` currently performs loop timing metrics, freshness evaluation, stale counters, fail-safe status synthesis, and normal pipeline execution in one method.

### Risk

- Hard to reason about correctness of each branch.
- Test setup for edge cases is heavier than needed.

### Recommendation

Extract pure helpers, e.g.:

- `compute_loop_timing_metrics(...)`
- `evaluate_freshness_gate(...)`
- `build_fail_safe_status(...)`
- `run_nominal_control(...)`

Keep side effects (buffer writes/logging/counters) in a thin orchestration shell.

### Acceptance criteria

- Control gate logic has dedicated unit tests with stale estimator/intent matrices.
- `controlStep` body reduced significantly (target: <120 lines).

---

## C) Remove firmware route forwarding wrappers (medium impact)

### Observations

`hexapod-client/command_dispatch.cpp` defines many thin `*Routed` functions that simply forward to handlers.

### Risk

- Adds noisy maintenance overhead for every command addition.
- Easy to miss route/payload consistency updates.

### Recommendation

Adopt a small adapter utility (or overload set) so routes can bind directly to existing handler signatures where possible.

### Acceptance criteria

- Forwarding wrappers reduced by >70%.
- Route table remains compile-time constant.
- Existing command-router tests still pass.

---

## D) Centralize protocol command metadata (medium impact)

### Observations

Command identity and semantics are repeated in multiple places:

- `CommandClient::command_name(...)` switch on host.
- Firmware `COMMAND_ROUTES` payload policies.

### Risk

- Inconsistent updates when new commands are introduced.
- Debug logs and payload validation may drift from protocol definition.

### Recommendation

Create a shared command metadata registry in `hexapod-common` (X-macro/table) to generate:

- command name map
- optional default payload policy hints
- docs/test vectors

### Acceptance criteria

- One source of truth for command names.
- Host logging and firmware route declarations consume generated/derived metadata.

---

## E) Fix documentation drift (quick win)

### Observations

Root `README.md` references `docs/REFACTORING_REVIEW.md` and `docs/NEXT_STEPS.md`, which were missing before this review artifact update.

### Recommendation

Keep roadmap docs versioned and explicitly date-stamped so root references stay accurate.

---

## Priority roadmap

1. **P0 (this week):** Refactor A + B (transport helper unification and control-step split).
2. **P1 (next):** Refactor C + D (firmware route simplification + protocol metadata centralization).
3. **P2:** Expand host/firmware cross-integration tests for invalid payload and NACK behavior.
4. **P3:** Add lightweight architectural decision records (ADRs) for protocol schema evolution and runtime safety policy.

---

## Suggested tracking metrics

- **Complexity:** LOC/function for `controlStep` and `hardware_bridge.cpp`.
- **Reliability:** count of command failure logs by outcome class.
- **Velocity:** lines touched per new command introduction.
- **Safety:** stale-intent and stale-estimator event rates per runtime hour.

---

## Validation run during review

Executed in `hexapod-server/`:

- `cmake --preset tests`
- `cmake --build --preset tests -j`
- `ctest --preset tests --output-on-failure`

All commands completed successfully with 13/13 tests passing.
