# Codebase Review (2026-03-20)

## Scope

This review covers architecture, runtime behavior, test/build health, and maintainability for:

- `hexapod-server/`
- `hexapod-client/`
- `hexapod-common/`

I did a static review of core modules and ran a server build to validate baseline health.

## Executive summary

The monorepo has a **solid architectural direction** (clear split between host, firmware, and shared protocol), but current branch health is **red** due to an incomplete refactor in `RobotRuntime` that breaks compilation.

### Overall assessment

- **Architecture:** Good boundaries and sensible module decomposition.
- **Operational risk:** Medium-high until compile break and temporal-contract gaps are resolved.
- **Test confidence:** Medium-low because default configuration does not build tests.
- **Refactoring readiness:** High (code is already modular enough for incremental hardening).

---

## What is working well

### 1) Clean host/firmware/common separation

The monorepo layout and protocol sharing are well structured, with the host runtime (`hexapod-server`), firmware command plane (`hexapod-client`), and framing/protocol constants (`hexapod-common`) separated in a maintainable way.

### 2) Runtime modules are conceptually well-factored

The server control stack separates orchestration (`RobotControl`), runtime stage execution (`RobotRuntime`), loop scheduling (`LoopExecutor`), control-policy (`ControlPipeline`), and fault handling (`SafetySupervisor`). This is a strong base for deterministic improvements.

### 3) Firmware command routing is table-driven

The firmware dispatcher uses a command route table with payload policies and handler indirection, which is a good pattern for extending protocol coverage with lower regression risk.

### 4) Scenario support improves offline validation

Scenario loading and simulated fault toggles are practical and useful for non-hardware validation workflows.

---

## Critical findings (high priority)

## F1 — Build is currently broken in `hexapod-server`

A direct configure/build run fails in `robot_runtime.cpp` and `scenario_driver.cpp`.

### Evidence

`cmake -S hexapod-server -B hexapod-server/build && cmake --build hexapod-server/build -j` fails with:

- missing members: `intent_sample_id_`, `raw_sample_id_`, `estimator_sample_id_`, `stale_estimator_count_`
- use of `config_.freshness` even though `ControlConfig` has no `freshness`
- type mismatches where `DoubleBuffer<T>` holds `T` but code reads `StreamSample<T>`
- undefined identifiers `est_sample` / `intent_sample`
- unused function warning escalated to error (`buildMotionIntent` in `scenario_driver.cpp`)

### Root cause (inference)

`RobotRuntime` appears partially migrated to stream-sampled freshness semantics, but header/state declarations and config schema were not completed. The resulting hybrid state cannot compile.

### Recommendation

Reconcile `RobotRuntime` and `ControlConfig` in one atomic change:

1. Either revert to plain `T` buffers immediately (short-term stabilization), or
2. Complete migration to `StreamSample<T>` buffers + freshness config fields + counters.

Do not keep mixed semantics in tree.

## F2 — Temporal contracts are under-specified even before the failed refactor

`LoopExecutor` runs bus/estimator/control/safety/diagnostics loops as independent periodic threads. Without explicit stage sequence/epoch contracts, control can consume mixed-age data.

### Risk

- Harder causality during fault triage.
- Stale input handling becomes heuristic rather than deterministic.

### Recommendation

Introduce explicit sample metadata and freshness policies (or move to a single coordinator tick with async I/O isolation).

## F3 — Tests are opt-in, easy to skip unintentionally

`HEXAPOD_SERVER_BUILD_TESTS` defaults to `OFF`, so standard build instructions can pass while running zero tests.

### Risk

Merge-time confidence is weaker than it appears.

### Recommendation

Provide a default test-enabled preset/CI lane and make it the documented path.

---

## Medium-priority maintainability findings

## F4 — Transport transactions are still fragmented

`hardware_bridge.cpp` centralizes some ACK/NACK handling with `TransportSession`/`CommandClient`, but command-specific behaviors remain scattered and error-mapping is still ad-hoc.

### Recommendation

Define a single transaction/result model for host command execution with:

- standardized timeout/retry policy,
- canonical protocol error mapping,
- structured telemetry fields (cmd, seq, latency, outcome).

## F5 — Firmware state is still globally singleton-scoped

`firmware_context.cpp` exposes global mutable state through `firmware()`.

### Risk

- Reduced command-handler testability.
- Hidden dependencies in command functions.

### Recommendation

Pass `FirmwareContext&` through handler surfaces and keep singleton usage only at boot/wiring boundary.

## F6 — Scenario parser is permissive in ways that can hide authoring mistakes

Scenario parsing validates key enums, but still defaults many fields silently. This is practical but can conceal malformed inputs in larger test matrices.

### Recommendation

Add strict mode / linter mode for scenario schema validation and unknown-key detection.

---

## Refactoring opportunities (ordered backlog)

### P0 (stabilization)

1. **Fix compile break in `RobotRuntime` migration** (F1).
2. **Define canonical runtime freshness contract** (F2).
3. **Enable test-on-by-default profile in docs + CI** (F3).

### P1 (reliability)

4. **Unify host transaction outcomes and error taxonomy** (F4).
5. **Add structured runtime metrics (loop jitter, freshness violations, transport RTT)**.
6. **Add scenario strict-validation path and tests** (F6).

### P2 (design hardening)

7. **De-singleton firmware command surfaces** (F5).
8. **Consolidate intent-building and remove dead helper paths**.
9. **Add static analysis/lint gates for unused functions and naming consistency**.

---

## Suggested KPIs

Track these as part of runtime hardening:

- Build health (main branch compile success rate).
- Tests executed per CI run (count + pass rate).
- Control-loop average and P99 jitter.
- Freshness violations per minute (intent/estimator).
- Transport command latency P50/P95/P99.
- Fault trip counts by `FaultCode`, plus mean clear time.

---

## Immediate next steps (recommended sequencing)

1. **Emergency stabilization PR**
   - Resolve `RobotRuntime` compile mismatch.
   - Remove/guard dead helper triggering `-Werror` failure.

2. **Runtime contract PR**
   - Add explicit freshness fields to config/types.
   - Add unit tests for stale estimator/intent behavior.

3. **Build/test workflow PR**
   - Add test-enabled preset as standard.
   - Update docs to lead with test build path.

4. **Transport and observability PR**
   - Normalize command outcome model.
   - Emit structured counters/histograms.

5. **Firmware seam PR**
   - Reduce singleton coupling in command handlers.

This order restores branch health first, then raises reliability and maintainability with minimal disruption.
