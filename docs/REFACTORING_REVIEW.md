# Refactoring Plan (Detailed)

## 1) Control/Safety testability refactor

### Goal
Create deterministic, host-only tests for safety and control policy without requiring hardware.

### Current pain points

- Safety behavior is stateful and timing dependent.
- Time injection seams are limited in supervisor logic.

### Proposed changes

1. Inject a clock provider interface into safety and control modules.
2. Add pure decision helpers for fault evaluation and lifecycle transitions.
3. Build table-driven tests for fault precedence and recovery pathways.

### Acceptance criteria

- `SafetySupervisor` tests cover all `FaultCode` branches.
- Recovery timers are validated using deterministic timestamps.
- Tests run in CI without serial or hardware dependencies.

---

## 2) Serial protocol call-site consolidation

### Goal
Reduce repetitive command boilerplate in `SimpleHardwareBridge` and enforce consistent error handling.

### Current pain points

- Repeated send + wait + decode patterns across commands.
- Per-command logging and failure paths diverge.

### Proposed changes

1. Keep generic bridge helpers (`request_ack`, `request_ack_payload`, `request_decoded`) as the single path.
2. Migrate remaining direct command logic to those helpers.
3. Centralize NACK/timeout/status error mapping.

### Acceptance criteria

- Significant reduction in duplicated command code in `hardware_bridge.cpp`.
- Uniform failures include command name and sequence context.

### Implementation notes (2026-03-20)

- Added `SimpleHardwareBridge` helper primitives for ACK-only, ACK+payload, and typed decode flows.
- Migrated command call sites (`read`, `write`, scalar sensors, servo commands, diagnostics).
- Consolidated decode/error logging with consistent command-context formatting.

---

## 3) Data type unification for joint/leg state

### Goal
Reduce duplicate conversion logic and enforce stronger invariants for state transformations.

### Current pain points

- Similar structs (`LegRawState`, `LegState`) and parallel conversions increase maintenance cost.

### Proposed changes

1. Introduce a shared joint-array primitive (for example `JointArray<T, kJointsPerLeg>`).
2. Use named wrappers for semantic contexts (raw servo, kinematic, estimated).
3. Replace duplicated calibration conversion functions with one generic implementation.

### Acceptance criteria

- `types.cpp` conversion logic is reduced to one implementation per direction.
- No behavior regression in integration/simulation results.

---

## 4) Loop timing instrumentation

### Goal
Make runtime loop behavior observable and diagnosable.

### Current pain points

- No comprehensive overrun/jitter counters by loop.

### Proposed changes

1. Extend `LoopExecutor::Task` with stable task naming.
2. Track execution duration and overrun counts in thread-safe metrics.
3. Emit periodic diagnostics summary for all loops.

### Acceptance criteria

- Runtime logs include loop rate, average runtime, and overrun count.
- Control regressions are diagnosable from logs without hardware traces.

---

## 5) Config parser hardening

### Goal
Improve parser maintainability and schema evolution readiness.

### Current pain points

- String-key lookups are repetitive and easy to drift.
- Validation diagnostics are split across helper lambdas.

### Proposed changes

1. Add typed extraction utilities with consistent diagnostics.
2. Move expected key descriptors into a static schema map.
3. Return a config validation report object (errors + warnings).

### Acceptance criteria

- Single-pass validation summarizes all invalid keys/ranges.
- Parser behavior is covered by fixture-based tests for valid + invalid configs.
