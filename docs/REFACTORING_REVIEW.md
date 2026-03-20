# Refactoring Plan (Detailed)

## 1) Control/Safety Testability Refactor

### Goal
Create deterministic, host-only tests for safety and control policy without requiring hardware.

### Current pain points
- Safety behavior is stateful and timing-dependent.
- No explicit seams for time injection in supervisor logic.

### Proposed changes
1. Inject a clock provider interface into safety and control modules.
2. Add pure decision helpers for fault evaluation and lifecycle transitions.
3. Build table-driven tests for all fault precedence and recovery pathways.

### Acceptance criteria
- `SafetySupervisor` test suite covers all `FaultCode` branches.
- Recovery timer behavior is tested with deterministic synthetic timestamps.
- CI can run tests without serial/hardware dependencies.

---

## 2) Serial Protocol Call-Site Consolidation

### Goal
Reduce repetitive command boilerplate in `SimpleHardwareBridge` and improve consistency.

### Current pain points
- Repeated command-send + ack + decode patterns.
- Per-command logging and failure handling diverges.

### Proposed changes
1. Add generic helper functions inside bridge layer:
   - `bool requestAck(uint8_t cmd, span<const uint8_t> payload)`
   - `template <typename T> bool requestDecoded(uint8_t cmd, Decoder<T>, T& out)`
2. Convert existing command methods to use helpers.
3. Centralize error mapping for NACK/status mismatches.

### Acceptance criteria
- At least 40% reduction in repetitive command code in `hardware_bridge.cpp`.
- Uniform error messages include command name and sequence context.

### Implementation notes (2026-03-20)
- Added `SimpleHardwareBridge` helpers:
  - `request_ack(...)`
  - `request_ack_payload(...)`
  - `request_decoded(...)`
- Migrated command call sites (`read`, `write`, `get_*`, servo commands, and diagnostics)
  to use the shared helpers.
- Consolidated command failure/decode logging so failures now consistently include the
  command name and expected response type.

---

## 3) Data Type Unification for Joint/Leg State

### Goal
Cut duplicate conversion logic and enforce stronger invariants for state transformations.

### Current pain points
- Similar structs (`LegRawState`, `LegState`) and parallel conversion functions increase maintenance cost.

### Proposed changes
1. Introduce shared joint-array primitive (e.g., `JointArray<T, kJointsPerLeg>`).
2. Create named wrappers for semantic contexts (raw servo, kinematic, estimated).
3. Replace duplicated calibration conversion functions with one generic implementation.

### Acceptance criteria
- Conversion functions in `types.cpp` reduced to one implementation per direction.
- No behavior regression in existing integration behavior (validated by sim/unit tests).

---

## 4) Loop Timing Instrumentation

### Goal
Make real-time loop behavior observable and diagnosable.

### Current pain points
- No overrun/jitter counters per loop task.

### Proposed changes
1. Extend `LoopExecutor::Task` with task name.
2. Track execution duration and overrun count in a thread-safe metrics store.
3. Emit periodic diagnostics summary (bus/estimator/control/safety/diagnostics loops).

### Acceptance criteria
- Runtime log includes per-loop rate, average runtime, and overrun count.
- Control regressions can be identified from logs alone.

---

## 5) Config Parser Hardening

### Goal
Improve parser maintainability and schema evolution readiness.

### Current pain points
- String-key lookups are repetitive and easy to drift.
- Partial warnings/errors are spread across several helper lambdas.

### Proposed changes
1. Add typed extraction utilities with consistent diagnostics.
2. Move expected key descriptors into a static schema map.
3. Add config validation report object (errors + warnings) returned to caller.

### Acceptance criteria
- Single-pass validation output summarizes all invalid keys and ranges.
- Parser behavior is covered by fixture-based tests (valid + invalid configs).
