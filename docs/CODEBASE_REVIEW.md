# Codebase Review (March 20, 2026)

## Scope

This review covers:

- `hexapod-server/` host control runtime (threading, control loop orchestration, safety, serial bridge)
- `hexapod-client/` RP2040 firmware command loop and transport
- `hexapod-common/` framing/protocol contracts
- Documentation and repository hygiene gaps that impact maintainability

## Executive Summary

The project has a strong architectural skeleton (clear host/firmware/common split, explicit control loop periods, protocol framing reuse), but maintainability and correctness risks concentrate in three areas:

1. **Safety/control semantics are clear but under-tested**: core behavior is encoded in code paths without automated regression tests.
2. **Protocol handling is mostly robust but repetitive**: duplicated ACK/NACK, sequence, payload-validation, and command routing logic can drift over time.
3. **Configuration and observability are functional but brittle**: globals and stringly-typed config fields increase hidden coupling.

## What Is Working Well

### 1) Clear modular boundaries

- The monorepo cleanly separates host runtime, firmware, and shared protocol assets.
- The server’s runtime loop decomposition (`bus`, `estimator`, `control`, `safety`, `diagnostics`) is explicit and easy to reason about.

### 2) Strong protocol lifecycle shape

- Handshake and heartbeat are modeled as first-class behavior in the server transport stack.
- Firmware enforces explicit state transitions (`WAITING_FOR_HOST` → `ACTIVE`) and host-liveness timeout fallback.

### 3) Safety-first defaults

- Safety supervisor latches faults and uses recovery hold semantics.
- Firmware host-disconnect path actively disables servos and power relay GPIO mask.

## Key Risks and Refactoring Opportunities

## A. Data model duplication and weak invariants

### Findings

- Domain types for joint and leg state are split across `JointRawState`, `JointState`, `LegRawState`, `LegState`, and `JointTargets`, with partial overlap and repetitive conversion code paths.
- `ServoCalibration` contains near-duplicate transform logic for both `LegRawState` and `LegState`.
- The same semantic entity (angle) is represented in both strongly typed wrappers and plain `double/float` payload paths.

### Impact

- Increased bug surface when modifying kinematics or calibration logic.
- Harder unit-test coverage due to repeated implementations.

### Refactoring Opportunity

- Introduce a single internal representation for kinematics-space and transport-space leg state, with thin adapters at boundaries.
- Collapse calibration transforms to a templated helper (or compile-time indexed utility) that maps joint arrays once.

## B. Runtime concurrency model is simple but coarse-grained

### Findings

- `DoubleBuffer<T>` uses one mutex and value copy on every read/write.
- `RobotRuntime::busStep()` reads/writes hardware state and command state with no explicit time synchronization marker beyond per-struct timestamps.
- `LoopExecutor` runs independent periodic threads without overrun accounting or jitter statistics.

### Impact

- Potential timing drift goes unobserved.
- High-copy data movement can become costly as telemetry grows.

### Refactoring Opportunity

- Upgrade `DoubleBuffer` to support lock-free index swap for trivially copyable payloads or use a reader/writer strategy.
- Add per-loop timing metrics (last duration, moving average, overrun count) and emit in diagnostics.
- Introduce a monotonic sequence counter per produced frame (`raw`, `estimated`, `control`) to track staleness explicitly.

## C. Serial bridge and command handling repetition

### Findings

- Server bridge contains repeated send/wait/decode patterns (`get_current`, `get_voltage`, `get_sensor`, etc.).
- Firmware dispatch table is clean, but routing wrappers and payload policies are manually managed, increasing maintenance load as protocol grows.
- Error logging text has minor inconsistencies/typos (e.g., “recieved”, “initialised”), signaling lack of lint/static checks.

### Impact

- Higher risk of inconsistent behavior across commands.
- Harder onboarding for adding new command IDs safely.

### Refactoring Opportunity

- Introduce generic request helpers:
  - `request_ack(cmd, payload)`
  - `request_decode<T>(cmd, payload, decoder)`
- Move command metadata (ID, payload policy, handler, capabilities) to a shared declarative table and generate router glue where possible.
- Add spellcheck/lint pass for log/user-facing messages.

## D. Safety and control logic need explicit test harnesses

### Findings

- Safety decision matrix (bus timeout, motor bounds, foot contacts, tip-over, command staleness) is implemented in `SafetySupervisor` with no visible automated tests.
- Gait/body controller files contain TODO markers for key motion policy behavior.

### Impact

- Regressions in high-risk safety behavior may only appear in hardware-in-loop testing.

### Refactoring Opportunity

- Add host-side deterministic tests for:
  - fault priority ordering,
  - latch/recovery lifecycle,
  - command timeout behavior,
  - safe-idle inhibit and torque-cut transitions.
- Add property-style tests for IK/FK sanity envelopes (reachable workspace bounds, inversion consistency).

## E. Configuration and docs alignment gaps

### Findings

- Root `README.md` references planning docs that were not present before this review update.
- Config parsing currently depends on precise key spelling and nested dotted keys, but schema evolution tooling is minimal.

### Impact

- Documentation drift and config fragility during iterative development.

### Refactoring Opportunity

- Establish configuration schema evolution guide (versioned migration rules).
- Validate key presence with grouped “required/optional/deprecated” reporting in a single parser pass.

## Suggested Priority Order

1. **Safety test harness + deterministic unit tests** (highest risk reduction).
2. **Serial command helper abstraction** (largest repetitive code reduction).
3. **Loop observability instrumentation** (enables performance tuning and fault triage).
4. **Type model consolidation for leg/joint/calibration transforms** (long-term maintainability).
5. **Config/documentation hardening** (developer experience and onboarding).
