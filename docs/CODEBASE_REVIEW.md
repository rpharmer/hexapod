# Codebase Review (March 20, 2026)

## Scope

This review covers:

- `hexapod-server/` host control runtime (threading, control-loop orchestration, safety, serial bridge)
- `hexapod-client/` RP2040 firmware command loop and transport
- `hexapod-common/` framing/protocol contracts
- repository documentation and maintainability gaps

## Executive summary

The codebase has a strong architectural foundation (clear host/firmware/common split and reusable framing), but maintainability and correctness risks still cluster in a few areas:

1. **Safety/control semantics are clear but under-tested.**
2. **Protocol handling is robust but repetitive across call sites.**
3. **Configuration and observability are functional but still brittle.**

## What is working well

### 1) Clear modular boundaries

- Runtime responsibilities are cleanly separated across host, firmware, and shared protocol modules.
- Server loop decomposition (`bus`, `estimator`, `control`, `safety`, `diagnostics`) is explicit.

### 2) Strong protocol lifecycle

- Handshake and heartbeat are first-class behavior in host transport.
- Firmware enforces lifecycle transitions (`WAITING_FOR_HOST` → `ACTIVE`) and host-liveness timeout handling.

### 3) Safety-first defaults

- Safety supervisor latches faults and enforces recovery hold semantics.
- Firmware disconnect handling disables servos and drops relay outputs.

## Key risks and refactoring opportunities

### A. Data model duplication and weak invariants

**Findings**

- Overlapping domain types (`JointRawState`, `JointState`, `LegRawState`, `LegState`, `JointTargets`) duplicate semantics.
- Calibration transforms are repeated for structurally similar state containers.
- The same conceptual quantity (joint angle) appears in both wrapped and primitive forms.

**Impact**

- Higher change risk when evolving kinematics/calibration code.
- Harder to build concise unit tests for transformation behavior.

**Opportunity**

- Use a unified internal representation for kinematic-space and transport-space leg state.
- Keep thin adapters at boundaries instead of duplicating conversion logic.

### B. Runtime concurrency is simple but coarse-grained

**Findings**

- `DoubleBuffer<T>` currently uses one mutex and copies on each read/write.
- Cross-loop synchronization relies mostly on timestamps inside data structures.
- Loop threads lack consolidated overrun/jitter reporting.

**Impact**

- Timing drift can go undetected in long runs.
- Copy overhead can grow with telemetry payload size.

**Opportunity**

- Add lock-free index swap (where safe) or reader/writer strategies.
- Emit per-loop runtime metrics (last duration, moving average, overrun count).
- Add monotonic frame sequence counters to track staleness.

### C. Serial bridge and dispatch repetition

**Findings**

- Host bridge has repeated send/wait/decode logic for scalar and sensor queries.
- Firmware dispatch is organized but still manually managed per command.
- Error wording is inconsistent in some log paths.

**Impact**

- Inconsistent behavior is more likely as command surface area grows.
- Onboarding new command IDs remains higher effort than needed.

**Opportunity**

- Continue consolidating request helpers (`request_ack`, `request_decode<T>` style APIs).
- Move command metadata toward declarative tables for routing and payload validation.
- Add lightweight linting/spellcheck for logs and docs.

### D. Safety and control need stronger automated validation

**Findings**

- Safety decision logic is sophisticated but only partially regression-tested.
- Control/gait modules still contain policy TODOs.

**Impact**

- High-risk regressions may appear only in hardware-in-loop runs.

**Opportunity**

- Expand deterministic host-side tests for fault precedence and latch/recovery behavior.
- Add IK/FK property-style checks for workspace and inversion consistency.

### E. Config/doc alignment and schema evolution

**Findings**

- Config parser behavior depends on exact key spelling and nested key conventions.
- Schema migration/error reporting patterns are still minimal.

**Impact**

- Config drift can cause runtime surprises during iterative development.

**Opportunity**

- Introduce schema descriptors and grouped required/optional/deprecated reporting.
- Keep docs synchronized with runtime schema changes in the same PR.

## Suggested priority order

1. **Safety test harness expansion** (highest risk reduction)
2. **Protocol helper abstraction completion** (largest repetitive code reduction)
3. **Loop observability instrumentation** (supports performance triage)
4. **Type model consolidation for state/calibration paths** (maintainability)
5. **Config schema/documentation hardening** (developer experience)
