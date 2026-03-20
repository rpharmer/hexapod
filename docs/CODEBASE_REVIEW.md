# Codebase Review (2026-03-20)

## Scope and approach

This review covers architecture, code quality, runtime safety, testability, and maintainability across:

- `hexapod-server/`
- `hexapod-client/`
- `hexapod-common/`

I built the server target and ran CTest discovery to verify baseline build health and test wiring.

## Executive summary

The codebase is in a **good functional shape** with clear subsystem boundaries (transport, runtime loop, estimator, safety, control pipeline) and a practical simulation path (`SimHardwareBridge`) that reduces hardware coupling risk.

The biggest improvement opportunities are:

1. **Configuration and state ownership**: mutable global config values are runtime-singleton style and not strongly scoped.
2. **Threaded loop scheduling and freshness semantics**: periodic loops are independent but there is no explicit data age/sequence contract between producers and consumers.
3. **Protocol + command handling duplication**: both host and firmware include repeated handshake/ACK/NACK and command routing logic that should be normalized.
4. **Testing ergonomics**: tests exist but are not built unless a specific CMake option is enabled, and default workflows currently discover zero tests.
5. **Observability**: good logging exists, but there are no exported metrics/traces for loop jitter, stale intent rates, or command latency distributions.

## What is working well

### 1) Clear layered control path on server

The server flow (`RobotControl` -> `RobotRuntime` -> `ControlPipeline`) is straightforward and easy to reason about. The separation between bus, estimator, control, safety, and diagnostics steps is clean and sets up future scheduler changes well.

### 2) Practical safety foundation

`SafetySupervisor` already centralizes fault ranking, latching, and controlled recovery hold-time behavior, including stale-intent protection and bus/power checks.

### 3) Useful simulation hooks

`ScenarioDriver` + `SimHardwareBridge` provide a valuable integration harness for non-hardware testing and fault injection.

### 4) Shared protocol codec discipline

`hexapod-common` and `protocol_codec.hpp` style usage give a good base for interface consistency across host and firmware.

## Refactoring opportunities (prioritized)

## P0 (high impact)

### A) Replace mutable global runtime config with explicit injected config

**Current pattern**

`control_config` stores mutable globals (`kBusLoopPeriod`, `kCommandTimeoutUs`, etc.) and mutates them through `loadFromParsedToml`. This tightly couples runtime behavior to global process state.

**Problems**

- Harder test isolation (tests can leak config mutations across cases).
- Harder future multi-robot/multi-instance support in one process.
- Hidden dependency graph (functions depend on global config but signatures do not show it).

**Refactor direction**

- Introduce immutable `ControlConfig` + `SafetyConfig` value objects.
- Parse TOML directly into those objects.
- Inject config into `RobotControl` / `RobotRuntime` / `SafetySupervisor` / loop scheduler at construction.

**Expected outcome**

- Better determinism and testability.
- Cleaner dependency boundaries and easier simulation matrix testing.

### B) Add explicit freshness/sequence contracts between runtime stages

**Current pattern**

`DoubleBuffer<T>` provides lock-based latest-value exchange, but stages (`busStep`, `estimatorStep`, `controlStep`, `safetyStep`) run in independent periodic threads without a sequence-id or per-stage age policy.

**Problems**

- Control step can consume mixed-epoch data (e.g., newest intent + older estimated state).
- Hard to detect stale producer failure except by heuristic checks in safety.

**Refactor direction**

- Extend shared state with monotonically increasing `sample_id` and `timestamp_us` per stream.
- Add `FreshnessPolicy` checks in control step (e.g., max estimator age, max intent age).
- Optionally migrate to single tick coordinator (deterministic pipeline) while keeping bus I/O async.

**Expected outcome**

- Stronger temporal correctness and simpler fault diagnosis.

### C) Normalize command/ACK/NACK transport semantics into a shared transaction layer

**Current pattern**

`hardware_bridge.cpp` has duplicated response validation logic (handshake, heartbeat, generic command ACK parsing). Firmware side command routing also repeats payload checks and ACK/NACK scaffolding.

**Problems**

- Duplicate protocol edge-case handling.
- Risk of drift in error responses across commands.

**Refactor direction**

- Extract reusable host-side `PacketTransaction` helper with uniform timeout/retry/seq validation.
- Add firmware command descriptor table with shared payload validators and common respond helpers.
- Define command-specific error enums mapped to protocol NACK codes in one place.

**Expected outcome**

- Smaller defect surface in serial protocol handling.
- Easier expansion of command set.

## P1 (medium impact)

### D) Improve test defaults and CI predictability

**Current pattern**

CMake only builds tests when `HEXAPOD_SERVER_BUILD_TESTS=ON`; default workflows discover no tests.

**Problems**

- Easy to assume tests ran when they did not.
- Reduces confidence in merge-time checks.

**Refactor direction**

- Add CI profile/build preset with tests enabled by default.
- Update README server instructions with explicit test build command.
- Add `ctest` smoke target in CI.

### E) Strengthen scenario validation and schema

**Current pattern**

Scenario parsing is permissive; invalid modes/gaits generally fall back to defaults.

**Problems**

- Silent fallback can hide scenario authoring mistakes.

**Refactor direction**

- Add strict schema validation with explicit errors on unknown enums.
- Add scenario lint command (offline check).

### F) Consolidate duplicated motion-intent builders

Both `hexapod-server.cpp` and `scenario_driver.cpp` define similar motion intent construction helpers.

**Refactor direction**

- Move intent-building helpers into a shared utility in server core.

## P2 (incremental polish)

### G) Improve firmware state ownership and test seams

**Current pattern**

Firmware uses a process-wide singleton `FirmwareContext` (`firmware()` global accessor).

**Problems**

- Hard to unit test command handlers without global state coupling.

**Refactor direction**

- Introduce handler interfaces/functions parameterized by `FirmwareContext&`.
- Keep singleton only at boot boundary.

### H) Naming and typo consistency pass

There are recurring typo strings (`recieved`) and mixed naming styles.

**Refactor direction**

- Apply lint/static analysis plus low-risk consistency cleanup.

## Recommended next steps (execution plan)

### Next 2 weeks

1. **Config injection epic (P0-A)**
   - Add immutable config structs.
   - Thread through constructors.
   - Remove mutable globals from control/safety paths.

2. **Testing workflow hardening (P1-D)**
   - Add test-enabled build preset.
   - Add CI `cmake --preset` + `ctest` lane.
   - Document local commands in `hexapod-server/README.md`.

3. **Protocol transaction helper (P0-C, part 1)**
   - Host-side first: unify ACK/NACK parsing for all command paths.

### 2–6 weeks

4. **Freshness/epoch contracts (P0-B)**
   - Add sample IDs and stale detection telemetry.
   - Gate control output if inputs violate age policy.

5. **Scenario strict validation (P1-E)**
   - Define accepted schema, enforce unknown-key/enum failure.
   - Add scenario unit tests for malformed files.

6. **Firmware handler refactor seams (P2-G)**
   - Dependency-inject context into command handlers.
   - Keep behavior unchanged.

## Suggested KPIs to track

- Control loop deadline miss rate (%).
- Mean/P99 control-loop jitter.
- Intent staleness events per minute.
- Command round-trip latency P50/P95/P99.
- Fault trip counts by type and clear success rate.
- Scenario run determinism (reproducible fault timelines in sim).

## Validation run notes

- Server builds cleanly with default options.
- `ctest` currently discovers no tests unless test option is enabled.
