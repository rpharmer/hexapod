# Codebase Review (2026-03-24)

## Scope

This review focused on:

- `hexapod-server` runtime/control/hardware boundary
- `hexapod-client` command dispatch integration with shared protocol metadata
- `hexapod-common` protocol definitions shared by both binaries
- test topology and maintainability characteristics

Validation baseline was `./scripts/verify.sh` from repo root.

## High-level assessment

The repository has strong engineering fundamentals:

- clear monorepo boundaries (`server`, `client`, `common`)
- a shared protocol contract in `hexapod-common`
- broad automated coverage and deterministic simulator scenarios
- production-oriented runtime instrumentation (freshness diagnostics, loop timing metrics)

Primary opportunities are now **maintainability and scaling**:

1. reducing repetitive command-bridge logic
2. decomposing large orchestration classes/files
3. simplifying oversized integration-style tests through reusable fixtures
4. tightening protocol typing to reduce legacy uint8 command aliases over time
5. improving operational signal quality in logs/exit semantics

---

## Detailed findings and refactoring opportunities

### 1) Hardware bridge has repeated precondition + link checks

`SimpleHardwareBridge` repeats nearly identical guards in most public methods:

- init/object readiness checks
- `ensure_link(kRequestedCapabilities)` checks
- command_api null checks

This pattern appears across command methods such as `set_power_relay`, `get_current`, `get_voltage`, `get_sensor`, LED/servo methods, etc.

**Impact**

- high copy/paste surface for future behavior changes
- inconsistent logging behavior risk (some paths log, many quietly return false)
- harder to add cross-cutting concerns (telemetry, retries, error taxonomy)

**Refactoring direction**

- introduce a private helper such as:
  - `bool requireReady(bool need_feedback_estimator = false)`
  - `template<typename F> bool withCommandApi(F&& fn)` to centralize link+null checks
- standardize failure logging through one utility that includes command name and reason
- optionally return richer error type internally (enum/class) and map to bool at interface boundary for incremental adoption

---

### 2) `RobotRuntime` combines many responsibilities in one class

`RobotRuntime` currently owns and coordinates:

- hardware IO loop behavior
- estimator integration
- control pipeline gating
- safety freshness evaluation
- diagnostics/timing metrics and log emission
- intent stamping/sample sequencing

This is solid functionally, but it increases coupling between real-time logic and observability/state bookkeeping.

**Impact**

- harder to reason about behavioral changes in one loop without touching others
- increased risk of regression from mixed concerns
- difficult to performance-profile hot path independent of diagnostics concerns

**Refactoring direction**

- split into composable helpers:
  - `RuntimeFreshnessGate` (evaluation + stale counters + reject status generation)
  - `RuntimeTimingMetrics` (dt/jitter tracking)
  - `RuntimeDiagnosticsReporter` (status + structured metrics logging)
- keep `RobotRuntime` as orchestration shell calling these components
- preserve existing public API to avoid broad call-site changes

---

### 3) Large integration test file should be modularized

`hexapod-server/tests/test_hardware_bridge_transport.cpp` is very large and mixes many concerns in one translation unit (handshake, retries, malformed payloads, calibration, led, servos, diagnostics, capabilities).

**Impact**

- slower comprehension and slower targeted iteration
- larger compile units
- more fragile merge conflict surface

**Refactoring direction**

- split test file by behavior domains:
  - `test_hardware_bridge_handshake.cpp`
  - `test_hardware_bridge_commands.cpp`
  - `test_hardware_bridge_failures.cpp`
- extract shared fake endpoint and helper assertions into a tiny local test utility header
- keep current test naming for CI continuity via CTest labels

---

### 4) Shared protocol header is in a transitional typing state

`hexapod-common/include/hexapod-common.hpp` defines strong enums (`CommandCode`, etc.) but still exports broad legacy `uint8_t` aliases for compatibility.

**Impact**

- call sites can still accidentally pass arbitrary bytes where command enums are expected
- limits compile-time safety and discoverability

**Refactoring direction**

- phase migration toward enum-based APIs in host and firmware transport layers
- add overloads taking `CommandCode` and keep legacy `uint8_t` wrappers temporarily
- after migration, shrink alias surface to only externally required protocol constants

---

### 5) Runtime logging semantics can be clarified for ops

Scenario smoke logs show both:

- `All workers joined` as WARN
- `Dropped messages=0` as ERROR

even in successful completion.

**Impact**

- can produce false-positive alerting/noise in log pipelines
- obscures real fault triage when operators scan by severity

**Refactoring direction**

- make graceful shutdown informational when exit code is success
- only emit error severity for non-zero dropped-message counts or real shutdown faults
- add a short `shutdown_summary` log line with explicit success/failure state

---

## Recommended next steps (prioritized)

### Phase 1 (1-2 days): low-risk maintainability wins

1. Add bridge precondition helper + central failure reason logging in `SimpleHardwareBridge`.
2. Reclassify shutdown log severities and add explicit success summary.
3. Add test utility helpers for bridge transport tests (without splitting yet).

### Phase 2 (2-4 days): structural decomposition

1. Extract freshness/timing/diagnostics helper components from `RobotRuntime`.
2. Keep exact behavior and existing tests green.
3. Add unit tests specifically for extracted components to improve isolation.

### Phase 3 (2-3 days): protocol type-hardening migration

1. Add strongly-typed command overloads in command client/router APIs.
2. Update server/client call sites incrementally.
3. Deprecate broad legacy aliases once all internal usages are migrated.

### Phase 4 (1-2 days): test topology cleanup

1. Split `test_hardware_bridge_transport.cpp` into domain-oriented files.
2. Introduce fixture utility header for fake endpoint + packet helpers.
3. Keep scenario-level integration tests as final confidence layer.

---

## Suggested objective metrics to track refactor progress

- `SimpleHardwareBridge` repeated readiness guard count (target: near-zero duplicates)
- `RobotRuntime` LOC and method count (target: meaningful reduction via extracted helpers)
- largest single test translation unit LOC (target: < 500)
- number of call sites using raw `uint8_t` command constants (target: trending toward zero internally)
- ratio of WARN/ERROR logs during successful scenario completion (target: zero false positives)

---

## Validation performed for this review

- full repository verification via `./scripts/verify.sh`
  - server tests passed
  - firmware host tests passed
  - scenario smoke passed

No behavioral code changes were applied in this review document update.
