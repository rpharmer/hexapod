# Hexapod Codebase Review (2026-03-23)

## Executive summary

The repository has a strong architectural backbone:

- Clear separation between host control (`hexapod-server`), firmware (`hexapod-client`), and protocol/shared definitions (`hexapod-common`).
- Mature host-side test coverage focused on runtime safety, transport, and kinematics.
- Good protocol rigor (metadata-driven command contracts and payload validation).

Primary refactoring opportunities are not about correctness defects, but about **scaling maintainability**:

1. Split large "god files" into narrower units with explicit interfaces.
2. Centralize repetitive command transaction/decode patterns.
3. Make thread loops and timing behavior observable/testable without real sleeping.
4. Improve configuration and geometry mutability boundaries.
5. Bring firmware-side host/unit tests into CI parity with server tests.

---

## Scope and method

This review combined:

- Architectural walkthrough of top-level docs and module boundaries.
- Targeted source inspection of high-churn/high-complexity areas.
- Build/test execution for server and firmware projects.
- Lightweight structural metrics (largest C/C++ files by line count).

---

## What is working well

### 1) Layered control architecture in server runtime

`RobotRuntime` cleanly stages bus read/write, estimation, safety, control pipeline, and diagnostics with explicit data handoff via double-buffer style state containers. This makes timing behavior understandable and testable.

### 2) Safety/freshness policy is explicit and enforced

Freshness gating and safety state transitions are first-class citizens rather than hidden ad hoc checks. This is a strong reliability trait for robot software.

### 3) Shared protocol metadata reduces drift risk

Firmware command routing validates payload lengths using metadata and compile-time assertions. This is an excellent pattern for protocol evolution.

### 4) Strong host-side verification baseline

`hexapod-server` has broad test coverage across key domains (runtime loop behavior, fault handling, transport, parser validation, IK/FK flow).

---

## Refactoring opportunities

## P0/P1 opportunities (highest leverage)

### A) Decompose oversized implementation units

**Observed:** Several files are large and combine multiple responsibilities:

- `hexapod-server/src/hardware/hardware_bridge.cpp` (~406 LOC): link lifecycle, request/response orchestration, payload encode/decode, and software feedback simulation.
- `hexapod-server/src/control/mode_runners.cpp` (~356 LOC): CLI parsing, scenario execution, interactive control mode state machine, calibration probe triggers.
- `hexapod-server/src/config/toml_parser.cpp` (~362 LOC): schema validation, transport/runtime parsing, calibration parsing/sorting, tuning/geometry parsing.

**Recommendation:** Split each by concern and create thin composition roots.

Suggested extraction map:

- `hardware_bridge.cpp` →
  - `hardware_transactions.cpp` (request helpers)
  - `hardware_calibration_io.cpp`
  - `software_feedback_estimator.cpp`
  - keep `SimpleHardwareBridge` as orchestrator only
- `mode_runners.cpp` →
  - `cli_options.cpp`
  - `controller_mode_mapper.cpp`
  - `calibration_actions.cpp`
  - `scenario_runner.cpp`
- `toml_parser.cpp` →
  - `schema_header_parser.cpp`
  - `runtime_transport_parser.cpp`
  - `calibration_table_parser.cpp`
  - `tuning_parser.cpp` / `geometry_parser.cpp`

**Expected impact:** lower merge conflicts, easier targeted testing, reduced review burden per change.

### B) Remove command transaction boilerplate in bridge

**Observed:** `SimpleHardwareBridge` repeats variants of:

1) build request payload,
2) call transaction helper,
3) decode scalar/array payload,
4) map decoded data into output type.

**Recommendation:** Introduce a typed request helper API (template or function-object based) for command + codec operations. Keep one internal transaction path and command-level wrappers that become declarative.

**Expected impact:** fewer subtle inconsistencies and easier protocol command additions.

### C) Improve loop executor testability and shutdown semantics

**Observed:** `LoopExecutor` runs periodic threads with sleep-until behavior but no explicit stop token/cancellation synchronization primitives beyond an atomic flag.

**Recommendation:**

- Introduce a `StopToken`/`CancellationSource` abstraction and inject sleep/timing strategy.
- Add unit tests with fake clock/ticker to validate cadence and prompt shutdown under load.

**Expected impact:** more deterministic timing tests; lower flake risk when adding new loops.

---

## P2 opportunities (important, medium horizon)

### D) Consolidate fault reasoning into declarative rule table

**Observed:** `SafetySupervisor::evaluateCurrentFault` interleaves sensor checks and priority selection logic with manual condition sequencing.

**Recommendation:** model fault checks as ordered rules (condition + fault + torque-cut). Keep priority policy data-driven in one location.

**Expected impact:** easier safety policy evolution and clearer safety reviews.

### E) Formalize mutable global geometry usage

**Observed:** interactive calibration paths update active geometry dynamics in place.

**Recommendation:**

- define ownership boundaries for mutable geometry,
- expose explicit "apply calibration profile" transaction,
- persist/load calibrated dynamics profiles through config instead of in-memory-only mutation.

**Expected impact:** clearer runtime behavior and reproducibility between sessions.

### F) Strengthen firmware host-test pathway

**Observed:** host build invocation completed successfully but `ctest` reported no tests discovered. Existing firmware tests are present in tree, but build wiring does not expose them in this environment.

**Recommendation:**

- add explicit CMake option for firmware host tests,
- register test binaries in `ctest`,
- run these in CI alongside server tests.

**Expected impact:** better protocol/dispatch regression protection where bugs are expensive on hardware.

---

## Potential risk themes

1. **Runtime complexity concentration** in a few large files may slow safe iteration.
2. **Protocol evolution overhead** from repetitive manual wrappers.
3. **Calibration reproducibility risk** if runtime mutations are not serialized.
4. **Coverage asymmetry** between server and firmware test discoverability.

---

## Prioritized next-step plan

### Sprint 1 (1–2 weeks)

- Extract CLI parsing + scenario runner from `mode_runners.cpp`.
- Introduce typed command transaction helper in `SimpleHardwareBridge` for scalar reads (`GET_CURRENT`, `GET_VOLTAGE`, `GET_SENSOR`) first.
- Wire firmware host tests into CMake/CTest and add CI execution.

### Sprint 2 (2–3 weeks)

- Split `toml_parser.cpp` calibration parsing into dedicated parser module with focused tests.
- Introduce safety rule table representation while preserving current behavior (golden tests).
- Add loop executor fake-clock tests.

### Sprint 3 (3+ weeks)

- Separate software feedback estimator from transport bridge.
- Add persisted calibration/dynamics profile schema and migration plan.
- Evaluate tracing/metrics extension for loop jitter and stale-data diagnostics trend export.

---

## Suggested success metrics

- Reduce top-3 production `.cpp` file sizes by ~30–40%.
- Keep or improve test runtime while adding firmware host tests in CI.
- Zero behavior regression in existing server 17-test suite.
- New protocol command additions require ≤1 helper wrapper and ≤1 decoder binding.

---

## Verification commands executed during review

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests --output-on-failure

cd ../hexapod-client
cmake -S . -B build-host-test -DHEXAPOD_CLIENT_ENABLE_HOST_TESTS=ON
cmake --build build-host-test -j
ctest --test-dir build-host-test --output-on-failure
```

Result summary:

- Server suite passed (17/17 tests).
- Firmware build succeeded; no tests were discovered by CTest in the current configuration.
