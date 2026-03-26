# Hexapod Codebase Review (2026-03-26) — Comprehensive Edition

## What changed from the previous review

This revision expands coverage from a representative architectural sample to a **full-source inventory review** of the server codebase.

- Reviewed scope now explicitly includes **every `*.cpp` and `*.hpp`** under:
  - `hexapod-server/src`
  - `hexapod-server/include`
  - `hexapod-server/tests`
- Total files reviewed: **184**.
- File-by-file matrix with follow-ups: `docs/CODEBASE_SOURCE_FILE_MATRIX_2026-03-26.md`.

---

## Review Scope and Method

## Inventory coverage

1. Enumerated source/test headers+implementations across runtime/control/hardware/input/kinematics/config/scenario/utils.
2. Added a per-file review matrix with:
   - approximate LOC,
   - primary responsibility,
   - next-step recommendation (refactor/hardening/testing).

## Deep-dive quality pass

Beyond the matrix pass, conducted detailed inspection of high-impact runtime and orchestration files:

- `hexapod-server/src/control/runtime/robot_runtime.cpp`
- `hexapod-server/src/control/pipeline/gait_policy_planner.cpp`
- `hexapod-server/src/hardware/hardware_bridge.cpp`
- `hexapod-server/src/scenario/scenario_driver.cpp`
- `hexapod-server/src/control/runtime/loop_executor.cpp`
- `hexapod-server/src/utils/logger.cpp`
- `hexapod-server/CMakeLists.txt`
- `docs/DYNAMIC_GAIT_POLICY.md`

---

## Systemic Findings Across the Full File Set

## Strengths

1. **Subsystem decomposition is consistent** (app/config/control/hardware/input/kinematics/scenario/utils).
2. **Safety-first architecture exists in multiple layers** (freshness checks, fallback states, safety supervisor, scenario fault injection).
3. **High baseline test footprint** with broad subsystem test files already present.
4. **Explicit source list in CMake** keeps build composition deterministic.

## Cross-cutting risks

1. **Concentration risk in large files**: a handful of high-LOC units carry disproportionate logic (runtime orchestration, gait policy, hardware bridge).
2. **Policy drift risk**: dynamic gait policy thresholds are duplicated between documentation and planner implementation.
3. **Operational toggles not fully runtime-driven** in some spots (notably IMU read behavior).
4. **Build/test maintenance overhead** due to repetitive test wiring patterns.
5. **Observability gaps** for long-window timing/backpressure trends.

---

## Refactoring Opportunities (Prioritized)

## P0 — Reliability and deployment safety

### 1) Convert compile-time behavior toggles into runtime configuration

- Move IMU-read enable/disable from compile-time constants to validated config fields.
- Publish effective mode in startup logs and telemetry metadata.
- Add tests ensuring config toggles behave consistently in sim + serial modes.

### 2) Add longitudinal runtime health signals

- Extend diagnostics with p50/p95/p99 loop timing and overrun trends.
- Export logger queue pressure/dropped-message counters.
- Introduce thresholded warnings before hard faults trigger.

## P1 — Maintainability and modularity

### 3) Split oversized runtime and hardware units

- Extract `hardware_bridge` responsibilities into link/protocol/command metadata modules.
- Split runtime orchestration by step responsibility (bus, estimator, control, telemetry, diagnostics).
- Extract gait planner subcomponents (region classifier, family selector, fallback policy).

### 4) Isolate scenario parse/validate/execute layers

- Separate TOML decode from semantic validation and runtime execution.
- Add table-driven tests for strict-mode errors and edge constraints.

## P2 — Developer velocity and evolution

### 5) Reduce CMake test boilerplate

- Introduce helper function/macros for standardized test registration.
- Optionally break test definitions into subsystem cmake include files.

### 6) Formalize contract docs per subsystem

- For each subsystem, define invariants, ownership boundaries, and failure contracts.
- Link these contracts to relevant tests in the matrix.

---

## File-Coverage Deliverable

Full file-by-file review output:

- `docs/CODEBASE_SOURCE_FILE_MATRIX_2026-03-26.md`

What it provides:

1. Every source/test header+implementation file accounted for.
2. Per-file role tagging.
3. Concrete next-step recommendation for each file.

This matrix is designed to be a working backlog seed for phased execution.

---

## 90-Day Execution Plan (Updated)

## Phase 1 (Weeks 1-3): Hardening + telemetry

1. Runtime-configurable IMU reads and explicit mode telemetry.
2. Scheduler percentile metrics in diagnostics stream.
3. Logger backpressure telemetry and warning thresholds.

## Phase 2 (Weeks 4-8): Core modularization

1. `hardware_bridge` decomposition with no behavior change.
2. `gait_policy_planner` decomposition + targeted tests.
3. Scenario parser/validator/executor separation.

## Phase 3 (Weeks 9-12): Scale and maintainability

1. CMake test helper rollout.
2. Subsystem contract docs tied to tests.
3. Golden scenario telemetry regression harness.

---

## Future Feature Directions

1. **Runtime-selectable gait profile packs** (`precision`, `rough_terrain`, `throughput`).
2. **Adaptive terrain feedback loops** (contact/stability-driven CR and cadence adaptation).
3. **Fault replay harness** from captured telemetry + intent streams.
4. **Versioned telemetry contracts** for visualizer/analytics compatibility.
5. **Predictive health scoring** blending freshness, timing, power, and transport quality.

---

## Success Metrics

- File ownership hot-spot reduction (large-file change concentration).
- Mean review time for control/hardware PRs.
- p95 control-loop jitter and overrun rate under scenario suite.
- Incident diagnosis turnaround time.
- Time-to-add new scenario fields with strict validation and test coverage.
