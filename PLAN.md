# Mission Autonomy & Navigation Implementation Plan

This plan converts the proposed autonomy architecture into an incremental, testable implementation roadmap aligned with the current `hexapod-server` codebase.

## Current Status Snapshot (2026-03-27)

### Implemented
- Contract documentation and review checklist exist in `docs/contracts/autonomy/`, including `traceability_matrix.md`.
- Contract/runtime foundations are in place:
  - common contract types
  - contract enforcer with version compatibility and freshness checks
- Source scaffolding is implemented:
  - lifecycle/health module stubs
  - per-module shell classes for autonomy module boundaries
- Mission/navigation baseline primitives are implemented and unit tested:
  - mission scripting parser
  - mission executive FSM
  - navigation manager intent emission
- Recovery/arbitration baseline primitives are implemented and unit tested:
  - progress monitor no-progress timeout
  - recovery strategy chain (hold/retry/replan/abort)
  - arbiter priority (`E_STOP > HOLD > RECOVERY > NAV`)

### Partially Implemented
- Runtime integration is present for the autonomy stack and runtime bridge tests, but full mission-scenario telemetry validation is still incomplete.
- World model + traversability contracts are wired through the autonomy stack with baseline planner consumption; broader planner fidelity and scenario depth remain open.

### Not Implemented Yet
- Criticality-based process split, watchdogs, supervisor restart policy, degraded mode behavior.
- Full mission E2E simulation scenario suite with explicit operator-facing telemetry acceptance criteria.

---

## Objectives

- Introduce explicit mission/navigation/autonomy module boundaries without destabilizing current locomotion control.
- Define and lock contract-first interfaces (units, frames, rates, timestamps, QoS, fault semantics).
- Deliver a first end-to-end waypoint mission path with recovery behavior.
- Evolve from single-process runtime to criticality-based process split after interfaces stabilize.

---

## Architecture Scope (Target Modules)

- `mission_executive`
- `navigation_manager`
- `recovery_manager`
- `motion_arbiter`
- `localization`
- `world_model`
- `traversability_analyzer`
- `locomotion_interface`
- `global_planner`
- `local_planner`
- `progress_monitor`
- `mission_scripting`

---

## Epic 1: Contract Foundation (V1 Freeze)

### Goal
Lock interface contracts before behavior work.

### Tasks
1. Create contract docs under `docs/contracts/autonomy/`:
   - `common_types.md`
   - `mission_executive.md`
   - `navigation_manager.md`
   - `recovery_manager.md`
   - `motion_arbiter.md`
   - `localization.md`
   - `world_model.md`
   - `traversability_analyzer.md`
   - `locomotion_interface.md`
2. Define canonical conventions:
   - units (SI)
   - frames (`map`, `odom`, `base_link`, `sensor_*`)
   - timestamp policy (`timestamp_ms` monotonic + optional UTC)
   - sample IDs and correlation IDs
   - QoS classes + TTL/staleness policy
3. Define contract versioning policy (`v1`, `v1.x`).
4. Add contract review checklist (units, frame tags, timing, fault behavior).

### Exit Criteria
- All module interfaces documented and reviewed.
- Arbitration priority and stale-data semantics explicitly specified.

### Status
**Mostly complete** (docs and validation primitives are in place). Remaining: enforce contracts at all runtime ingress/egress boundaries.

---

## Epic 2: Source Scaffolding (No Behavior Change)

### Goal
Introduce module boundaries around existing runtime.

### Tasks
1. Add module namespaces/directories for the target modules.
2. Implement lifecycle stubs: `init/start/step/stop`.
3. Add health and heartbeat status surfaces.
4. Wrap current implementations:
   - `SimpleEstimator` behind `localization`.
   - `IHardwareBridge::write(...)` behind `locomotion_interface`.
   - Existing safety/fault gating behind initial `motion_arbiter` adapter.

### Exit Criteria
- Build passes with new scaffolding.
- Existing behavior unchanged in sim/hardware mode.

### Status
**Scaffolding complete** (stubs/shells and tests exist). Remaining: replace placeholder shells with runtime-wired adapters.

---

## Epic 3: Mission + Navigation Vertical Slice

### Goal
Deliver one end-to-end mission pathway using existing control backend.

### Tasks
1. Implement `mission_scripting` waypoint parser/validator.
2. Implement `mission_executive` FSM:
   - `IDLE -> READY -> EXEC -> PAUSED/ABORTED/COMPLETE`.
3. Implement `navigation_manager` minimal orchestration:
   - consume mission waypoint
   - emit nav intent/local goal
   - publish nav status
4. Route nav output via adapters into existing locomotion/control path.

### Exit Criteria
- Waypoint mission executes end-to-end in simulation.
- Mission status/progress observable in telemetry/status channels.

### Status
**Partially complete** (1-3 implemented with tests). Remaining: runtime adapter wiring and E2E simulation/telemetry verification.

---

## Epic 4: Progress Monitoring + Recovery

### Goal
Add deterministic recovery for blocked/no-progress states.

### Tasks
1. Implement `progress_monitor`:
   - progress trend
   - no-progress timeout
   - oscillation detection
2. Implement `recovery_manager`:
   - retry budget
   - strategy chain (hold/retry/replan/abort)
3. Upgrade `motion_arbiter` priority policy:
   - `E_STOP > HOLD > RECOVERY > NAV`
4. Add recovery escalation hooks in `mission_executive`.

### Exit Criteria
- Stall conditions trigger recovery.
- Recovery success resumes mission; repeated failure aborts with explicit reason.

### Status
**Mostly complete** (1-4 implemented with tests, including mission escalation hooks). Remaining: expanded integrated blocked-path/E2E scenario coverage in the simulation harness.

---

## Epic 5: World Model + Traversability

### Goal
Standardize environment understanding input to planning.

### Tasks
1. Introduce `world_model` local map slice abstraction.
2. Implement `traversability_analyzer` producing risk/cost/confidence layers.
3. Feed `local_planner` from world model + traversability contracts.
4. Start with simple rule-based cost model; iterate later.

### Exit Criteria
- Planner consumes standardized world/traversability interfaces.
- Cost/risk changes are reflected in selected motion outputs.

### Status
**Partially complete** (baseline world-model snapshot + traversability analysis + planner wiring are implemented). Remaining: richer map semantics, planner cost tuning, and broader scenario validation.

---

## Epic 6: Process Split + Runtime Hardening

### Goal
Move from monolith to criticality-based multi-process runtime.

### Tasks
1. Split runtime by criticality:
   - critical: `localization`, `motion_arbiter`, `locomotion_interface`
   - soft RT: planners, `navigation_manager`, `recovery_manager`
   - noncritical: mission scripting, telemetry, operator API
2. Add supervisor restart policies and degraded-mode behavior.
3. Add watchdog + heartbeat dependency checks.

### Exit Criteria
- Crash isolation demonstrated.
- Degraded mode behavior documented and validated.

### Status
**Not started**.

---

## Testing Strategy (Across All Epics)

1. **Contract tests**
   - schema validity
   - required fields
   - units/frame/timestamp assertions
   - version compatibility
2. **FSM tests**
   - mission, navigation, recovery transitions
3. **Integration tests**
   - mission happy path
   - blocked path and recovery flow
4. **Fault injection tests**
   - stale localization
   - planner timeout
   - transport/bus faults
5. **Regression gates**
   - existing freshness/safety/transport test suites remain green

---

## Updated Rollout View

- **Sprint 1**: Epic 1 + Epic 2 ✅ (baseline complete)
- **Sprint 2**: Epic 3 🟡 (logic complete, runtime E2E pending)
- **Sprint 3**: Epic 4 + baseline Epic 5 🟡 (Epic 4 partial, Epic 5 pending)
- **Sprint 4**: Epic 6 + hardening/regression ⏳

---

## Definition of Done

- Module boundaries are implemented and documented.
- V1 contracts are versioned and validated.
- Waypoint mission E2E is passing in sim.
- Recovery paths and arbitration priority are tested.
- Process split completed with health/watchdog/degraded-mode behavior.

---

## Next Execution Slice (Immediate)

1. **Runtime integration PR**
   - wire mission/navigation/recovery/arbiter into `robot_runtime`
   - route selected nav intent into locomotion adapters
2. **E2E simulation PR**
   - add mission happy-path and blocked-path scenario tests
   - validate telemetry/status outputs and recovery transitions
3. **World/traversability bootstrap PR**
   - implement minimal world slice + traversability cost output
   - connect to local planner via contract interfaces

## Current Focus (2026-03-27)

Only open execution items are listed below.

1. **Mission/runtime E2E scenario closure**  
   **Owner:** Autonomy Runtime (Mission Integration)  
   **Open work:** close remaining mission-scenario validation gaps (happy path + blocked/recovery + telemetry acceptance) at the runtime boundary.  
   **Primary files/tests:**  
   - `hexapod-server/tests/test_robot_runtime_autonomy_integration.cpp`  
   - `hexapod-server/tests/test_autonomy_stack.cpp`  
   - `hexapod-server/src/control/runtime/robot_runtime.cpp`  
   - `hexapod-server/src/autonomy/modules/autonomy_stack.cpp`

2. **Simulation harness + acceptance assertions expansion**  
   **Owner:** Simulation & QA  
   **Open work:** encode additional scenario-driver cases and assertions for recovery transitions and operator-visible status channels.  
   **Primary files/tests:**  
   - `hexapod-server/src/scenario/scenario_driver.cpp`  
   - `hexapod-server/tests/test_autonomy_stack.cpp`  
   - `hexapod-server/tests/test_robot_runtime_autonomy_integration.cpp`

3. **World/traversability fidelity follow-through**  
   **Owner:** Planning/Perception  
   **Open work:** deepen map/traversability signal quality and planner behavior verification beyond the current baseline contract wiring.  
   **Primary files/tests:**  
   - `hexapod-server/src/autonomy/modules/traversability_analyzer.cpp`  
   - `hexapod-server/src/autonomy/modules/autonomy_stack.cpp`  
   - `hexapod-server/tests/test_autonomy_stack.cpp`  
   - `hexapod-server/tests/test_toml_parser_sections.cpp`

4. **Criticality process split + runtime hardening**  
   **Owner:** Platform Runtime  
   **Open work:** implement true multi-process split and validate watchdog/restart/degraded-mode behavior under fault injection.  
   **Primary files/tests:**  
   - `docs/contracts/autonomy/process_supervision_and_ipc.md`  
   - `hexapod-server/src/autonomy/modules/autonomy_stack.cpp`  
   - `hexapod-server/tests/test_autonomy_supervision_integration.cpp`
