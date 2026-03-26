# Mission Autonomy & Navigation Implementation Plan

This plan converts the proposed autonomy architecture into an incremental, testable implementation roadmap aligned with the current `hexapod-server` codebase.

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

## 4-Sprint Rollout

- **Sprint 1**: Epic 1 + Epic 2
- **Sprint 2**: Epic 3
- **Sprint 3**: Epic 4 + baseline Epic 5
- **Sprint 4**: Epic 6 + hardening/regression

---

## Definition of Done

- Module boundaries are implemented and documented.
- V1 contracts are versioned and validated.
- Waypoint mission E2E is passing in sim.
- Recovery paths and arbitration priority are tested.
- Process split completed with health/watchdog/degraded-mode behavior.
