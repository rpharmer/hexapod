# Mission Autonomy & Navigation Implementation Plan

This plan tracks the autonomy implementation roadmap and current execution state for `hexapod-server`.

## Current Status Snapshot (2026-03-27)

### Implemented
- Contract documentation is in place under `docs/contracts/autonomy/`, including process supervision and traceability references.
- Contract validation plumbing is integrated in `AutonomyStack` with per-stream sample ordering and freshness checks.
- Autonomy module boundaries are implemented (lifecycle + health + module shells).
- Mission/navigation baseline behavior is implemented and tested:
  - mission script parsing
  - mission executive FSM
  - navigation intent emission
- Recovery/arbitration baseline behavior is implemented and tested:
  - progress no-motion/no-progress detection path
  - recovery chain (`hold -> replan -> abort`)
  - arbitration priority (`E_STOP > HOLD > RECOVERY > NAV`)
- World/traversability/planning baseline behavior is implemented and wired:
  - world model snapshot update
  - traversability risk/cost/confidence reporting
  - global/local planner shell behavior feeding locomotion dispatch gating
- Runtime integration is implemented for the autonomy-enabled control path with integration tests.
- Supervision policy metadata and fault-injection handling are implemented in stack runtime (criticality/process-group contracts, restart budgets, degraded-mode reasons, safe-stop on exhausted critical restart budget).

### Implemented but still baseline-level
- World/traversability and planner behavior are functional but intentionally lightweight shell algorithms.
- Supervision/process-group policies are fully represented and tested in-process; the architecture is not yet physically split into OS processes with IPC transport/runtime orchestration.

### Remaining / Not Implemented Yet
- True multi-process autonomy runtime split with IPC transport and external supervisor lifecycle orchestration.
- Hardware-in-the-loop acceptance coverage for autonomy mission/recovery/degraded behavior beyond simulation-centric tests.
- Long-duration reliability campaigns (restart churn, sustained degraded operation, and mission endurance evidence).

---

## Objectives

- Preserve contract-first autonomy interfaces and deterministic safety semantics.
- Keep mission/recovery behavior observable and testable end-to-end.
- Progress from in-process supervision semantics to true multi-process runtime isolation.
- Raise confidence from unit/integration simulation coverage to sustained HIL and endurance validation.

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

## Delivery State by Epic

### Epic 1: Contract Foundation (V1 Freeze)
**Status:** Complete in code/docs for v1.

### Epic 2: Source Scaffolding (No Behavior Change)
**Status:** Complete.

### Epic 3: Mission + Navigation Vertical Slice
**Status:** Complete for simulation/runtime baseline.

### Epic 4: Progress Monitoring + Recovery
**Status:** Complete for baseline escalation behavior and arbitration integration.

### Epic 5: World Model + Traversability
**Status:** Complete for baseline shell-level planning signals; ongoing fidelity/hardening work remains.

### Epic 6: Process Split + Runtime Hardening
**Status:** Partially complete.
- Implemented: process-group contracts, restart/safe-stop policy semantics, degraded-mode/fault-injection behavior in-process.
- Open: true process split + IPC orchestration at runtime boundary.

---

## Testing Strategy (Current)

1. **Contract tests**
   - schema validity and required fields
   - frame/timestamp/sample-order checks via contract enforcer
2. **FSM tests**
   - mission transitions and recovery escalation hooks
3. **Integration tests**
   - autonomy stack happy-path and blocked/recovery flows
   - robot runtime autonomy routing into control path
4. **Fault-injection tests**
   - crash/timeout isolation by criticality group
   - degraded-mode behavior and critical safe-stop on exhausted restart budget
5. **Regression gates**
   - existing freshness/safety/transport suites remain green

---

## Definition of Done (Autonomy Runtime v1.1)

- Module boundaries are implemented and documented.
- V1 contracts are versioned and enforced at autonomy runtime boundaries.
- Waypoint mission execution and recovery transitions are verified in simulation/runtime integration tests.
- Degraded-mode and restart/safe-stop policies are validated via fault injection.
- Remaining gaps (below) are explicitly tracked with acceptance evidence.

---

## Current Focus (2026-03-27)

Only open execution items are listed below.

1. **True multi-process autonomy split**
   - Implement separate critical / soft-RT / noncritical runtime processes.
   - Enforce existing logical IPC contracts over concrete IPC transport.
   - Add process lifecycle orchestration external to `AutonomyStack` in-process loops.

2. **HIL validation expansion**
   - Validate mission/recovery/degraded behavior on hardware transport paths.
   - Add acceptance checks for real timing jitter and transport fault behavior.

3. **Reliability & endurance evidence**
   - Run long-duration autonomous mission campaigns.
   - Track restart counts, degraded-mode time, mission completion reliability, and recovery success rates.

4. **Documentation and operator runbooks**
   - Keep contracts, runtime expectations, and failure handling runbooks synchronized with implementation.
