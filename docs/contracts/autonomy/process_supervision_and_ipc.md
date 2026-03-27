# Process Supervision + IPC Contracts (v1)

## Process boundaries

The autonomy stack is split into three explicit process groups with strict IPC crossings:

- **critical**: safety/motion gating and final dispatch path.
- **soft-RT**: planning + state-estimation path.
- **noncritical**: mission authoring/progress/recovery bookkeeping.

Each module contract maps to one process group and a restart/safe-stop policy:

| Module | Criticality class | Process group | Heartbeat timeout | Max restarts | Safe-stop on exhausted restarts | Dependencies |
|---|---|---|---:|---:|---|---|
| mission_executive | **critical** | critical | 100 ms | 1 | yes | - |
| motion_arbiter | **critical** | critical | 100 ms | 1 | yes | navigation_manager, recovery_manager |
| locomotion_interface | **critical** | critical | 75 ms | 2 | yes | motion_arbiter, local_planner |
| navigation_manager | **soft-RT** | soft-RT | 100 ms | 2 | no | mission_executive |
| localization | **soft-RT** | soft-RT | 75 ms | 3 | no | navigation_manager |
| world_model | **soft-RT** | soft-RT | 100 ms | 2 | no | localization |
| traversability_analyzer | **soft-RT** | soft-RT | 100 ms | 2 | no | world_model |
| global_planner | **soft-RT** | soft-RT | 120 ms | 2 | no | navigation_manager, traversability_analyzer |
| local_planner | **soft-RT** | soft-RT | 120 ms | 2 | no | global_planner |
| mission_scripting | noncritical | noncritical | 500 ms | 1 | no | - |
| progress_monitor | noncritical | noncritical | 500 ms | 1 | no | mission_executive |
| recovery_manager | noncritical | noncritical | 500 ms | 1 | no | progress_monitor |

Supervisor semantics:

1. Every process publishes a heartbeat timestamp each control cycle.
2. Crash and timeout faults are monitored independently.
3. If restart budget is available and all declared dependencies are healthy, supervisor performs `stop -> init -> start` restart.
4. If dependencies are unhealthy, restart is deferred and the module is isolated.
5. If restart budget is exhausted:
   - contracts with `safe-stop on exhausted restarts = yes` fail the autonomy step (hard fault),
   - all others are isolated and the stack remains up in degraded behavior.

## IPC contracts (logical)

The v1 IPC schema between process boundaries is defined by typed messages already used in module shells:

- `mission_executive -> navigation_manager` via `NavigationUpdate` (**critical -> soft-RT boundary**).
- `navigation_manager -> global_planner` via `NavigationUpdate`.
- `localization -> world_model` via `LocalizationEstimate`.
- `world_model -> traversability_analyzer` via `WorldModelSnapshot`.
- `traversability_analyzer -> global_planner` via `TraversabilityReport`.
- `global_planner -> local_planner` via `GlobalPlan`.
- `local_planner -> locomotion_interface` via `LocalPlan` (**soft-RT -> critical boundary**).
- `motion_arbiter -> locomotion_interface` via `MotionDecision`.
- `progress_monitor -> recovery_manager` via `ProgressEvaluation`.
- `recovery_manager -> motion_arbiter` via `RecoveryDecision` (**noncritical -> critical boundary**).

QoS contract:

- Producers must update heartbeat every cycle they emit data.
- Consumers must treat dependencies as stale if heartbeat age exceeds timeout.
- Stale or unavailable dependencies must be handled through degraded-mode behavior (below), not by unsafe best-effort dispatch.

## Degraded-mode behaviors

Safe fallback is explicit and deterministic for high-risk dependency loss:

1. **Stale localization**
   - Trigger: localization timeout/crash, invalid localization estimate, or soft-RT group heartbeat degradation.
   - Action: motion gated off, locomotion dispatch suppressed.
   - Output reason: `"stale localization"`.

2. **Planner unavailable**
   - Trigger: global/local planner timeout/crash, or missing global/local plan.
   - Action: motion gated off, locomotion dispatch suppressed.
   - Output reason: `"planner unavailable"`.

3. **Locomotion dispatch failure**
   - Trigger: dispatch rejected/not sent while motion path expected healthy.
   - Action: force hold, no command dispatch.
   - Output reason: `"locomotion dispatch failure"`.

## Integration verification

`test_autonomy_supervision_integration` injects:

- soft-RT crash (`global_planner`) and verifies isolation + restart + planner fallback;
- noncritical crash (`mission_scripting`) and verifies isolation without stack teardown;
- soft-RT timeout (`localization`) and verifies stale-localization safe hold;
- repeated critical crash (`motion_arbiter`) and verifies safe-stop after restart budget exhaustion.

These tests assert process boundary metadata, watchdog behavior, restart accounting, safe-stop behavior, and degraded-mode output reasons.
