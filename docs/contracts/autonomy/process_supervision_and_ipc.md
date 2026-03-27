# Process Supervision + IPC Contracts (v1)

## Process boundaries

The autonomy stack is split into process-isolated module classes with explicit restart/watchdog policy:

| Module | Criticality class | Heartbeat timeout | Max restarts | Dependencies |
|---|---|---:|---:|---|
| mission_executive | **critical** | 100 ms | 1 | - |
| motion_arbiter | **critical** | 100 ms | 1 | navigation_manager, recovery_manager |
| locomotion_interface | **critical** | 75 ms | 2 | motion_arbiter, local_planner |
| navigation_manager | **soft-RT** | 100 ms | 2 | mission_executive |
| localization | **soft-RT** | 75 ms | 3 | navigation_manager |
| world_model | **soft-RT** | 100 ms | 2 | localization |
| traversability_analyzer | **soft-RT** | 100 ms | 2 | world_model |
| global_planner | **soft-RT** | 120 ms | 2 | navigation_manager, traversability_analyzer |
| local_planner | **soft-RT** | 120 ms | 2 | global_planner |
| mission_scripting | noncritical | 500 ms | 1 | - |
| progress_monitor | noncritical | 500 ms | 1 | mission_executive |
| recovery_manager | noncritical | 500 ms | 1 | progress_monitor |

Supervisor semantics:

1. Every process publishes a heartbeat timestamp each control cycle.
2. Crash and timeout faults are monitored independently.
3. If restart budget is available, supervisor performs `stop -> init -> start` restart.
4. If restart budget is exhausted:
   - critical module faults fail the autonomy step (hard fault),
   - soft-RT/noncritical faults are isolated and the stack enters degraded mode.

## IPC contracts (logical)

The v1 IPC schema between process boundaries is defined by typed messages already used in module shells:

- `NavigationUpdate` (mission executive/navigation intent handoff).
- `LocalizationEstimate` (pose freshness + frame-consistent state).
- `WorldModelSnapshot` and `TraversabilityReport` (map/risk for planning).
- `GlobalPlan` and `LocalPlan` (hierarchical planning contract).
- `MotionDecision` and `LocomotionCommand` (final motion gating + dispatch).
- `ProgressEvaluation` and `RecoveryDecision` (watchdog/recovery feedback loop).

QoS contract:

- Producers must update heartbeat every cycle they emit data.
- Consumers must treat dependencies as stale if heartbeat age exceeds timeout.
- Stale or unavailable dependencies must be handled through degraded-mode behavior (below), not by unsafe best-effort dispatch.

## Degraded-mode behaviors

Safe fallback is explicit and deterministic for high-risk dependency loss:

1. **Stale localization**
   - Trigger: localization timeout/crash, or invalid localization estimate.
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
- soft-RT timeout (`localization`) and verifies stale-localization safe hold;
- critical-path dispatch crash (`locomotion_interface`) and verifies restart and dispatch-failure fallback.

These tests assert process boundary metadata, watchdog behavior, restart accounting, and degraded-mode output reasons.
