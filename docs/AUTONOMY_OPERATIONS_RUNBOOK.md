# Autonomy Operations Runbook

This runbook is the operator/developer reference for startup sequencing, supervision behavior, degraded-mode interpretation, safe-stop handling, HIL acceptance, and reliability campaign execution.

## 1) Startup topology and bring-up order

### 1.1 Canonical module topology

```text
mission_executive ──> navigation_manager ──> global_planner ──> local_planner ──> locomotion_interface
         │                                           ▲                                  ▲
         └────────> progress_monitor ──> recovery_manager ──┘                                  │
                                                                                                  │
localization ──> world_model ──> traversability_analyzer ─────────────────────────────────────────┘

motion_arbiter <──────────────────────── navigation_manager, recovery_manager
motion_arbiter ─────────────────────────────────────────────────────────────> locomotion_interface
```

### 1.2 Process-group topology

- **critical group**: `mission_executive`, `motion_arbiter`, `locomotion_interface`
- **soft-RT group**: `navigation_manager`, `localization`, `world_model`, `traversability_analyzer`, `global_planner`, `local_planner`
- **noncritical group**: `mission_scripting`, `progress_monitor`, `recovery_manager`

### 1.3 Safe bring-up sequence

1. Verify all required hardware/transport links are connected and telemetry sink is writable.
2. Start **critical group** first and verify heartbeats.
3. Start **soft-RT group** and wait until localization and planners report healthy.
4. Start **noncritical group**.
5. Arm mission execution only after degraded reason is empty and dispatch path is healthy.

## 2) Supervisor restart policy and restart-budget implications

### 2.1 Restart policy summary

- Fault classes: crash and heartbeat timeout are tracked independently.
- Restart action with available budget + healthy dependencies: `stop -> init -> start`.
- If dependencies are unhealthy: restart is deferred and module is isolated.
- If restart budget is exhausted:
  - modules marked safe-stop-on-exhaustion trigger hard fault/safe-stop;
  - all others remain isolated and stack continues in degraded mode.

### 2.2 Restart budgets and on-call impact

| Module | Group | Max restarts | Safe-stop on exhausted restarts | Key dependencies | On-call implication |
|---|---|---:|---|---|---|
| mission_executive | critical | 1 | yes | - | Immediate incident; autonomy must remain stopped until root cause identified. |
| motion_arbiter | critical | 1 | yes | navigation_manager, recovery_manager | Any second failure in window forces safe-stop; treat as Sev-1 during missions. |
| locomotion_interface | critical | 2 | yes | motion_arbiter, local_planner | After repeated dispatch-path failures, safe-stop is expected and must not be overridden. |
| navigation_manager | soft-RT | 2 | no | mission_executive | Can degrade safely, but mission progress stalls; investigate planner chain health. |
| localization | soft-RT | 3 | no | navigation_manager | Common degraded trigger; motion remains gated until fresh localization returns. |
| world_model | soft-RT | 2 | no | localization | Isolation impacts traversability/planning quality; expect planner unavailability if prolonged. |
| traversability_analyzer | soft-RT | 2 | no | world_model | Planner quality/availability degradation; watch for cascading planner holds. |
| global_planner | soft-RT | 2 | no | navigation_manager, traversability_analyzer | Route generation unavailable; degraded reason should flip to planner unavailable. |
| local_planner | soft-RT | 2 | no | global_planner | Dispatch path blocked without local plan; degraded reason should flip to planner unavailable. |
| mission_scripting | noncritical | 1 | no | - | Scripted mission authoring unavailable; does not require emergency stop. |
| progress_monitor | noncritical | 1 | no | mission_executive | Recovery signal quality reduced; monitor recovery_manager decisions closely. |
| recovery_manager | noncritical | 1 | no | progress_monitor | Fallback recovery behavior may be limited; motion_arbiter may become more conservative. |

## 3) Degraded-mode interpretation and troubleshooting decision trees

### 3.1 Canonical degraded reasons

Only these canonical reasons should be emitted for the top-level degraded hold path:

- `stale localization`
- `planner unavailable`
- `locomotion dispatch failure`

### 3.2 Decision tree: `stale localization`

```text
[degraded_reason == "stale localization"]
  |
  +--> Is localization process alive?
        |
        +-- no --> Check supervisor event: crash vs timeout.
        |          - crash: inspect recent restart attempts, dependency health.
        |          - timeout: inspect sensor/clock/heartbeat pipeline.
        |
        +-- yes --> Are localization timestamps advancing within timeout?
                     |
                     +-- no --> investigate timebase drift, blocked estimator loop, sensor backlog.
                     |
                     +-- yes --> check estimate validity flags/frame metadata mismatch;
                                 quarantine bad upstream data and reinitialize estimator.
```

### 3.3 Decision tree: `planner unavailable`

```text
[degraded_reason == "planner unavailable"]
  |
  +--> Is global_planner healthy?
        |
        +-- no --> inspect traversability_analyzer + navigation_manager heartbeats and restart events.
        |
        +-- yes --> Is local_planner healthy and receiving GlobalPlan?
                     |
                     +-- no --> inspect IPC edge global_planner -> local_planner.
                     |
                     +-- yes --> verify plan freshness/TTL and arbitration gates in motion_arbiter.
```

### 3.4 Decision tree: `locomotion dispatch failure`

```text
[degraded_reason == "locomotion dispatch failure"]
  |
  +--> Did motion_arbiter produce a command this cycle?
        |
        +-- no --> inspect upstream gating from navigation_manager/recovery_manager.
        |
        +-- yes --> Did locomotion_interface accept and send command?
                     |
                     +-- no --> inspect transport/firmware link, command validation rejects, rate limiting.
                     |
                     +-- yes --> check downstream ack/telemetry path for false negative dispatch status.
```

## 4) Safe-stop response runbook

When safe-stop triggers due to critical restart-budget exhaustion or critical hard fault:

1. **Acknowledge and hold**: confirm motion remains gated and no dispatch resumes automatically.
2. **Capture evidence (first 2 minutes)**:
   - supervisor fault log (fault type, module, restart count, dependency status),
   - degraded reason timeline,
   - recent heartbeat ages for critical + soft-RT modules.
3. **Stabilize platform**: place robot in physically safe posture and lock operator controls.
4. **Classify trigger**:
   - critical crash loop,
   - critical timeout loop,
   - dispatch-path hard failure.
5. **Recover only via controlled sequence**:
   - clear transient hardware/transport causes,
   - restart stack from bring-up order,
   - re-arm only after healthy heartbeat + no degraded reason.
6. **Escalate** if safe-stop repeats in the same test window; open incident with attached logs and exact module failure chain.

## 5) HIL acceptance procedure

Use this before promoting autonomy/runtime changes to broader testing.

### 5.1 Preconditions

- Candidate build hash and matching docs revision identified.
- Contract/test changes merged for modified modules.
- Hardware fixture calibration complete; telemetry collection active.

### 5.2 Test sequence

1. Execute nominal startup and mission smoke run.
2. Inject soft-RT fault (`localization` timeout) and verify degraded hold reason becomes `stale localization`.
3. Inject planner fault (`global_planner` or `local_planner`) and verify reason `planner unavailable`.
4. Inject dispatch-path fault in locomotion path and verify reason `locomotion dispatch failure`.
5. Inject repeated critical fault (`motion_arbiter` crash loop) and verify safe-stop on restart-budget exhaustion.
6. Confirm post-fault recovery path succeeds only after healthy dependencies and explicit re-arm.

### 5.3 Acceptance criteria

- All expected degraded reasons appear exactly/canonically.
- No unsafe command dispatch during degraded hold.
- Safe-stop engages deterministically for critical exhaustion.
- Supervisor event log and telemetry counters align with observed behavior.

## 6) Reliability campaign execution (operators + developers)

### 6.1 Campaign setup

- Define campaign window (duration, terrain profile, mission mix).
- Freeze build + docs + schema versions for campaign batch.
- Declare success metrics (e.g., mission completion rate, MTBF proxy, restart frequency by module, degraded-reason frequency).

### 6.2 Execution loop

1. Run repeated missions under scripted variability (load, terrain, sensor stress).
2. Collect per-run artifacts:
   - module heartbeat statistics,
   - supervisor restart counters,
   - degraded reason counts and dwell times,
   - safe-stop occurrence and trigger chain.
3. Tag each run as pass/degraded-safe/safe-stop/fail-unsafe.
4. Triage daily:
   - operator view: reliability impact + operational mitigations,
   - developer view: root-cause hypotheses + candidate fixes/tests.

### 6.3 Exit criteria and follow-up

- Exit when reliability targets are met for consecutive windows without unsafe outcomes.
- For each recurring degraded reason, file corrective action linking code, contract, and test updates.
- Publish campaign summary with top regressions, mitigations, and next validation gate.

## 7) Contract change checklist (IPC/schema changes)

Any IPC payload, schema, or contract-field change must land with this checklist completed:

- [ ] **Contract docs updated**: relevant file(s) in `docs/contracts/autonomy/` reflect new/changed fields, QoS, and failure semantics.
- [ ] **Producer + consumer code updated**: all impacted module endpoints parse/emit the new schema version.
- [ ] **Tests updated together**: unit/integration/fault-injection tests cover new behavior and backward compatibility expectations.
- [ ] **Telemetry updated together**: counters/events/dashboards/parsers consuming the payload are revised in the same change window.
- [ ] **Traceability updated**: `traceability_matrix.md` links the changed contract elements to implementation/tests.
- [ ] **Rollout note included**: compatibility or migration plan documented for operators.

## 8) Minimal on-call quick reference

### 8.1 Dependency highlights

- Dispatch path requires: `motion_arbiter` + `local_planner` + `locomotion_interface`.
- Planning path requires: `navigation_manager` -> `global_planner` -> `local_planner`.
- Mapping/perception path requires: `localization` -> `world_model` -> `traversability_analyzer`.
- Recovery influence path: `progress_monitor` -> `recovery_manager` -> `motion_arbiter`.

### 8.2 Fast triage

1. Identify degraded reason.
2. Check module heartbeat age and restart count for direct path dependencies.
3. If critical module exhausted restart budget, do not force re-enable; follow safe-stop response.
4. If soft-RT/noncritical isolation only, keep robot held, restore dependencies, then re-arm through normal startup gate.
