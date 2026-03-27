# Autonomy Contract Traceability Matrix (v1)

This matrix ties each v1 autonomy contract to an owning producer/consumer boundary and baseline QoS/staleness expectations.

| Contract | Primary Producer | Primary Consumer | QoS | Max Age (ms) | Stale Behavior |
| --- | --- | --- | --- | ---: | --- |
| `mission_executive` | `mission_executive` | `navigation_manager` | `QOS_STATUS` | 500 | `HOLD` |
| `navigation_manager` | `navigation_manager` | `motion_arbiter` | `QOS_CONTROL` | 500 | `HOLD` |
| `recovery_manager` | `recovery_manager` | `motion_arbiter`, `mission_executive` | `QOS_CONTROL` | 500 | `DEGRADE` |
| `motion_arbiter` | `motion_arbiter` | `locomotion_interface` | `QOS_CRITICAL` | 100 | `SAFE_STOP` |
| `localization` | `localization` | `navigation_manager`, `local_planner` | `QOS_CRITICAL` | 100 | `DEGRADE` |
| `world_model` | `world_model` | `traversability_analyzer`, `local_planner` | `QOS_BULK` | 1000 | `DEGRADE` |
| `traversability_analyzer` | `traversability_analyzer` | `local_planner` | `QOS_BULK` | 1000 | `DEGRADE` |
| `locomotion_interface` | `motion_arbiter` | hardware bridge | `QOS_CRITICAL` | 100 | `SAFE_STOP` |

## Notes

- Major version compatibility follows `common_types.md`: consumers reject unknown major versions and tolerate additive fields within `v1.x`.
- Every contract stream must carry `timestamp_ms`, `sample_id`, and `correlation_id` where mission/nav action tracing applies.
- Runtime boundary validation is enforced through `contract_enforcer` at these boundaries:
  - **Mission script ingestion:** `autonomy.mission_script.ingress`
  - **Autonomy step ingress:** `autonomy.step.ingress`
  - **Planner boundaries:** `autonomy.global_planner.output`, `autonomy.local_planner.output`
  - **Locomotion command boundary:** `autonomy.locomotion.command_output`
- Stream IDs are canonical and must match exactly (non-empty mismatches are rejected) to prevent stream-ID drift from bypassing per-stream sample ordering checks.
- Per-stream `sample_id` monotonicity is tracked across runtime boundaries and rejects duplicate or out-of-order samples.
