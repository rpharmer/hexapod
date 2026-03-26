# motion_arbiter Contract (v1)

## Responsibilities
- Select final motion command among competing intents.
- Enforce safety and priority ordering before locomotion output.

## Priority Policy (normative)
`E_STOP > HOLD > RECOVERY > NAV`

## Inputs
- Safety controls (`E_STOP`, holds)
- Recovery intents
- Navigation intents
- Localization freshness/health indicators

## Output
- Single `motion_command` envelope to `locomotion_interface`:
  - command type
  - target velocity/body motion
  - source_priority
  - timestamp_ms
  - correlation_id

## Stale Data Semantics
- Missing/stale safety channel: fail-safe to `HOLD`.
- Missing/stale nav intent: do not reuse indefinitely; transition to `HOLD` when `age_ms` exceeds contract threshold.

## Fault Behavior
- On internal arbitration inconsistency, emit `ERROR` and default to `HOLD`.
