# mission_executive Contract (v1)

## Responsibilities
- Own mission lifecycle and top-level mission FSM.
- Validate mission script inputs before execution.
- Delegate waypoint execution to `navigation_manager`.

## FSM
States:
- `IDLE`
- `READY`
- `EXEC`
- `PAUSED`
- `ABORTED`
- `COMPLETE`

Required transitions:
- `IDLE -> READY` on mission load success
- `READY -> EXEC` on start command
- `EXEC -> PAUSED` on pause command or safety hold
- `PAUSED -> EXEC` on resume
- `EXEC|PAUSED -> ABORTED` on fatal unrecoverable fault
- `EXEC -> COMPLETE` when mission goals exhausted and success criteria met

## Inputs
- Mission script/waypoints (validated by `mission_scripting`)
- Operator controls: start/pause/resume/abort
- Nav/recovery status updates

## Outputs
- Mission status snapshot:
  - `mission_id`
  - `state`
  - `active_waypoint_index`
  - `timestamp_ms`
  - `correlation_id`
- `nav_request` messages to `navigation_manager`
- `abort_reason` on terminal failure

## Fault Behavior
- On `FATAL` faults from motion/safety path: immediate transition to `ABORTED`.
- On repeated recovery exhaustion from `recovery_manager`: transition to `ABORTED` with explicit cause.
