# world_model Contract (v1)

## Responsibilities
- Maintain local environment representation for planning.
- Provide map slices keyed by frame, bounds, and timestamp.

## Inputs
- Sensor-derived environment updates.
- Optional localization transform references.

## Outputs
- Local map slice descriptor:
  - `frame_id`
  - `resolution_m`
  - `bounds`
  - `timestamp_ms`
  - layer registry (occupancy, elevation, etc.)

## Fault Behavior
- If map is stale/unavailable, publish degraded status so planners can reduce aggressiveness or hold.
