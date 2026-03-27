# navigation_manager Contract (v1)

## Responsibilities
- Convert mission waypoints into navigation intents/local goals.
- Orchestrate planner use and monitor planner outcomes.
- Publish navigation progress/status to `mission_executive`.

## Inputs
- Waypoint or route segment requests from `mission_executive`.
- Localization pose/velocity estimates.
- World/traversability updates.

## Outputs
- `nav_intent` to `motion_arbiter` (or downstream adapter).
- Navigation status:
  - `NAV_IDLE`, `NAV_ACTIVE`, `NAV_BLOCKED`, `NAV_COMPLETE`, `NAV_FAILED`
- Progress metrics:
  - `distance_remaining_m`
  - `heading_error_rad`
  - `timestamp_ms`
- Planner-boundary stream IDs used by runtime validation:
  - `autonomy.global_planner.output`
  - `autonomy.local_planner.output`

## Timing / Staleness
- Localization older than `max_age_ms=200` => mark nav degraded.
- World/traversability older than `max_age_ms=1000` => reduce speed envelope and request refresh.

## Fault Behavior
- Planner timeout or no-feasible-path emits `NAV_BLOCKED` and recovery trigger hint.
- Hard faults emit `NAV_FAILED` with `fault_code` and `correlation_id`.
