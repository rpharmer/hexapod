# Visualiser Telemetry Schema Contract

This document defines the UDP JSON contract between `hexapod-server` (producer) and `hexapod-visualiser/server.py` (consumer).

## Schema version

- Field: `schema_version`
- Type: integer
- Current value: `1`
- Requirement: **required** on every payload (`geometry` and `joints`).
- Consumer behavior: payloads with missing or incompatible `schema_version` are ignored and a warning is printed for operators.

## Leg name mapping

The robot core uses `LegID` names in right/left + index form (`R3/L3/...`).
The visualiser wire schema uses spatial keys (`LF/LM/...`).

| Server `LegID` | Visualiser key | Meaning |
|---|---|---|
| `L1` | `LF` | Left front |
| `L2` | `LM` | Left middle |
| `L3` | `LR` | Left rear |
| `R1` | `RF` | Right front |
| `R2` | `RM` | Right middle |
| `R3` | `RR` | Right rear |

Mapping is deterministic and encoded in `visualiser_telemetry::legIdToVisualiserKey`.

## Units by field

### Geometry payload (`type = "geometry"`)

| Field | Unit on wire | Internal source unit |
|---|---|---|
| `geometry.coxa` | millimeters (`mm`) | meters (`m`) |
| `geometry.femur` | millimeters (`mm`) | meters (`m`) |
| `geometry.tibia` | millimeters (`mm`) | meters (`m`) |
| `geometry.body_radius` | millimeters (`mm`) | meters (`m`) |

Conversion rule: `mm = m * 1000`.

### Joint payload (`type = "joints"`)

| Field | Unit on wire | Internal source unit |
|---|---|---|
| `angles_deg.<LEG>[0]` (coxa) | degrees (`deg`) | radians (`rad`) |
| `angles_deg.<LEG>[1]` (femur) | degrees (`deg`) | radians (`rad`) |
| `angles_deg.<LEG>[2]` (tibia) | degrees (`deg`) | radians (`rad`) |
| `timestamp_ms` | milliseconds (`ms`) since runtime epoch | microseconds (`us`) |

Conversion rules:
- `deg = rad * 180 / pi`
- `ms = us / 1000`

## Pose semantics (`autonomy_debug`) 

The visualiser reads world pose from these fields, in this order:

1. `autonomy_debug.current_pose`
2. `autonomy_debug.localization.current_pose` (only when `frame_id` is accepted; see frame rules below)
3. `dynamic_gait.current_pose` (legacy/optional source)
4. Fallback dead-reckoning from translation/velocity fields.

### `autonomy_debug.current_pose`

**Meaning:** best available absolute planar body pose (`x_m`, `y_m`, `yaw_rad`) in world coordinates for rendering map-anchored overlays and world grid motion.

**Producer behavior:**
- If localization pose is available, server copies that localization pose directly into `autonomy_debug.current_pose`.
- If localization pose is unavailable, server publishes fallback odometry pose in this field.

So, this field is always intended to be directly consumable by the visualiser, regardless of whether its source is localization or fallback odometry.

### `autonomy_debug.localization.current_pose`

**Meaning:** raw pose from localization module, with explicit `frame_id` attached.

**Consumer behavior:**
- The visualiser treats this as authoritative only when `frame_id` is accepted (`map` or `odom`).
- Unknown frames are preserved in telemetry state for debugging, but ignored for pose resolution in renderer logic.

This allows tests to assert both that unknown frames are retained in state and that rendering does not use them.

### `frame_id` values and handling rules

`autonomy_debug.localization.frame_id` is a string provided by telemetry producer.

| `frame_id` | Renderer handling | Notes |
|---|---|---|
| `map` | accepted for pose resolution | global map frame |
| `odom` | accepted for pose resolution | drifting local odometry frame |
| missing/empty | localization pose ignored; resolver falls through to next source | defensive behavior |
| any other value | localization pose ignored for rendering | kept in state for diagnostics |

Validation rule (testable): `frame_id` gating is implemented in `resolvePoseOffsetMm` before localization pose can be chosen.

## Fallback odometry semantics and limitations

When localization data is absent, fallback pose uses dead-reckoned integration in producer and/or renderer:

1. **Server fallback (`telemetry_publisher.cpp`)**
   - Integrates planar velocity into `odometry_pose_x_m_`, `odometry_pose_y_m_`.
   - Integrates yaw rate into `odometry_pose_yaw_rad_`.
   - Uses estimated planar velocity when non-zero finite values exist.
   - Otherwise falls back to commanded heading/speed.
   - Clamps integration timestep to `[0.0, 0.25]` seconds.

2. **Renderer fallback (`scene-renderer.js`)**
   - If no accepted absolute pose exists, attempts translation-derived pose.
   - If velocity and timestamp are available, integrates dead-reckoning in renderer-local state.
   - Also clamps timestep to `<= 0.25` s.

### Limitations (explicit)

- Fallback odometry is **not globally corrected** (no loop closure / map anchoring).
- Position and yaw can drift over time.
- Command-derived fallback (`speed + heading`) can diverge from physical motion (slip/compliance).
- Unknown localization frames will intentionally not override this fallback.

## Required vs optional fields

### Geometry payload

Required:
- `schema_version`
- `geometry` object

Optional:
- `type` (`"geometry"` recommended)
- any subset of geometry fields can be sent as partial update, but producer should send complete geometry at startup.
- `timestamp_ms` (accepted if present; not required for geometry updates).

### Joints payload

Required:
- `schema_version`
- `type` = `"joints"`
- `angles_deg` object with known leg keys

Optional:
- `timestamp_ms` (recommended)

Consumer only applies per-leg updates where value is a numeric 3-element joint array and leg key is one of `LF/LM/LR/RF/RM/RR`.

## Browser payload contract (`server.py -> static/app.js`)

WebSocket payloads emitted by `TelemetryState.to_payload` include:

| Field | Type | Required | Notes |
|---|---|---|---|
| `type` | string | yes | always `"state"` |
| `geometry` | object | yes | dictionary of geometry values in mm |
| `angles_deg` | object | yes | leg-key map to 3-element numeric arrays |
| `timestamp_ms` | integer or `null` | yes | `null` until a timestamp arrives |
| `active_mode` | string or `null` | yes | mode from upstream telemetry when present |
| `active_fault` | string or `null` | yes | active fault summary when present |
| `bus_ok` | boolean or `null` | yes | bus health when present |
| `estimator_valid` | boolean or `null` | yes | estimator health when present |
| `loop_counter` | integer or `null` | yes | control loop counter when present |
| `voltage` | number or `null` | yes | power bus voltage when present |
| `current` | number or `null` | yes | power bus current when present |
| `dynamic_gait` | object or `null` | yes | dynamic gait internals (family/region/fallback/envelope/per-leg phase+duty+stance) and body velocity vectors (`body_linear_velocity_mps`, `body_angular_velocity_radps`) for tuning + visual dead-reckoning |
| `autonomy_debug` | object or `null` | yes | optional autonomy mission-debug payload (waypoints, active waypoint index, current pose/localization) for route and mission visual overlays |

Optionality semantics:
- These fields are **always present in WebSocket payloads** but may be `null` until UDP telemetry includes a valid value.
- The UI must treat every status/health field as optional data.
- UDP parsing accepts these fields on any schema-v1 payload type (`geometry`, `joints`, or additive telemetry payloads) and merges them into the current state.

## Troubleshooting: grid moves wrong direction

Symptoms:
- Robot appears stationary while grid slides the wrong way.
- Waypoint overlay direction does not match expected motion.

Checks:
1. Confirm whether `autonomy_debug.current_pose` is present and finite.
2. If relying on localization, check `autonomy_debug.localization.frame_id` is exactly `map` or `odom`.
3. If frame is unknown, renderer will ignore localization and use fallback dead-reckoning.
4. Compare `dynamic_gait.body_linear_velocity_mps` vs commanded velocity fields; large mismatch can flip apparent motion under fallback.

Likely causes:
- Coordinate-frame mismatch in upstream localization (`frame_id` not in accepted set).
- Sign convention mismatch for yaw/velocity in upstream producer.
- Dead-reckoning drift from prolonged localization loss.

## Implementation references

- Producer and fallback odometry construction:
  - [`hexapod-server/src/control/telemetry/telemetry_publisher.cpp`](../hexapod-server/src/control/telemetry/telemetry_publisher.cpp)
- Telemetry sanitation and autonomy pose parsing:
  - [`hexapod-visualiser/telemetry_protocol.py`](../hexapod-visualiser/telemetry_protocol.py)
- Pose resolution, frame gating, and dead-reckoning integration in renderer:
  - [`hexapod-visualiser/static/scene-renderer.js`](../hexapod-visualiser/static/scene-renderer.js)

## Backward compatibility expectations

1. **Major schema change requires schema_version bump.**
   - Any incompatible wire-format change (field rename, unit change, semantic change, leg-key changes) must increment `schema_version`.
2. **Current consumer policy is strict schema gate.**
   - `hexapod-visualiser/server.py` accepts only `schema_version == 1` and ignores everything else with operator-visible warnings.
3. **Additive changes should preserve compatibility when possible.**
   - New optional fields may be added without version bump if existing fields and units are unchanged.
4. **Do not silently change units.**
   - Unit changes are breaking and must use a new schema version.
