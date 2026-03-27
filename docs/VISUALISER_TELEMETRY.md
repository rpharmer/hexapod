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
| `dynamic_gait` | object or `null` | yes | dynamic gait internals (family/region/fallback/envelope/per-leg phase+duty+stance) for tuning + safety validation |
| `autonomy_debug` | object or `null` | yes | optional autonomy mission-debug payload (waypoints, active waypoint index, current pose) for route/mission visual overlays |

Optionality semantics:
- These fields are **always present in WebSocket payloads** but may be `null` until UDP telemetry includes a valid value.
- The UI must treat every status/health field as optional data.
- UDP parsing accepts these fields on any schema-v1 payload type (`geometry`, `joints`, or additive telemetry payloads) and merges them into the current state.

## Backward compatibility expectations

1. **Major schema change requires schema_version bump.**
   - Any incompatible wire-format change (field rename, unit change, semantic change, leg-key changes) must increment `schema_version`.
2. **Current consumer policy is strict schema gate.**
   - `hexapod-visualiser/server.py` accepts only `schema_version == 1` and ignores everything else with operator-visible warnings.
3. **Additive changes should preserve compatibility when possible.**
   - New optional fields may be added without version bump if existing fields and units are unchanged.
4. **Do not silently change units.**
   - Unit changes are breaking and must use a new schema version.
