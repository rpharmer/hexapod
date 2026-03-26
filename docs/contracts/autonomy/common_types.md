# Autonomy Contracts: Common Types & Conventions (v1)

Status: **Draft for v1 freeze**

This document defines shared conventions across:
`mission_executive`, `navigation_manager`, `recovery_manager`, `motion_arbiter`, `localization`, `world_model`, `traversability_analyzer`, and `locomotion_interface`.

## 1) Versioning Policy

- **Major (`v1`, `v2`, ...):** breaking schema/semantic changes.
- **Minor (`v1.x`):** additive, backward-compatible fields/messages.
- Producers MUST emit `contract_version`.
- Consumers MUST reject newer major versions and SHOULD tolerate unknown additive fields within the same major.

## 2) Units

- Use SI units unless stated otherwise:
  - distance: `m`
  - velocity: `m/s`
  - acceleration: `m/s^2`
  - angular rates: `rad/s`
  - angles: `rad`
  - time intervals: `ms`
- Contract fields SHOULD include unit suffixes where ambiguity exists (e.g. `x_m`, `yaw_rad`, `age_ms`).

## 3) Frames

Canonical frame identifiers:
- `map`: global/world-fixed frame
- `odom`: locally continuous drift frame
- `base_link`: robot body frame
- `sensor_*`: sensor-local frames (e.g. `sensor_imu`, `sensor_depth`)

Rules:
- Every pose/twist/path message MUST include `frame_id`.
- Transform source/target frames MUST be explicit for all frame-conversion APIs.
- Frame-less geometric data is invalid.

## 4) Time, IDs, and Correlation

- `timestamp_ms`: monotonic clock in milliseconds from runtime start.
- `utc_timestamp_ms` (optional): wall-clock for logs/operator UX.
- `sample_id`: strictly increasing per data stream.
- `correlation_id`: carried across mission->nav->planner->arbiter events for traceability.

Staleness baseline classes:
- **hard realtime control inputs:** stale if `age_ms > 100`
- **navigation/status streams:** stale if `age_ms > 500`
- **world/traversability:** stale if `age_ms > 1000`

Module-specific contracts may tighten these limits.

## 5) QoS Classes

- `QOS_CRITICAL`: no drop policy in-process, strict freshness checks.
- `QOS_CONTROL`: bounded queue, newest-wins behavior allowed.
- `QOS_STATUS`: lossy acceptable, periodic snapshot semantics.
- `QOS_BULK`: map/layer transport where latency is acceptable.

All subscriptions MUST declare:
- QoS class
- max tolerated age (`max_age_ms`)
- behavior on stale/missing data (`HOLD`, `SAFE_STOP`, `DEGRADE`)

## 6) Fault Severity Semantics

Severity levels:
- `INFO`: no action required
- `WARN`: degraded behavior allowed
- `ERROR`: module must transition to safe degraded mode
- `FATAL`: immediate stop/abort path

Required fault fields:
- `fault_code`
- `severity`
- `timestamp_ms`
- `origin_module`
- `details` (human-readable)
- `correlation_id` (if linked to a mission/nav action)
