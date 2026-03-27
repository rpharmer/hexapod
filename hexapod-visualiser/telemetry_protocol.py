from __future__ import annotations

import asyncio
import json
from typing import Any, Callable

from runtime_support import Diagnostics, GEOMETRY_KEYS, LEG_KEYS, TelemetryState, log_event

EXPECTED_SCHEMA_VERSION = 1


class UdpTelemetryProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: TelemetryState, diagnostics: Diagnostics, on_update: Callable[[], None]):
        self.state = state
        self.diagnostics = diagnostics
        self.on_update = on_update

    def datagram_received(self, data: bytes, addr):
        self.diagnostics.udp_received += 1
        try:
            message = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.diagnostics.udp_rejected += 1
            log_event("warning", "udp_rejected", reason="invalid_json", addr=addr)
            return

        if not isinstance(message, dict):
            self.diagnostics.udp_rejected += 1
            log_event("warning", "udp_rejected", reason="payload_not_object", addr=addr)
            return

        schema_version = self._resolve_schema_version(message)
        if schema_version != EXPECTED_SCHEMA_VERSION:
            self.diagnostics.udp_rejected += 1
            log_event(
                "warning",
                "udp_rejected",
                reason="schema_version_mismatch",
                received_schema=schema_version,
                expected_schema=EXPECTED_SCHEMA_VERSION,
                addr=addr,
            )
            return

        incoming_timestamp_ms = message.get("timestamp_ms") if isinstance(message.get("timestamp_ms"), int) else None
        if self._is_stale_timestamp(incoming_timestamp_ms):
            self.diagnostics.udp_rejected += 1
            log_event(
                "debug",
                "udp_dropped_stale_timestamp",
                incoming_timestamp_ms=incoming_timestamp_ms,
                last_timestamp_ms=self.state.timestamp_ms,
                addr=addr,
            )
            return

        changed = False

        geometry = message.get("geometry")
        if isinstance(geometry, dict):
            merged = dict(self.state.geometry)
            merged.update(self._sanitize_geometry_update(geometry, source="geometry"))
            self.state.geometry = merged
            changed = True

        if message.get("type") == "geometry":
            merged = dict(self.state.geometry)
            merged.update(self._sanitize_geometry_update(message, source="legacy_geometry"))
            self.state.geometry = merged
            changed = True

        angles = message.get("angles_deg")
        if isinstance(angles, dict):
            sanitized: dict[str, list[float]] = {}
            for leg_name, values in angles.items():
                if (
                    leg_name in LEG_KEYS
                    and isinstance(values, list)
                    and len(values) == 3
                    and all(_is_number(v) for v in values)
                ):
                    sanitized[leg_name] = [float(v) for v in values]
            if sanitized:
                merged_angles = dict(self.state.angles_deg)
                merged_angles.update(sanitized)
                self.state.angles_deg = merged_angles
                changed = True

        changed = self._apply_common_state_fields(message) or changed

        if changed:
            self.diagnostics.mark_udp_update()
            self.on_update()

    def _is_stale_timestamp(self, incoming_timestamp_ms: int | None) -> bool:
        if incoming_timestamp_ms is None:
            return False
        if not isinstance(self.state.timestamp_ms, int):
            return False
        return incoming_timestamp_ms < self.state.timestamp_ms

    def _resolve_schema_version(self, message: dict[str, Any]) -> int | None:
        schema_version = message.get("schema_version")
        if isinstance(schema_version, int):
            return schema_version

        # Backward compatibility: older server builds did not include schema_version.
        # Treat recognised legacy packet layouts as schema v1 to keep the visualiser usable.
        packet_type = message.get("type")
        looks_like_legacy_geometry = isinstance(message.get("geometry"), dict)
        looks_like_legacy_joints = packet_type == "joints" and isinstance(message.get("angles_deg"), dict)
        if looks_like_legacy_geometry or looks_like_legacy_joints:
            return EXPECTED_SCHEMA_VERSION
        return None

    def _apply_common_state_fields(self, message: dict[str, Any]) -> bool:
        changed = False

        if isinstance(message.get("timestamp_ms"), int):
            self.state.timestamp_ms = int(message["timestamp_ms"])
            changed = True

        if isinstance(message.get("active_mode"), str):
            self.state.active_mode = message["active_mode"]
            changed = True

        if isinstance(message.get("active_fault"), str):
            self.state.active_fault = message["active_fault"]
            changed = True

        if isinstance(message.get("bus_ok"), bool):
            self.state.bus_ok = message["bus_ok"]
            changed = True

        if isinstance(message.get("estimator_valid"), bool):
            self.state.estimator_valid = message["estimator_valid"]
            changed = True

        if isinstance(message.get("loop_counter"), int):
            self.state.loop_counter = int(message["loop_counter"])
            changed = True

        if _is_number(message.get("voltage")):
            self.state.voltage = float(message["voltage"])
            changed = True

        if _is_number(message.get("current")):
            self.state.current = float(message["current"])
            changed = True

        if isinstance(message.get("dynamic_gait"), dict):
            self.state.dynamic_gait = message["dynamic_gait"]
            changed = True

        autonomy_debug = self._sanitize_autonomy_debug(message.get("autonomy_debug"))
        if autonomy_debug is not None:
            self.state.autonomy_debug = autonomy_debug
            changed = True

        return changed

    def _sanitize_geometry_update(self, geometry: dict[str, Any], *, source: str) -> dict[str, float]:
        sanitized: dict[str, float] = {}
        unknown_keys = sorted(
            key for key in geometry.keys() if key in {"schema_version", "type"} or key not in GEOMETRY_KEYS
        )
        if unknown_keys:
            log_event("debug", "geometry_unknown_keys_ignored", source=source, keys=unknown_keys)

        for key in GEOMETRY_KEYS:
            value = geometry.get(key)
            if _is_number(value):
                sanitized[key] = float(value)
        return sanitized

    def _sanitize_autonomy_debug(self, value: Any) -> dict[str, Any] | None:
        if not isinstance(value, dict):
            return None

        waypoints_raw = value.get("waypoints")
        waypoints: list[dict[str, float]] = []
        if isinstance(waypoints_raw, list):
            for item in waypoints_raw:
                if not isinstance(item, dict):
                    continue
                x_m = item.get("x_m")
                y_m = item.get("y_m")
                if not (_is_number(x_m) and _is_number(y_m)):
                    continue
                waypoint = {"x_m": float(x_m), "y_m": float(y_m)}
                yaw_rad = item.get("yaw_rad")
                if _is_number(yaw_rad):
                    waypoint["yaw_rad"] = float(yaw_rad)
                waypoints.append(waypoint)

        active_waypoint_index = value.get("active_waypoint_index")
        pose_payload = self._sanitize_pose_payload(value.get("current_pose"))
        localization_payload = self._sanitize_localization_payload(value.get("localization"))

        out: dict[str, Any] = {"waypoints": waypoints}
        if isinstance(active_waypoint_index, int):
            out["active_waypoint_index"] = int(active_waypoint_index)
        if pose_payload is not None:
            out["current_pose"] = pose_payload
        if localization_payload is not None:
            out["localization"] = localization_payload
        return out

    def _sanitize_pose_payload(self, value: Any) -> dict[str, float] | None:
        if not isinstance(value, dict):
            return None
        px = value.get("x_m") if _is_number(value.get("x_m")) else value.get("x")
        py = value.get("y_m") if _is_number(value.get("y_m")) else value.get("y")
        if not (_is_number(px) and _is_number(py)):
            return None
        pose_payload: dict[str, float] = {"x_m": float(px), "y_m": float(py)}
        yaw = value.get("yaw_rad") if _is_number(value.get("yaw_rad")) else value.get("yaw")
        if not _is_number(yaw):
            yaw = value.get("z")
        if _is_number(yaw):
            pose_payload["yaw_rad"] = float(yaw)
        return pose_payload

    def _sanitize_localization_payload(self, value: Any) -> dict[str, Any] | None:
        if not isinstance(value, dict):
            return None
        pose_payload = self._sanitize_pose_payload(value.get("current_pose"))
        if pose_payload is None:
            return None
        return {"current_pose": pose_payload}


def _is_number(value: Any) -> bool:
    return isinstance(value, (float, int)) and not isinstance(value, bool)


async def start_udp_listener(
    loop: asyncio.AbstractEventLoop,
    state: TelemetryState,
    diagnostics: Diagnostics,
    port: int,
    on_update: Callable[[], None],
) -> None:
    await loop.create_datagram_endpoint(
        lambda: UdpTelemetryProtocol(state, diagnostics, on_update),
        local_addr=("0.0.0.0", port),
    )
