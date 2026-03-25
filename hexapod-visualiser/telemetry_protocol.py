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

        schema_version = message.get("schema_version")
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
