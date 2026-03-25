from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Awaitable, Callable


DEFAULT_ANGLES = {
    "LF": [0.0, 10.0, -25.0],
    "LM": [0.0, 10.0, -25.0],
    "LR": [0.0, 10.0, -25.0],
    "RF": [0.0, 10.0, -25.0],
    "RM": [0.0, 10.0, -25.0],
    "RR": [0.0, 10.0, -25.0],
}
LEG_KEYS = frozenset(DEFAULT_ANGLES.keys())

DEFAULT_GEOMETRY = {
    "coxa": 35.0,
    "femur": 70.0,
    "tibia": 110.0,
    "body_radius": 60.0,
}
GEOMETRY_KEYS = frozenset(DEFAULT_GEOMETRY.keys())


def utc_now_iso() -> str:
    return datetime.now(tz=timezone.utc).isoformat().replace("+00:00", "Z")


def log_event(level: str, event: str, **context: Any) -> None:
    pairs = " ".join(f"{key}={value!r}" for key, value in sorted(context.items()))
    suffix = f" {pairs}" if pairs else ""
    print(f"[visualiser] level={level} event={event}{suffix}")


class CoalescingUpdateScheduler:
    """Coalesces update notifications and publishes at a bounded rate."""

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        publish_coro: Callable[[], Awaitable[None]],
        *,
        publish_hz: float = 25.0,
        log_every: int = 200,
    ):
        if publish_hz <= 0:
            raise ValueError("publish_hz must be > 0")

        self.loop = loop
        self.publish_coro = publish_coro
        self.publish_interval_s = 1.0 / publish_hz
        self.log_every = log_every

        self._dirty = False
        self._runner_task: asyncio.Task | None = None

        self.update_notifications = 0
        self.publish_cycles = 0
        self.coalesced_notifications = 0

    def notify_update(self) -> None:
        self.update_notifications += 1
        if self._dirty:
            self.coalesced_notifications += 1
            self._maybe_log_coalescing()
        self._dirty = True

        if self._runner_task is None or self._runner_task.done():
            self._runner_task = self.loop.create_task(self._run())

    def _maybe_log_coalescing(self) -> None:
        if self.coalesced_notifications and self.coalesced_notifications % self.log_every == 0:
            log_event(
                "info",
                "udp_coalesced",
                coalesced_notifications=self.coalesced_notifications,
                publish_cycles=self.publish_cycles,
                notifications=self.update_notifications,
            )

    async def _run(self) -> None:
        try:
            while self._dirty:
                self._dirty = False
                await self.publish_coro()
                self.publish_cycles += 1
                await asyncio.sleep(self.publish_interval_s)
        finally:
            self._runner_task = None


@dataclass
class TelemetryState:
    geometry: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_GEOMETRY))
    angles_deg: dict[str, list[float]] = field(default_factory=lambda: dict(DEFAULT_ANGLES))
    timestamp_ms: int | None = None
    active_mode: str | None = None
    active_fault: str | None = None
    bus_ok: bool | None = None
    estimator_valid: bool | None = None
    loop_counter: int | None = None
    voltage: float | None = None
    current: float | None = None

    def to_payload(self) -> dict[str, Any]:
        return {
            "type": "state",
            "geometry": self.geometry,
            "angles_deg": self.angles_deg,
            "timestamp_ms": self.timestamp_ms,
            "active_mode": self.active_mode,
            "active_fault": self.active_fault,
            "bus_ok": self.bus_ok,
            "estimator_valid": self.estimator_valid,
            "loop_counter": self.loop_counter,
            "voltage": self.voltage,
            "current": self.current,
        }


@dataclass
class Diagnostics:
    udp_received: int = 0
    udp_rejected: int = 0
    ws_clients_connected: int = 0
    ws_send_failures: int = 0
    last_udp_update_utc: str | None = None

    def mark_udp_update(self) -> None:
        self.last_udp_update_utc = utc_now_iso()

    def snapshot(self) -> dict[str, Any]:
        return {
            "udp_received": self.udp_received,
            "udp_rejected": self.udp_rejected,
            "ws_clients_connected": self.ws_clients_connected,
            "ws_send_failures": self.ws_send_failures,
            "last_udp_update_utc": self.last_udp_update_utc,
        }
