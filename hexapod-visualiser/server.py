#!/usr/bin/env python3
"""Bridge UDP telemetry from hexapod-server to browser clients over WebSocket."""

from __future__ import annotations

import argparse
import asyncio
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web


DEFAULT_ANGLES = {
    "LF": [0.0, 10.0, -25.0],
    "LM": [0.0, 10.0, -25.0],
    "LR": [0.0, 10.0, -25.0],
    "RF": [0.0, 10.0, -25.0],
    "RM": [0.0, 10.0, -25.0],
    "RR": [0.0, 10.0, -25.0],
}
EXPECTED_SCHEMA_VERSION = 1
LEG_KEYS = frozenset(DEFAULT_ANGLES.keys())

DEFAULT_GEOMETRY = {
    "coxa": 35.0,
    "femur": 70.0,
    "tibia": 110.0,
    "body_radius": 60.0,
}


class CoalescingUpdateScheduler:
    """Coalesces update notifications and publishes at a bounded rate."""

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        publish_coro,
        *,
        publish_hz: float = 25.0,
        log_every: int = 200,
    ):
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
            print(
                "[visualiser] info: coalesced notifications="
                f"{self.coalesced_notifications}, publish_cycles={self.publish_cycles}, "
                f"notifications={self.update_notifications}"
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

    def to_payload(self) -> dict[str, Any]:
        return {
            "type": "state",
            "geometry": self.geometry,
            "angles_deg": self.angles_deg,
            "timestamp_ms": self.timestamp_ms,
        }


class UdpTelemetryProtocol(asyncio.DatagramProtocol):
    def __init__(self, state: TelemetryState, on_update):
        self.state = state
        self.on_update = on_update

    def datagram_received(self, data: bytes, addr):
        try:
            message = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        if not isinstance(message, dict):
            return

        schema_version = message.get("schema_version")
        if schema_version != EXPECTED_SCHEMA_VERSION:
            print(
                f"[visualiser] warning: ignoring UDP payload with incompatible schema_version="
                f"{schema_version!r} from {addr}; expected {EXPECTED_SCHEMA_VERSION}"
            )
            return

        changed = False

        geometry = message.get("geometry")
        if isinstance(geometry, dict):
            merged = dict(self.state.geometry)
            merged.update({k: float(v) for k, v in geometry.items() if _is_number(v)})
            self.state.geometry = merged
            changed = True

        if message.get("type") == "geometry":
            merged = dict(self.state.geometry)
            merged.update({k: float(v) for k, v in message.items() if _is_number(v)})
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

        if message.get("type") == "joints":
            if isinstance(message.get("timestamp_ms"), int):
                self.state.timestamp_ms = int(message["timestamp_ms"])
                changed = True

        if isinstance(message.get("timestamp_ms"), int):
            self.state.timestamp_ms = int(message["timestamp_ms"])
            changed = True

        if changed:
            self.on_update()


def _is_number(value: Any) -> bool:
    return isinstance(value, (float, int)) and not isinstance(value, bool)


async def start_udp_listener(loop, state: TelemetryState, port: int, on_update):
    await loop.create_datagram_endpoint(
        lambda: UdpTelemetryProtocol(state, on_update),
        local_addr=("0.0.0.0", port),
    )


async def make_app(state: TelemetryState) -> web.Application:
    app = web.Application()
    clients: set[web.WebSocketResponse] = set()

    async def broadcast_state() -> None:
        if not clients:
            return
        payload = json.dumps(state.to_payload())
        stale_clients: list[web.WebSocketResponse] = []
        for client in clients:
            try:
                await client.send_str(payload)
            except ConnectionResetError:
                stale_clients.append(client)
        for client in stale_clients:
            clients.discard(client)

    app["broadcast_state"] = broadcast_state

    static_dir = Path(__file__).resolve().parent / "static"

    async def index(_: web.Request) -> web.Response:
        return web.FileResponse(static_dir / "index.html")

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        clients.add(ws)

        await ws.send_str(json.dumps(state.to_payload()))

        async for message in ws:
            if message.type == WSMsgType.TEXT and message.data == "ping":
                await ws.send_str("pong")
            elif message.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                break

        clients.discard(ws)
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/", static_dir, show_index=True)
    return app


async def main() -> None:
    parser = argparse.ArgumentParser(description="Hexapod visualiser UDP -> WebSocket bridge")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP/WebSocket port")
    parser.add_argument("--udp-port", type=int, default=9870, help="UDP telemetry input port")
    args = parser.parse_args()

    state = TelemetryState()
    app = await make_app(state)

    async def on_update_async() -> None:
        await app["broadcast_state"]()

    update_scheduler = CoalescingUpdateScheduler(
        asyncio.get_running_loop(),
        on_update_async,
        publish_hz=25.0,
    )

    await start_udp_listener(asyncio.get_running_loop(), state, args.udp_port, update_scheduler.notify_update)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.http_port)
    await site.start()

    print(f"Visualiser running: http://localhost:{args.http_port} | UDP input: {args.udp_port}")
    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
