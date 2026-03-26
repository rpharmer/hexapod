#!/usr/bin/env python3
"""Bridge UDP telemetry from hexapod-server to browser clients over WebSocket."""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

from aiohttp import WSMsgType, web

from runtime_support import CoalescingUpdateScheduler, Diagnostics, TelemetryState, log_event, utc_now_iso
from telemetry_protocol import UdpTelemetryProtocol, start_udp_listener


async def make_app(state: TelemetryState, diagnostics: Diagnostics, metrics_path: str) -> web.Application:
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
            except Exception as exc:  # noqa: BLE001
                diagnostics.ws_send_failures += 1
                log_event("warning", "ws_send_failed", error=type(exc).__name__)
                stale_clients.append(client)
        for client in stale_clients:
            clients.discard(client)
        diagnostics.ws_clients_connected = len(clients)

    app["broadcast_state"] = broadcast_state

    static_dir = Path(__file__).resolve().parent / "static"

    async def index(_: web.Request) -> web.Response:
        return web.FileResponse(static_dir / "index.html")

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        clients.add(ws)
        diagnostics.ws_clients_connected = len(clients)
        log_event("info", "ws_connected", client_count=diagnostics.ws_clients_connected)

        # Preserve an immediate full state push on initial connection.
        await ws.send_str(json.dumps(state.to_payload()))

        async for message in ws:
            if message.type == WSMsgType.TEXT and message.data == "ping":
                await ws.send_str("pong")
            elif message.type in (WSMsgType.CLOSE, WSMsgType.ERROR):
                break

        clients.discard(ws)
        diagnostics.ws_clients_connected = len(clients)
        log_event("info", "ws_disconnected", client_count=diagnostics.ws_clients_connected)
        return ws

    async def metrics_handler(_: web.Request) -> web.Response:
        return web.json_response(
            {
                "status": "ok",
                "telemetry_timestamp_ms": state.timestamp_ms,
                "generated_at_utc": utc_now_iso(),
                "diagnostics": diagnostics.snapshot(),
            }
        )

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get(metrics_path, metrics_handler)
    app.router.add_static("/", static_dir, show_index=True)
    return app


async def main() -> None:
    parser = argparse.ArgumentParser(description="Hexapod visualiser UDP -> WebSocket bridge")
    parser.add_argument("--http-port", type=int, default=8080, help="HTTP/WebSocket port")
    parser.add_argument("--udp-port", type=int, default=9870, help="UDP telemetry input port")
    parser.add_argument(
        "--metrics-path",
        type=str,
        default="/healthz",
        help="HTTP path for lightweight diagnostics JSON endpoint",
    )
    parser.add_argument(
        "--max-broadcast-hz",
        type=float,
        default=50.0,
        help="Maximum websocket broadcast frequency in Hz (latest update wins)",
    )
    parser.add_argument(
        "--stats-log-interval",
        type=float,
        default=30.0,
        help="Seconds between periodic diagnostics logs (0 disables periodic logs)",
    )
    args = parser.parse_args()

    state = TelemetryState()
    diagnostics = Diagnostics()
    app = await make_app(state, diagnostics, args.metrics_path)

    async def on_update_async() -> None:
        await app["broadcast_state"]()

    update_scheduler = CoalescingUpdateScheduler(
        asyncio.get_running_loop(),
        on_update_async,
        publish_hz=args.max_broadcast_hz,
    )

    await start_udp_listener(
        asyncio.get_running_loop(),
        state,
        diagnostics,
        args.udp_port,
        update_scheduler.notify_update,
    )

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.http_port)
    await site.start()

    async def periodic_stats_logger() -> None:
        if args.stats_log_interval <= 0:
            return
        while True:
            await asyncio.sleep(args.stats_log_interval)
            log_event("info", "periodic_stats", **diagnostics.snapshot())

    asyncio.create_task(periodic_stats_logger())
    log_event(
        "info",
        "startup",
        http_url=f"http://localhost:{args.http_port}",
        udp_port=args.udp_port,
        metrics_path=args.metrics_path,
        max_broadcast_hz=args.max_broadcast_hz,
        stats_log_interval=args.stats_log_interval,
    )
    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
