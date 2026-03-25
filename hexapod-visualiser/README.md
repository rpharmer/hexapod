# hexapod-visualiser

A lightweight browser visualiser for the hexapod.

It receives geometry + joint angles over **UDP** (easy for `hexapod-server` to publish from C++) and forwards the latest state to browser clients over WebSocket.

## Quick start

```bash
cd hexapod-visualiser
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server.py --http-port 8080 --udp-port 9870
```

Open: `http://localhost:8080`

Diagnostics endpoint (default): `http://localhost:8080/healthz`

In another terminal, run the sample data stream:

```bash
python simulate_telemetry.py --host 127.0.0.1 --port 9870 --hz 30
```

## Visualiser camera usage

- **Orbit:** left-click + drag on the canvas.
- **Pan:** right-click (or Shift + left-click) + drag.
- **Zoom:** mouse wheel / trackpad scroll.
- **Presets:** use **Top**, **Front**, **Side**, or **Reset View** in the header for deterministic camera poses.
## Operational diagnostics

The backend now emits structured logs with a consistent prefix:

- Prefix: `[visualiser]`
- Required fields: `level=<...> event=<...>`
- Additional context fields are logged as key/value pairs (for example: UDP source address, schema mismatch details, active client count).

### Counters tracked

- `udp_received`: total UDP datagrams received by the server.
- `udp_rejected`: UDP datagrams rejected due to invalid JSON, non-object payloads, or schema mismatch.
- `ws_clients_connected`: current number of active WebSocket clients.
- `ws_send_failures`: number of WebSocket broadcast send failures.
- `last_udp_update_utc`: UTC timestamp of the most recent accepted UDP update that changed state.

### Periodic stats logs

By default, a periodic structured `event=periodic_stats` line is emitted every 30 seconds.

- Change interval: `--stats-log-interval <seconds>`
- Disable periodic logs: `--stats-log-interval 0`

### Broadcast coalescing / max publish rate

WebSocket broadcasts are coalesced (many UDP packets can collapse into one outbound push), while always sending the latest merged state.

- Default max push rate: `25 Hz`
- Change max rate: `--max-broadcast-hz <hz>`

### Health / metrics endpoint

The server exposes a lightweight JSON diagnostics endpoint.

- Default path: `/healthz`
- Alternate path example: `--metrics-path /metrics`

Example response shape:

```json
{
  "status": "ok",
  "telemetry_timestamp_ms": 1712345678901,
  "generated_at_utc": "2026-03-24T00:00:00.000000Z",
  "diagnostics": {
    "udp_received": 1200,
    "udp_rejected": 3,
    "ws_clients_connected": 1,
    "ws_send_failures": 0,
    "last_udp_update_utc": "2026-03-24T00:00:00.000000Z"
  }
}
```

## Telemetry format (UDP JSON)

You can send partial updates or full state.

### Geometry update

```json
{
  "schema_version": 1,
  "geometry": {
    "coxa": 35.0,
    "femur": 70.0,
    "tibia": 110.0,
    "body_radius": 60.0
  }
}
```

### Joint angles update

```json
{
  "schema_version": 1,
  "type": "joints",
  "timestamp_ms": 1712345678901,
  "angles_deg": {
    "LF": [0.0, 20.0, -40.0],
    "LM": [0.0, 20.0, -40.0],
    "LR": [0.0, 20.0, -40.0],
    "RF": [0.0, 20.0, -40.0],
    "RM": [0.0, 20.0, -40.0],
    "RR": [0.0, 20.0, -40.0]
  }
}
```

Leg keys expected: `LF`, `LM`, `LR`, `RF`, `RM`, `RR`.
`schema_version` is required and must currently be `1`.

## Integration idea for `hexapod-server`

From the server loop, serialize latest geometry once at startup, then send joint angles each control cycle as UDP datagrams to the visualiser host/port.

This keeps visualisation decoupled from control timing and avoids adding heavy UI dependencies into the control process.

## Parser tests

Run parser/merge behavior tests for `TelemetryState` + `UdpTelemetryProtocol`:

```bash
cd hexapod-visualiser
python -m unittest tests/test_udp_parser.py
```

## End-to-end smoke flow

From repository root, run a smoke test that:

1. starts the visualiser backend,
2. sends a canonical UDP telemetry packet,
3. verifies WebSocket state contains expected fields.

```bash
python scripts/visualiser_smoke.py
```


## Capture and replay workflow

Use capture tooling from repository root (`scripts/`) to record raw UDP JSON traffic and replay it against the visualiser.

### Capture UDP datagrams to NDJSON

```bash
python scripts/capture_udp_telemetry.py \
  --host 0.0.0.0 \
  --port 9870 \
  --output scripts/fixtures/session.ndjson
```

Useful options:
- `--max-frames N`: stop capture after `N` packets.

Each NDJSON line contains capture timestamps (`captured_at_unix_ms` / `captured_at_unix_ns`), source address, raw payload text, and decoded JSON (`datagram`).

### Replay captured frames with timing control

```bash
python scripts/replay_udp_telemetry.py \
  --input scripts/fixtures/session.ndjson \
  --host 127.0.0.1 \
  --port 9870 \
  --speed 1.0
```

Useful replay options:
- `--speed`: timing scale (`1.0` = original capture timing, `2.0` = 2x faster, `0.5` = slower).
- `--loop`: repeat capture continuously.
- `--start-offset-s`: skip to an offset from the beginning of the recording.
- `--host` / `--port`: destination visualiser endpoint.

### Smoke check path (fixture replay + WebSocket assertion)

Run from repository root:

```bash
python scripts/visualiser_smoke.py --mode replay
```

This starts the visualiser server, replays `scripts/fixtures/visualiser_smoke.ndjson`, and validates that a matching WebSocket `state` update is observed.

You can still use direct single-packet mode:

```bash
python scripts/visualiser_smoke.py --mode direct
```
