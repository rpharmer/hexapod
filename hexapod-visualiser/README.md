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

In another terminal, run the sample data stream:

```bash
python simulate_telemetry.py --host 127.0.0.1 --port 9870 --hz 30
```

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

### Health / metrics endpoint

The server exposes a lightweight JSON diagnostics endpoint.

- Default path: `/healthz`
- Change path: `--metrics-path /metrics`

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
