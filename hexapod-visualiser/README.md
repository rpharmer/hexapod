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

## Telemetry format (UDP JSON)

You can send partial updates or full state.

### Geometry update

```json
{
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

## Integration idea for `hexapod-server`

From the server loop, serialize latest geometry once at startup, then send joint angles each control cycle as UDP datagrams to the visualiser host/port.

This keeps visualisation decoupled from control timing and avoids adding heavy UI dependencies into the control process.
