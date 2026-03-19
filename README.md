# Hexapod

Monorepo for a serial-controlled hexapod robot, containing Linux host control software, Servo 2040 firmware, and shared wire protocol definitions.

## Components

- **`hexapod-server/`** — Linux host application that loads robot config/calibrations, owns real-time control loops, and communicates with firmware over framed serial packets.
- **`hexapod-client/`** — Pimoroni Servo 2040 (RP2040) firmware that drives 18 servos, enforces host handshake/lifecycle state, and serves sensing + power commands.
- **`hexapod-common/`** — Shared protocol IDs, constants, and framing helpers used by both sides.

## Repository layout

```text
hexapod/
├── README.md
├── docs/
│   ├── CODEBASE_REVIEW.md
│   ├── REFACTORING_REVIEW.md
│   ├── NEXT_STEPS.md
│   ├── FIRMWARE.md
│   └── HARDWARE.md
├── hexapod-common/
│   ├── include/
│   │   ├── framing.hpp
│   │   └── hexapod-common.hpp
│   └── framing.cpp
├── hexapod-server/
│   ├── config.txt
│   ├── include/
│   ├── src/
│   └── README.md
└── hexapod-client/
    ├── firmware_boot.cpp
    ├── command_dispatch.cpp
    ├── motion_commands.cpp
    ├── sensing_commands.cpp
    ├── power_commands.cpp
    ├── firmware_context.cpp
    ├── firmware_context.hpp
    ├── serialCommsClient.cpp
    ├── serialCommsClient.hpp
    └── README.md
```

## End-to-end communication flow

1. Server parses `hexapod-server/config.txt`.
2. Server opens the serial device (commonly `/dev/ttyACM0`, `115200`).
3. Server sends `HELLO` with protocol metadata.
4. Firmware responds with `ACK` or `NACK`.
5. Server uploads calibration pairs for all 18 joints.
6. Runtime loop uses heartbeat + motion/sensing commands (`SET_JOINT_TARGETS`, `GET_FULL_HARDWARE_STATE`, etc.).

Protocol source of truth:

- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/include/framing.hpp`
- `docs/FIRMWARE.md`

## Quick start

### Host (server)

```bash
cd hexapod-server
cmake -S . -B build
cmake --build build -j
./build/hexapod-server
```

### Firmware (client)

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Flash by copying `hexapod-client/build/hexapod-client.uf2` to the board in BOOTSEL mode.

## Development notes

- Start with subsystem READMEs for local build/run details.
- Update `hexapod-server/config.txt` carefully: calibration ordering and bounds must stay valid.
- Validate safety behavior (relay default state, servo enable sequencing, E-stop path) before full-body motion testing.

## Project planning docs

- `docs/CODEBASE_REVIEW.md`
- `docs/REFACTORING_REVIEW.md`
- `docs/NEXT_STEPS.md`
