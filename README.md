# Hexapod

Monorepo for a serial-controlled hexapod robot, containing Linux host control software, Servo 2040 firmware, and shared wire protocol definitions.

## Components

- **`hexapod-server/`** — Linux host application that loads robot config/calibrations, owns real-time control loops, and communicates with firmware over framed serial packets.
- **`hexapod-client/`** — Pimoroni Servo 2040 (RP2040) firmware that drives 18 servos, enforces host handshake/lifecycle state, and serves sensing + power commands.
- **`hexapod-common/`** — Shared protocol IDs, constants, and framing helpers used by both sides.
- **`docs/`** — Architecture notes, protocol details, hardware references, and maintenance roadmap.

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
│   ├── config.sim.txt
│   ├── include/
│   ├── src/
│   ├── scenarios/
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
2. Server opens the serial device (commonly `/dev/ttyACM0` at `115200`).
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

## Development workflow

Use this loop when iterating on protocol/control changes that affect both host and firmware:

1. Update protocol IDs/payloads in `hexapod-common/include/hexapod-common.hpp`.
2. Update firmware handlers in `hexapod-client/*_commands.cpp`.
3. Update host-side transport/control consumers in `hexapod-server/src/*`.
4. Run server tests in simulator mode before hardware testing.

Suggested verification commands:

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests --output-on-failure
```

Then run a scenario sweep:

```bash
cd hexapod-server
cp config.sim.txt config.txt
for s in scenarios/*.toml; do
  ./build/hexapod-server --scenario "$s" || break
done
```

## Documentation map

- `docs/FIRMWARE.md` — wire protocol framing, constants, and command payloads.
- `docs/HARDWARE.md` — mechanical/electrical build reference and dimensions.

## Safety notes

- Validate relay defaults, servo enable sequencing, and emergency stop behavior before full-body motion testing.
- Start with the robot unloaded and low-amplitude commands after calibration changes.
- Prefer simulator mode (`hexapod-server/config.sim.txt`) for early control-policy validation.
- Keep one hand on power disconnect / E-stop whenever first exercising new gait or calibration logic on hardware.
