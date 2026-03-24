# Hexapod

Monorepo for a serial-controlled hexapod robot, containing Linux host control software, Servo 2040 firmware, and shared protocol definitions.

## Components

- **`hexapod-server/`** — Linux host application that loads calibration/configuration, runs real-time control loops, and communicates with firmware over framed serial packets.
- **`hexapod-client/`** — Pimoroni Servo 2040 (RP2040) firmware that drives 18 servos, handles handshake/lifecycle state, and serves sensing + power commands.
- **`hexapod-common/`** — shared protocol constants, IDs, and framing helpers used by both host and firmware.
- **`docs/`** — hardware and firmware deep-dive notes.

## Repository layout

```text
hexapod/
├── README.md
├── docs/
│   ├── FIRMWARE.md
│   └── HARDWARE.md
├── hexapod-common/
│   ├── include/
│   │   ├── framing.hpp
│   │   ├── hexapod-common.hpp
│   │   └── protocol_codec.hpp
│   └── framing.cpp
├── hexapod-server/
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   ├── config.txt
│   ├── config.sim.txt
│   ├── include/
│   ├── scenarios/
│   ├── src/
│   ├── tests/
│   └── README.md
└── hexapod-client/
    ├── CMakeLists.txt
    ├── firmware_boot.cpp
    ├── command_dispatch.cpp
    ├── command_router.cpp
    ├── motion_commands.cpp
    ├── sensing_commands.cpp
    ├── power_commands.cpp
    ├── serialCommsClient.cpp
    ├── tests/
    └── README.md
```

## End-to-end communication flow

1. Server parses `hexapod-server/config.txt`.
2. Server opens the serial device (commonly `/dev/ttyACM0` at `115200`).
3. Server sends `HELLO` with protocol metadata.
4. Firmware responds with `ACK` or `NACK`.
5. Server uploads calibration pairs for all 18 joints.
6. Runtime loop exchanges heartbeat, motion, power, and sensing commands.

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

### Firmware host tests (native)

```bash
cd hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

## Unified verification entrypoint

Run the default repository quality gates from the repository root:

```bash
./scripts/verify.sh
```

This single command runs, in order (fail-fast):

1. `hexapod-server` CMake test preset (`tests`)
2. `hexapod-client` firmware host-test preset (`host-tests`)
3. Lightweight server scenario smoke (`scenarios/01_nominal_stand_walk.toml`) when available

Each section prints clear markers so CI and local runs can quickly identify where failures occur.

## Development workflow

Use this loop when iterating on protocol/control changes that affect both host and firmware:

1. Update protocol IDs/payloads in `hexapod-common/include/hexapod-common.hpp`.
2. Update firmware handlers in `hexapod-client/*commands*.cpp` / router code.
3. Update host-side transport/control consumers in `hexapod-server/src/*`.
4. Run server tests in simulator mode before hardware testing.

Suggested verification commands:

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests --output-on-failure
```

```bash
cd hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

Then run a scenario sweep in simulator mode:

```bash
cd hexapod-server
cp config.sim.txt config.txt
for s in scenarios/*.toml; do
  ./build/hexapod-server --scenario "$s" || break
done
```

## Documentation map

- `hexapod-server/README.md` — host runtime architecture, simulation flow, and test commands.
- `hexapod-client/README.md` — firmware build/flash workflow and protocol command handling notes.
- `docs/FIRMWARE.md` — wire protocol framing, constants, and payload definitions.
- `docs/HARDWARE.md` — mechanical/electrical build reference and dimensions.

## Safety notes

- Validate relay defaults, servo enable sequencing, and emergency stop behavior before full-body motion testing.
- Start with the robot unloaded and low-amplitude commands after calibration changes.
- Prefer simulator mode (`hexapod-server/config.sim.txt`) for early control-policy validation.
- Keep one hand on power disconnect / E-stop whenever first exercising new gait or calibration logic on hardware.
