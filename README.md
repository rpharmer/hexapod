# Hexapod

This repository contains host and firmware code for a serial-controlled hexapod robot.

## Components

- **`hexapod-server/`**: Linux host application that loads configuration/calibrations, performs serial handshake with firmware, runs multi-loop control + safety logic, and exchanges framed packets with the microcontroller.
- **`hexapod-client/`**: Pimoroni Servo 2040 (RP2040) firmware that initializes hardware, enforces host handshake state, drives 18 servos, and serves sensing/power/motion commands over USB serial.
- **`hexapod-common/`**: Shared protocol constants and framing utilities used by server and firmware.

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

## Current communication flow

1. Server parses `hexapod-server/config.txt`.
2. Server opens the configured serial device (default workflow often uses `/dev/ttyACM0` at `115200`).
3. Server sends `HELLO` with protocol version + capability byte.
4. Firmware responds with `ACK` or `NACK`.
5. Server uploads calibration pairs for each servo.
6. Server uses heartbeat + command traffic (`SET_JOINT_TARGETS`, sensing requests, etc.) while firmware is active.

Protocol constants and framing are shared in:

- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/include/framing.hpp`
- `docs/FIRMWARE.md`

## Project planning

- `docs/CODEBASE_REVIEW.md`
- `docs/REFACTORING_REVIEW.md`
- `docs/NEXT_STEPS.md`

## Build instructions

### Server (Linux)

Prerequisites:

- C++20-capable compiler
- CMake 3.16+
- `CppLinuxSerial`
- `toml11`

Build:

```bash
cd hexapod-server
cmake -S . -B build
cmake --build build -j
```

Run:

```bash
cd hexapod-server
./build/hexapod-server
```

### Client firmware (Servo 2040 / Pico)

Prerequisites:

- Pico SDK
- Pimoroni Pico libraries
- CMake 3.12+
- ARM GCC toolchain (`gcc-arm-none-eabi`)

Build setup target:

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
```

Build firmware target:

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Flash:

- Copy generated `.uf2` to Pico mass-storage device, or
- Use your environment's OpenOCD/programming flow if configured.

## Safety

- Validate calibration bounds before powering servos to avoid mechanical overtravel.
- Verify relay default state and power sequencing before full-system bring-up.
