# Hexapod

This repository contains host and firmware code for a serial-controlled hexapod robot.

## Components

- **`hexapod-server/`**: Linux host application that controls hexapod robot. It loads servo calibration data from `config.txt`, performs a serial handshake, and streams calibration values to the microcontroller and then sends/receives packets to/from hexapod-client.
- **`hexapod-client/`**: Pimoroni Servo 2040 (based on Raspberry Pi Pico) firmware that controls servos/IO and communicates over USB serial.
- **`hexapod-common/`**: Shared protocol constants and abstract serial interface declarations.

## Repository layout

```text
hexapod/
├── README.md
├── docs/
│   ├── CODEBASE_REVIEW.md
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
│   │   ├── body_controller.hpp
│   │   ├── double_buffer.hpp
│   │   ├── estimator.hpp
│   │   ├── gait_scheduler.hpp
│   │   ├── hardware_bridge.hpp
│   │   ├── hexapod-server.hpp
│   │   ├── leg_fk.hpp
│   │   ├── leg_ik.hpp
│   │   ├── robot_control.hpp
│   │   ├── safety_supervisor.hpp
│   │   ├── serialCommsServer.hpp
│   │   ├── toml.hpp.tmp
│   │   └── types.hpp
│   ├── src/
│   │   ├── body_controller.cpp
│   │   ├── estimator.cpp
│   │   ├── gait_scheduler.cpp
│   │   ├── hardware_bridge.cpp
│   │   ├── hexapod-server.cpp
│   │   ├── leg_fk.cpp
│   │   ├── leg_ik.cpp
│   │   ├── robot_control.cpp
│   │   ├── safety_supervisor.cpp
│   │   └── serialCommsServer.cpp
│   └── CMakeLists.txt
└── hexapod-client/
    ├── CMakeLists.txt
    ├── hexapod-client.cpp
    ├── hexapod-client.hpp
    ├── serialCommsClient.cpp
    ├── serialCommsClient.hpp
    ├── pimoroni_pico_import.cmake
    ├── pico_sdk_import.cmake
    └── README.md
```

## Current communication flow

1. Server parses `hexapod-server/config.txt` using `toml11`.
2. Calibration tuples are sorted into expected servo order.
3. Server opens `/dev/ttyACM0` at `115200` baud.
4. Server sends handshake bytes: `HELLO`, protocol version, requested capabilities.
5. Firmware responds with `ACK`/`NACK`.
6. On successful handshake, server sends calibration min/max pairs (`uint16_t`) for each servo.
7. Server sends periodic `HEARTBEAT` commands and expects `ACK` responses to verify link health.

Protocol constants and message definitions live in:

- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/protocol.md`

## Project planning

- See `docs/NEXT_STEPS.md` for a prioritized roadmap of fixes, reliability work, and longer-term improvements.

## Build instructions

### Server (Linux)

Prerequisites:

- C++17-capable compiler
- `CppLinuxSerial`
- `toml11`
- `make`

Build:

```bash
cd hexapod-server
mkdir build
cd build
cmake ..
make
```

Run:

```bash
cd hexapod-server
./build/hexapod-server
```

### Client firmware (Servo 2040(Pico))

Prerequisites:

- Pico SDK
- Pimoroni Pico libraries
- CMake 3.12+
- ARM GCC toolchain (`gcc-arm-none-eabi`)

Build:

Run setup/prebuild in `hexapod-client/build` (this prebuilds Pico SDK/Pimoroni dependency objects for `hexapod-client` without compiling project firmware sources):

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
```

Build `hexapod-client.elf` after setup/prebuild is complete (reconfigure with `HEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF` first):

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Quick verification flow (setup prebuild, then full firmware compile):

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Quick verification flow (prebuild/setup + compile):

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
cmake --build build --target hexapod-client
```

Flash:

- Copy generated `.uf2` to Pico mass-storage device, or
- Use project-specific flash target/tooling when configured in your local environment.

## Notes and limitations

- `hexapod-client/README.md` is still the upstream Pico boilerplate README and may not reflect project-specific behavior.

## Safety

- Validate calibration bounds before powering servos to avoid mechanical overtravel.
- Verify relay default state and power sequencing before full system bring-up.
