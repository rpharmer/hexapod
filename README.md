# Hexapod

This repository contains host and firmware code for a serial-controlled hexapod robot.

## Components

- **`hexapod-server/`**: Linux host application that loads servo calibration data from `config.txt`, performs a serial handshake, and streams calibration values to the microcontroller.
- **`hexapod-client/`**: Raspberry Pi Pico + Pimoroni Servo 2040 firmware that controls servos/IO and communicates over USB serial.
- **`hexapod-common/`**: Shared protocol constants and abstract serial interface declarations.

## Repository layout

```text
hexapod/
├── README.md
├── hexapod-common/
│   ├── include/hexapod-common.hpp
│   └── protocol.md
├── hexapod-server/
│   ├── config.txt
│   ├── include/
│   │   ├── hexapod-server.hpp
│   │   └── serialCommsServer.hpp
│   ├── src/
│   │   ├── hexapod-server.cpp
│   │   └── serialCommsServer.cpp
│   └── makefile
└── hexapod-client/
    ├── CMakeLists.txt
    ├── hexapod-client.cpp
    ├── hexapod-client.hpp
    ├── serialCommsClient.cpp
    └── serialCommsClient.hpp
```

## Current communication flow

1. Server parses `hexapod-server/config.txt` using `toml11`.
2. Calibration tuples are sorted into expected servo order.
3. Server opens `/dev/ttyACM0` at `115200` baud.
4. Server sends a framed `HELLO` packet with a sequence number, protocol version, and requested capabilities.
5. Firmware responds with framed `ACK`/`NACK` using the same sequence number.
6. On successful handshake, server sends framed calibration min/max pairs (`uint16_t`) for each servo and validates the response sequence number.

Protocol constants and message definitions live in:

- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/protocol.md`

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
make
```

Run:

```bash
cd hexapod-server
./build/hexapod-server
```

> Note: serial port path and timeout are currently hard-coded in `hexapod-server/src/hexapod-server.cpp`.

### Client firmware (Pico + Servo 2040)

Prerequisites:

- Pico SDK
- Pimoroni Pico libraries
- CMake 3.12+
- ARM GCC toolchain (`gcc-arm-none-eabi`)

Build:

```bash
cd hexapod-client
mkdir -p build
cd build
cmake ..
cmake --build .
```

Flash:

- Copy generated `.uf2` to Pico mass-storage device, or
- Use project-specific flash target/tooling when configured in your local environment.

## Notes and limitations

- The server currently focuses on handshake + calibration transfer in `main`; higher-level runtime control commands are not yet implemented there.
- `hexapod-common/protocol.md` describes additional commands beyond the calibration startup path currently exercised by `hexapod-server`.
- `hexapod-client/README.md` is still the upstream Pico boilerplate README and may not reflect project-specific behavior.

## Safety

- Validate calibration bounds before powering servos to avoid mechanical overtravel.
- Verify relay default state and power sequencing before full system bring-up.
