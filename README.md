# Hexapod

A multi-part C++ project for controlling a hexapod robot using a Raspberry Pi Pico (Servo 2040) as the embedded controller and a Linux host as the serial command server.

## Repository layout

- `hexapod-common/` – shared protocol constants and byte-packing macros used by both host and firmware.
- `hexapod-server/` – Linux-side serial application that loads calibration data from `config.txt` and sends it to the robot.
- `hexapod-client/` – Pico firmware (Pimoroni Servo 2040 stack) that receives commands and drives servos, relay, and sensors.

## How it works

1. The host app (`hexapod-server`) parses calibration values from TOML config (`hexapod-server/config.txt`).
2. It opens `/dev/ttyACM0` at 115200 baud and sends command bytes + payload.
3. The Pico firmware (`hexapod-client`) reads commands over USB serial and handles:
   - setting angle calibrations
   - setting target servo angle
   - relay power control
   - reporting calibrations, current, voltage, and sensor values
4. Both sides share command IDs and byte macros from `hexapod-common/include/hexapod-common.hpp`.

## Command protocol (shared)

Defined in `hexapod-common/include/hexapod-common.hpp`:

- `0x01` `SET_ANGLE_CALIBRATIONS`
- `0x02` `SET_TARGET_ANGLE`
- `0x03` `SET_POWER_RELAY`
- `0x04` `GET_ANGLE_CALIBRATIONS`
- `0x05` `GET_CURRENT`
- `0x06` `GET_VOLTAGE`
- `0x07` `GET_SENSOR`

## Build

### 1) Build shared library (`hexapod-common`)

```bash
cd hexapod-common
make
```

Outputs static and shared libraries in `hexapod-common/lib/`.

### 2) Build Linux server (`hexapod-server`)

```bash
cd hexapod-server
make
```

Binary output: `hexapod-server/build/hexapod-server`.

### 3) Build Pico firmware (`hexapod-client`)

The client uses Pico SDK + Pimoroni libraries via CMake.

```bash
cd hexapod-client
mkdir -p build
cd build
cmake ..
cmake --build .
```

UF2 output will be generated in the build directory. See `hexapod-client/README.md` for SDK/environment setup details.

## Runtime notes

- Server expects the robot serial device at `/dev/ttyACM0`.
- Default server config file is `hexapod-server/config.txt`.
- `hexapod-client/CMakeLists.txt` currently contains an absolute include path for `hexapod-common`; update it for your machine if needed.

## Dependencies

### Linux server/common

- C++ compiler (GNU Make-based build)
- `CppLinuxSerial`
- `toml11`

### Pico client

- Raspberry Pi Pico SDK
- Pimoroni `pimoroni-pico` libraries
- CMake and ARM GCC toolchain

## Development quick start

```bash
# build common + server
cd hexapod-common && make
cd ../hexapod-server && make

# run server (from hexapod-server so config.txt resolves)
./build/hexapod-server
```

Then flash `hexapod-client` firmware to the Servo 2040 and connect via USB.
