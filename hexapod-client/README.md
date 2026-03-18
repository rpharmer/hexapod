# hexapod-client

Firmware for the **Pimoroni Servo 2040** that drives the hexapod’s 18 servos, reads onboard analog/sensor channels, controls a power relay pin, and exchanges framed packets with `hexapod-server` over USB serial.

## What this firmware does

- Initializes Servo 2040 peripherals and an 18-channel `ServoCluster`.
- Applies per-servo pulse-width calibration data.
- Waits for USB serial host availability with LED status animation.
- Enforces host handshake state before accepting active control commands.
- Processes framed protocol commands from `hexapod-common`.
- Supports both low-level and full-snapshot hardware queries.

## Source map

- `firmware_boot.cpp`: hardware bring-up/teardown and handshake helpers.
- `command_dispatch.cpp`: runtime command loop + domain dispatch.
- `motion_commands.cpp`: calibration and joint-target write handlers.
- `sensing_commands.cpp`: sensor/current/voltage/full-state handlers.
- `power_commands.cpp`: relay and servo-enable handlers.
- `firmware_context.cpp/.hpp`: shared firmware state and hardware objects.
- `serialCommsClient.cpp/.hpp`: USB serial transport used by framed protocol.
- `../hexapod-common/framing.cpp` + `../hexapod-common/include/hexapod-common.hpp`: shared framing/protocol constants.

## Firmware runtime states

The firmware uses explicit lifecycle states:

- `BOOT`
- `WAITING_FOR_HOST`
- `ACTIVE`
- `STOPPING`
- `OFF`

## Repository assumptions

Expected sibling layout:

```text
<workspace>/
├── pico-sdk/
├── pimoroni-pico/
└── hexapod/
    └── hexapod-client/
```

`pico_sdk_import.cmake` and `pimoroni_pico_import.cmake` assume this by default.

## Prerequisites

Install build tools:

```bash
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi build-essential
```

Clone SDK dependencies if needed:

```bash
git clone https://github.com/raspberrypi/pico-sdk
git -C pico-sdk submodule update --init

git clone https://github.com/pimoroni/pimoroni-pico
```

Set SDK path (recommended):

```bash
export PICO_SDK_PATH="/absolute/path/to/pico-sdk"
```

## Build

From `hexapod-client/`:

### 1) Optional dependency prebuild

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
```

### 2) Full firmware build

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Build outputs under `build/` include:

- `hexapod-client.elf`
- `hexapod-client.uf2`
- `hexapod-client.hex`

## Flashing

### UF2 drag-and-drop

1. Hold BOOTSEL while connecting the Pico/Servo 2040.
2. A mass-storage device appears.
3. Copy `build/hexapod-client.uf2` to that device.

### OpenOCD helper target

```bash
cmake --build build --target program-pico
```

This invokes generated script `build/program-pico.sh`.

## Protocol overview

Shared command IDs and protocol constants live in:

- `../hexapod-common/include/hexapod-common.hpp`

Notable handled commands include:

- `HELLO`
- `HEARTBEAT`
- `KILL`
- `SET_ANGLE_CALIBRATIONS`
- `SET_TARGET_ANGLE`
- `SET_JOINT_TARGETS`
- `SET_POWER_RELAY`
- `SET_SERVOS_ENABLED`
- `GET_SERVOS_ENABLED`
- `SET_SERVOS_TO_MID`
- `GET_ANGLE_CALIBRATIONS`
- `GET_CURRENT`
- `GET_VOLTAGE`
- `GET_SENSOR`
- `GET_FULL_HARDWARE_STATE`

### Payload notes

- Calibration upload expects `18 * 2 * sizeof(float)` bytes.
- Joint target upload expects `18 * sizeof(float)` bytes (radians).
- Full hardware state response includes:
  - 18 floats (last commanded joint targets, radians)
  - 6 foot-contact bytes
  - battery voltage (float)
  - current draw (float)

## Troubleshooting

- **CMake cannot find Pico SDK**
  - Confirm `PICO_SDK_PATH` or update `pico_sdk_import.cmake` assumptions.
- **Pimoroni libs not found**
  - Ensure `pimoroni-pico` exists in expected sibling path.
- **No serial handshake**
  - Verify cable/data path, host serial device, and protocol version alignment.
- **Unexpected servo behavior**
  - Re-check calibration payload ordering and joint-target units (radians).

## Related docs

- Root overview: `../README.md`
- Shared protocol definitions: `../hexapod-common/include/hexapod-common.hpp`
