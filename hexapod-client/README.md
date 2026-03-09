# hexapod-client

Firmware for the **Pimoroni Servo 2040** that drives the hexapod’s 18 servos, reads onboard analog channels/sensors, controls a power relay pin, and exchanges framed packets with `hexapod-server` over USB serial.

## What this firmware does

- Initializes Servo 2040 peripherals and an 18-channel `ServoCluster`.
- Applies per-servo pulse-width calibration for all joints.
- Waits for a USB-serial host connection (with LED status animation while waiting).
- Processes framed protocol commands defined in `hexapod-common`.
- Supports both low-level and higher-level hardware queries (including a full hardware snapshot).

Primary source files:

- `hexapod-client.cpp`: main loop, command handlers, hardware I/O.
- `serialCommsClient.cpp/.hpp`: USB serial transport implementation for the framed protocol.
- `../hexapod-common/framing.cpp` and `../hexapod-common/include/hexapod-common.hpp`: shared framing + protocol constants.

## Repository assumptions

This project expects the following layout (sibling directories):

```text
<workspace>/
├── pico-sdk/
├── pimoroni-pico/
└── hexapod/
    └── hexapod-client/
```

`pico_sdk_import.cmake` and `pimoroni_pico_import.cmake` are configured with this layout in mind.

## Prerequisites

Install build tools:

```bash
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi build-essential
```

Clone SDK dependencies (if not already installed):

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

This mode is useful to populate/prebuild SDK and Pimoroni dependency objects without compiling firmware sources.

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
```

### 2) Full firmware build

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Build outputs are generated under `build/`, including:

- `hexapod-client.elf`
- `hexapod-client.uf2`
- `hexapod-client.hex`

## Flashing

### UF2 drag-and-drop

1. Hold BOOTSEL while connecting the Pico/Servo 2040.
2. A mass-storage device appears.
3. Copy `build/hexapod-client.uf2` to that device.

### OpenOCD custom target

A convenience target is generated:

```bash
cmake --build build --target program-pico
```

This executes the generated `build/program-pico.sh` script, which invokes OpenOCD with RP2040 config files. Ensure your local OpenOCD setup and interface config (`interface/linuxgpiod-new.cfg`) are available.

## Runtime behavior

At startup the firmware:

1. Initializes stdio over USB.
2. Configures analog mux pull settings for six sensor inputs.
3. Initializes and calibrates all 18 servos.
4. Displays a rotating LED animation until USB serial is connected.
5. Enables all servos and enters packet-processing loop.

## Protocol overview

All command/status/error IDs and protocol version are shared in:

- `../hexapod-common/include/hexapod-common.hpp`

Notable commands handled by this firmware:

- `HELLO`
- `SET_ANGLE_CALIBRATIONS`
- `SET_TARGET_ANGLE`
- `SET_JOINT_TARGETS`
- `SET_POWER_RELAY`
- `GET_ANGLE_CALIBRATIONS`
- `GET_CURRENT`
- `GET_VOLTAGE`
- `GET_SENSOR`
- `GET_FULL_HARDWARE_STATE`
- `HEARTBEAT`

### Payload notes

- **Calibration upload** expects `18 * 2 * sizeof(float)` bytes.
- **Joint target upload** expects `18 * sizeof(float)` bytes, each target in radians.
- **Full hardware state response** returns:
  - 18 floats (last commanded joint targets, radians)
  - 6 foot-contact bytes (derived from sensor voltage threshold)
  - battery voltage (float)
  - current draw (float)

## Hardware mapping notes

- Servo channels: `SERVO_1`..`SERVO_18` (Servo 2040 pin mapping).
- Relay control pin mask uses `A0` (`GPIO26`) in `hexapod-client.hpp`.
- Shared ADC + analog mux are used for voltage/current/sensor reads.

## Troubleshooting

- **CMake cannot find Pico SDK**
  - Confirm `PICO_SDK_PATH` is set correctly, or update `pico_sdk_import.cmake` path assumptions.
- **Pimoroni libs not found**
  - Ensure `pimoroni-pico` exists at expected sibling path.
- **No serial handshake**
  - Verify USB cable/data path, correct host serial device, and protocol version match.
- **Unexpected servo behavior**
  - Re-check calibration payload ordering and units (radians for joint targets, pulse mapping done on-device).

## Related docs

- Root project overview: `../README.md`
- Shared protocol definitions: `../hexapod-common/include/hexapod-common.hpp`
