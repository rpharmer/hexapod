# hexapod-client

Firmware for the **Pimoroni Servo 2040**. It drives 18 servos, reads analog/sensor channels, controls a relay pin, and exchanges framed packets with `hexapod-server` over USB serial.

## Responsibilities

- Initialize Servo 2040 peripherals and an 18-channel servo cluster.
- Apply per-servo pulse calibration values from host configuration.
- Wait for host serial availability and enforce handshake/lifecycle state.
- Dispatch motion, sensing, and power commands from `hexapod-common` protocol IDs.
- Return full hardware snapshots for host estimation and diagnostics.

## Source map

- `firmware_boot.cpp` — hardware bring-up/teardown and handshake helpers.
- `command_dispatch.cpp` — runtime command loop and domain dispatch.
- `motion_commands.cpp` — calibration and joint target handlers.
- `sensing_commands.cpp` — sensors, current, voltage, full-state handlers.
- `power_commands.cpp` — relay and servo-enable handlers.
- `firmware_context.cpp/.hpp` — shared firmware runtime state.
- `serialCommsClient.cpp/.hpp` — USB serial transport.
- `../hexapod-common/*` — framed protocol implementation and command IDs.

## Runtime lifecycle

Firmware state transitions:

- `BOOT`
- `WAITING_FOR_HOST`
- `ACTIVE`
- `STOPPING`
- `OFF`

## Repository assumptions

Default sibling layout:

```text
<workspace>/
├── pico-sdk/
├── pimoroni-pico/
└── hexapod/
    └── hexapod-client/
```

`pico_sdk_import.cmake` and `pimoroni_pico_import.cmake` rely on this layout unless overridden.

## Prerequisites

```bash
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi build-essential
```

If missing, clone dependencies:

```bash
git clone https://github.com/raspberrypi/pico-sdk
git -C pico-sdk submodule update --init

git clone https://github.com/pimoroni/pimoroni-pico
```

Recommended environment variable:

```bash
export PICO_SDK_PATH="/absolute/path/to/pico-sdk"
```

## Build

From `hexapod-client/`:

### 1) Optional dependency setup target

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
```

### 2) Full firmware target

```bash
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Typical artifacts under `build/`:

- `hexapod-client.elf`
- `hexapod-client.uf2`
- `hexapod-client.hex`

## Host tests (native)

Run host-mode command/router tests without Pico SDK or toolchain:

```bash
cd hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

Equivalent non-preset commands:

```bash
cd hexapod-client
cmake -S . -B build-host-tests -DHEXAPOD_CLIENT_ENABLE_HOST_TESTS=ON
cmake --build build-host-tests -j
ctest --test-dir build-host-tests --output-on-failure
```

Clean rebuild when switching SDK locations or CMake cache settings:

```bash
cd hexapod-client
rm -rf build
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

## Flashing

### UF2 drag-and-drop

1. Hold BOOTSEL while connecting the board.
2. Wait for the USB mass-storage device.
3. Copy `build/hexapod-client.uf2` to the device.

### OpenOCD helper target

```bash
cmake --build build --target program-pico
```

This uses the generated script `build/program-pico.sh`.

## Protocol highlights

Defined in `../hexapod-common/include/hexapod-common.hpp` and described in `../docs/FIRMWARE.md`.

Common commands:

- `HELLO`, `HEARTBEAT`, `KILL`
- `SET_ANGLE_CALIBRATIONS`, `SET_TARGET_ANGLE`, `SET_JOINT_TARGETS`
- `SET_POWER_RELAY`, `SET_SERVOS_ENABLED`, `GET_SERVOS_ENABLED`, `SET_SERVOS_TO_MID`
- `GET_ANGLE_CALIBRATIONS`, `GET_CURRENT`, `GET_VOLTAGE`, `GET_SENSOR`, `GET_FULL_HARDWARE_STATE`

Payload shape expectations:

- Calibration upload: `18 * 2 * sizeof(float)` bytes.
- Joint target upload: `18 * sizeof(float)` bytes (radians).
- Full-state response:
  - 18 floats (joint positions, radians)
  - 6 foot-contact bytes
  - battery voltage (`float`)
  - current draw (`float`)

## Troubleshooting

- **Pico SDK not found**: verify `PICO_SDK_PATH` and CMake import paths.
- **Pimoroni libraries missing**: confirm `pimoroni-pico` sibling checkout.
- **No serial handshake**: verify USB data cable, host serial device path, and protocol version.
- **Unexpected servo motion**: validate calibration ordering/ranges and radian-based command inputs.
- **Board repeatedly re-enumerates USB**: check power budget (servo rail brownout) and isolate from high-load servo startup during bring-up.

## Related docs

- Root overview: `../README.md`
- Shared protocol constants: `../hexapod-common/include/hexapod-common.hpp`
- Wire protocol details: `../docs/FIRMWARE.md`
