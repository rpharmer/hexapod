# hexapod-client

Firmware for the **Pimoroni Servo 2040**. It drives 18 servos, reads onboard analog/sensor channels, controls a relay pin, and exchanges framed packets with `hexapod-server` via USB serial.

## Responsibilities

- Initialize Servo 2040 peripherals and an 18-channel `ServoCluster`.
- Apply per-servo pulse calibration values from host configuration.
- Wait for host serial availability and enforce handshake/lifecycle state.
- Dispatch motion, sensing, and power commands from `hexapod-common` protocol IDs.
- Return full robot-side hardware snapshots for host estimation/diagnostics.

## Source map

- `firmware_boot.cpp` — hardware bring-up/teardown and handshake helpers.
- `command_dispatch.cpp` — runtime command loop + domain dispatch.
- `motion_commands.cpp` — calibration + joint target handlers.
- `sensing_commands.cpp` — sensors, current, voltage, full-state handlers.
- `power_commands.cpp` — relay and servo-enable handlers.
- `firmware_context.cpp/.hpp` — shared firmware runtime state.
- `serialCommsClient.cpp/.hpp` — USB serial transport.
- `../hexapod-common/*` — framed protocol implementation and command IDs.

## Runtime lifecycle

The firmware transitions through explicit states:

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

## Flashing

### UF2 drag-and-drop

1. Hold BOOTSEL while connecting the board.
2. Wait for the mass-storage device.
3. Copy `build/hexapod-client.uf2` to the device.

### OpenOCD helper target

```bash
cmake --build build --target program-pico
```

This uses generated script `build/program-pico.sh`.

## Protocol highlights

Defined in `../hexapod-common/include/hexapod-common.hpp`.

Common commands:

- `HELLO`, `HEARTBEAT`, `KILL`
- `SET_ANGLE_CALIBRATIONS`, `SET_TARGET_ANGLE`, `SET_JOINT_TARGETS`
- `SET_POWER_RELAY`, `SET_SERVOS_ENABLED`, `GET_SERVOS_ENABLED`, `SET_SERVOS_TO_MID`
- `GET_ANGLE_CALIBRATIONS`, `GET_CURRENT`, `GET_VOLTAGE`, `GET_SENSOR`, `GET_FULL_HARDWARE_STATE`

Payload shape expectations:

- Calibration upload: `18 * 2 * sizeof(float)` bytes.
- Joint target upload: `18 * sizeof(float)` bytes (radians).
- Full-state response:
  - 18 floats (commanded targets, radians)
  - 6 foot-contact bytes
  - battery voltage (`float`)
  - current draw (`float`)

## Troubleshooting

- **Pico SDK not found**: check `PICO_SDK_PATH` and import cmake paths.
- **Pimoroni libs missing**: confirm `pimoroni-pico` sibling checkout.
- **No serial handshake**: verify data-capable USB cable, host serial device, and protocol version match.
- **Unexpected servo motion**: validate calibration ordering/range and radian-based command inputs.

## Related docs

- Root overview: `../README.md`
- Shared protocol constants: `../hexapod-common/include/hexapod-common.hpp`
