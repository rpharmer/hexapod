# hexapod-server

`hexapod-server` is the Linux host-side control process for this project’s hexapod robot.
It connects to the microcontroller over USB serial, runs the control pipeline, and streams per-joint targets to the firmware.

## What it does

- Loads runtime serial + calibration settings from `config.txt`.
- Creates a hardware bridge (`SimpleHardwareBridge`) over `CppLinuxSerial`.
- Spawns a multi-thread control stack in `RobotControl`:
  - **bus loop** at 500 Hz (hardware read/write)
  - **estimator loop** at 500 Hz
  - **safety loop** at 500 Hz
  - **control loop** at 250 Hz
  - **diagnostics loop** at 2 Hz (stdout status)
- Sends continuous `MotionIntent` commands (stand, then walk) until SIGINT/SIGTERM.

## Directory layout

```text
hexapod-server/
├── CMakeLists.txt
├── config.txt
├── README.md
├── include/
│   ├── body_controller.hpp
│   ├── double_buffer.hpp
│   ├── estimator.hpp
│   ├── gait_scheduler.hpp
│   ├── hardware_bridge.hpp
│   ├── hexapod-server.hpp
│   ├── leg_fk.hpp
│   ├── leg_ik.hpp
│   ├── robot_control.hpp
│   ├── safety_supervisor.hpp
│   ├── serialCommsServer.hpp
│   └── types.hpp
└── src/
    ├── body_controller.cpp
    ├── estimator.cpp
    ├── gait_scheduler.cpp
    ├── hardware_bridge.cpp
    ├── hexapod-server.cpp
    ├── leg_fk.cpp
    ├── leg_ik.cpp
    ├── robot_control.cpp
    ├── safety_supervisor.cpp
    └── serialCommsServer.cpp
```

## Build

### Requirements

- Linux
- CMake 3.16+
- C++20 compiler
- `CppLinuxSerial`
- `toml11`

### Configure and compile

```bash
cd hexapod-server
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
cd hexapod-server
./build/hexapod-server
```

Use `Ctrl+C` to stop cleanly.

## Configuration (`config.txt`)

The server expects a TOML file with:

- `title = "Hexapod Config File"`
- `SerialDevice` (e.g., `/dev/ttyACM0`)
- `BaudRate` (e.g., `115200`)
- `Timeout_ms` (serial timeout)
- `MotorCalibrations` with exactly **18** entries:
  - tuple format: `["<JointID>", <min_pulse>, <max_pulse>]`
  - joint IDs follow the shared protocol ordering (e.g., `R31`, `L13`)

Example:

```toml
title = "Hexapod Config File"
SerialDevice = '/dev/ttyACM0'
BaudRate = 115200
Timeout_ms = 100

MotorCalibrations = [
  ["R31", 1031, 2088],
  ["R32", 1003, 2016],
  # ... total 18 entries ...
  ["L13", 1035, 2027]
]
```

## Runtime behavior

At startup (`src/hexapod-server.cpp`), the process:

1. Parses `config.txt`.
2. Initializes hardware + estimator components.
3. Starts `RobotControl` worker threads.
4. Sends a `STAND` intent briefly.
5. Repeatedly sends `WALK` intent updates every 100 ms.

### Control pipeline

`RobotControl` composes the major subsystems:

- **Estimator** (`SimpleEstimator`) to convert raw hardware state to estimated state.
- **Gait scheduler** (`GaitScheduler`) to produce gait phase data.
- **Body controller** (`BodyController`) to generate foot targets.
- **IK solver** (`LegIK`) to convert foot targets into joint targets.
- **Safety supervisor** (`SafetySupervisor`) to evaluate faults and inhibit motion.

Data handoff between loops uses lock-free-style double buffers (`DoubleBuffer<T>`).

## Protocol notes

- Wire command and protocol constants are defined in `../hexapod-common/include/hexapod-common.hpp`.
- `SimpleHardwareBridge::write()` transmits `SET_JOINT_TARGETS` packets and expects `ACK`.
- `SimpleHardwareBridge::read()` requests `GET_FULL_HARDWARE_STATE` and decodes positions, foot contacts, voltage, and current.

## Troubleshooting

- **Cannot open serial device**
  - Verify `SerialDevice` in `config.txt`.
  - Check permissions (`dialout`/udev rules).
- **No ACK / timeouts**
  - Confirm firmware is flashed and running on the client.
  - Confirm server/client protocol versions match.
- **Unexpected motion or joint limits**
  - Re-validate calibration values in `MotorCalibrations` before enabling power.

## Safety

- Keep the robot mechanically unloaded for first bring-up after calibration changes.
- Verify E-stop and relay behavior before enabling full walking tests.
