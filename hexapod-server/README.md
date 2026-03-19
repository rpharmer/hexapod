# hexapod-server

`hexapod-server` is the Linux host-side control process for the hexapod robot. It connects to firmware over USB serial, runs control/safety loops, and streams joint targets.

## What it does

- Loads serial + calibration settings from `config.txt`.
- Creates a serial-backed hardware bridge (`SimpleHardwareBridge`).
- Starts `RobotControl` multi-thread loops:
  - bus loop (500 Hz)
  - estimator loop (500 Hz)
  - safety loop (500 Hz)
  - control loop (250 Hz)
  - diagnostics loop (2 Hz)
- Sends `MotionIntent` updates while running.

## Directory layout

```text
hexapod-server/
├── CMakeLists.txt
├── config.txt
├── README.md
├── include/
│   ├── body_controller.hpp
│   ├── control_config.hpp
│   ├── control_pipeline.hpp
│   ├── double_buffer.hpp
│   ├── estimator.hpp
│   ├── gait_scheduler.hpp
│   ├── geometry_config.hpp
│   ├── hardware_bridge.hpp
│   ├── hexapod-server.hpp
│   ├── leg_fk.hpp
│   ├── leg_ik.hpp
│   ├── logger.hpp
│   ├── loop_timing.hpp
│   ├── robot_control.hpp
│   ├── safety_supervisor.hpp
│   ├── serialCommsServer.hpp
│   ├── status_reporter.hpp
│   └── types.hpp
└── src/
    ├── body_controller.cpp
    ├── control_config.cpp
    ├── control_pipeline.cpp
    ├── estimator.cpp
    ├── gait_scheduler.cpp
    ├── geometry_config.cpp
    ├── hardware_bridge.cpp
    ├── hexapod-server.cpp
    ├── leg_fk.cpp
    ├── leg_ik.cpp
    ├── logger.cpp
    ├── loop_timing.cpp
    ├── robot_control.cpp
    ├── safety_supervisor.cpp
    ├── serialCommsServer.cpp
    ├── status_reporter.cpp
    └── types.cpp
```

## Build

Requirements:

- Linux
- CMake 3.16+
- C++20 compiler
- `CppLinuxSerial`
- `toml11`

Configure + build:

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

Stop with `Ctrl+C`.

## Configuration (`config.txt`)

Expected TOML fields:

- `title = "Hexapod Config File"`
- `Schema = "hexapod.server.config"`
- `SchemaVersion = 1`
- `SerialDevice` (e.g., `/dev/ttyACM0`)
- `BaudRate` (e.g., `115200`)
- `Timeout_ms`
- `MotorCalibrations` with exactly 18 entries:
  - `[
    "<JointID>", <min_pulse>, <max_pulse>
  ]`
  - no duplicate/missing joints
  - `500 <= min_pulse < max_pulse <= 2500`

## Control pipeline overview

`ControlPipeline` executes per control step:

1. `GaitScheduler::update(...)`
2. `BodyController::update(...)`
3. `LegIK::solve(...)`
4. status synthesis (`ControlStatus`)

`SafetySupervisor` evaluates faults in parallel safety loop. Shared data handoff between loops uses `DoubleBuffer<T>`.

## Protocol notes

- Wire constants are shared in `../hexapod-common/include/hexapod-common.hpp`.
- `SimpleHardwareBridge::write()` sends `SET_JOINT_TARGETS` and expects `ACK`.
- `SimpleHardwareBridge::read()` requests `GET_FULL_HARDWARE_STATE` and decodes joint targets, foot contacts, voltage, and current.

## Troubleshooting

- **Cannot open serial device**
  - Verify `SerialDevice` and Linux permissions (`dialout`/udev rules).
- **No ACK / timeouts**
  - Confirm firmware is running and protocol versions match.
- **Unexpected motion/joint behavior**
  - Re-check calibration values and ordering.

## Safety

- Keep the robot mechanically unloaded for initial bring-up after calibration changes.
- Verify E-stop and relay behavior before enabling walking tests.
