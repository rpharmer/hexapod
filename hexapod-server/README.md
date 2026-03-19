# hexapod-server

Linux host-side control process for the hexapod robot. It connects to firmware over USB serial, runs multi-loop control/safety logic, and streams joint targets.

## Responsibilities

- Load serial and calibration settings from `config.txt`.
- Construct serial-backed hardware bridge (`SimpleHardwareBridge`).
- Execute `RobotControl` loops:
  - bus loop (500 Hz)
  - estimator loop (500 Hz)
  - safety loop (500 Hz)
  - control loop (250 Hz)
  - diagnostics loop (2 Hz)
- Emit `MotionIntent` updates while monitoring health/fault state.

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
- `SerialDevice` (example: `/dev/ttyACM0`)
- `BaudRate` (example: `115200`)
- `Timeout_ms`
- `MotorCalibrations` with exactly 18 entries:
  - format: `["<JointID>", <min_pulse>, <max_pulse>]`
  - no duplicates or missing joint IDs
  - `500 <= min_pulse < max_pulse <= 2500`

## Control pipeline

Per control step, `ControlPipeline` performs:

1. `GaitScheduler::update(...)`
2. `BodyController::update(...)`
3. `LegIK::solve(...)`
4. control status synthesis (`ControlStatus`)

`SafetySupervisor` runs independently in the safety loop. Cross-loop data exchange uses `DoubleBuffer<T>`.

## Protocol bridge notes

- Shared wire constants: `../hexapod-common/include/hexapod-common.hpp`.
- `SimpleHardwareBridge::write()` sends `SET_JOINT_TARGETS` and expects `ACK`.
- `SimpleHardwareBridge::read()` requests `GET_FULL_HARDWARE_STATE` and decodes joint targets, contacts, voltage, and current.

## Troubleshooting

- **Cannot open serial device**: verify `SerialDevice` path and Linux permissions (`dialout`/udev).
- **ACK timeout / handshake failures**: confirm firmware is running and protocol versions match.
- **Unexpected joint behavior**: re-check calibration ordering and pulse bounds.

## Safety checklist

- Keep the robot mechanically unloaded during initial bring-up after calibration edits.
- Validate E-stop path and relay defaults before enabling walking gaits.
- Start with low-amplitude commands when testing new hardware changes.
