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


## Offline simulation/testing

Use offline mode when you want deterministic control/safety validation without a physical robot.

### 1) Required config flags for simulator mode

Set the runtime mode to `sim` in `config.txt` (or copy `config.sim.txt` over `config.txt` before running):

- `Runtime.Mode = "sim"` (required)
- Optional simulator tuning under `Runtime.Sim.*`:
  - `InitialVoltageV`
  - `InitialCurrentA`
  - `ResponseRateHz`
  - `DropBus`, `LowVoltage`, `HighCurrent` (fault injection toggles; keep `false` for baseline runs)

> Note: scenarios call `setSimFaultToggles(...)` and will fail if runtime mode is not `sim`.

### 2) Execute scenario runs

Baseline scenarios are in `hexapod-server/scenarios/`:

- `01_nominal_stand_walk.toml`
- `02_command_timeout_fallback.toml`
- `03_power_fault_triggers.toml`
- `04_contact_loss_edge_cases.toml`

Run one scenario:

```bash
cd hexapod-server
cp config.sim.txt config.txt
cmake -S . -B build
cmake --build build -j
./build/hexapod-server --scenario scenarios/01_nominal_stand_walk.toml
```

Run all baseline scenarios in a loop:

```bash
cd hexapod-server
cp config.sim.txt config.txt
for s in scenarios/*.toml; do
  echo "=== Running $s ==="
  ./build/hexapod-server --scenario "$s" || break
done
```

### 3) Run tests (including newly added tests)

Tests are built only when `HEXAPOD_SERVER_BUILD_TESTS=ON` is enabled:

```bash
cd hexapod-server
cmake -S . -B build-tests -DHEXAPOD_SERVER_BUILD_TESTS=ON
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

To run one new test binary directly:

```bash
./build-tests/test_robot_runtime_loop
```

### 4) Log markers for pass/fail interpretation

When reading console/app logs for scenario runs:

- **Healthy run indicators**
  - `Runtime.Mode=sim`
  - `Running scenario: <name>`
  - repeated `Scenario event @<N>ms mode update` entries (for scenarios with mode events)
  - process exits with status `0`
- **Failure indicators**
  - `Failed to load scenario file '<path>'`
  - `Scenario driver requires sim runtime (SimHardwareBridge)`
  - any `LOG_ERROR(...)` line during scenario execution
  - non-zero process exit status

For unit tests, failures print `FAIL: <message>` and return non-zero.

### Troubleshooting (offline mode)

| Symptom | Likely cause | Fix |
|---|---|---|
| `Scenario driver requires sim runtime (SimHardwareBridge)` | `Runtime.Mode` is still `serial` | Set `Runtime.Mode = "sim"` in active config, then rerun. |
| `Failed to load scenario file '...'` | Wrong/missing scenario path or invalid TOML | Confirm file path from `hexapod-server/` and validate scenario TOML syntax/keys. |
| Scenario assertions are flaky due to timing | Test assumes exact wall-clock timing under CPU load | Prefer pass/fail checks based on mode/fault transitions and log markers, not exact timestamps. |

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
