# hexapod-server

Linux host-side control process for the hexapod robot. It connects to firmware over USB serial, runs multi-loop control/safety logic, and streams joint targets.

## Responsibilities

- Load serial and calibration settings from `config.txt`.
- Construct a serial-backed hardware bridge (`SimpleHardwareBridge`).
- Execute `RobotControl` loops:
  - bus loop (500 Hz)
  - estimator loop (500 Hz)
  - safety loop (500 Hz)
  - control loop (250 Hz)
  - diagnostics loop (2 Hz)
- Emit `MotionIntent` updates while monitoring health and fault state.

## Directory layout

```text
hexapod-server/
├── CMakeLists.txt
├── config.txt
├── config.sim.txt
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
├── src/
├── scenarios/
└── tests/
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

Build with tests enabled:

```bash
cd hexapod-server
cmake -S . -B build-tests -DHEXAPOD_SERVER_BUILD_TESTS=ON
cmake --build build-tests -j
```

## Run

```bash
cd hexapod-server
./build/hexapod-server
```

Stop with `Ctrl+C`.

Run with an explicit config file:

```bash
cd hexapod-server
./build/hexapod-server --config config.txt
```

Controller-driven loop (optional):

```bash
cd hexapod-server
./build/hexapod-server --xbox-device /dev/input/eventX
```

Button mappings in controller mode:

- `A` => WALK
- `B` => STAND
- `X` => RIPPLE gait
- `Y` => TRIPOD gait
- Left stick => walk heading + speed (`heading_rad`, `speed_mps`)
  - stick axes (`LX`,`LY`) are radial-deadzone filtered and normalized to `[-1, 1]` for downstream consumers
- Right stick X => body yaw setpoint (`twist_pos_rad.z`)
  - stick axis (`RX`) is radial-deadzone filtered and normalized to `[-1, 1]` before yaw scaling
- Triggers (`LT`/`RT`) => body height down/up (`body_trans_m.z`)


## Offline simulation/testing

Use offline mode for deterministic control/safety validation without physical robot hardware.

### 1) Enable simulator mode

Set `Runtime.Mode = "sim"` in `config.txt`, or copy `config.sim.txt` over `config.txt`:

- `Runtime.Mode = "sim"` (required)
- Optional tuning under `Runtime.Sim.*`:
  - `InitialVoltageV`
  - `InitialCurrentA`
  - `ResponseRateHz`
  - `DropBus`, `LowVoltage`, `HighCurrent` (fault injection toggles)

> Scenario runs call `setSimFaultToggles(...)` and require simulator mode.

### 2) Execute scenario runs

Baseline scenarios in `hexapod-server/scenarios/`:

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

Run all scenarios:

```bash
cd hexapod-server
cp config.sim.txt config.txt
for s in scenarios/*.toml; do
  echo "=== Running $s ==="
  ./build/hexapod-server --scenario "$s" || break
done
```

### 3) Run tests

Recommended (preset-based) workflow:

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests
```

Equivalent explicit configure/build commands:

```bash
cd hexapod-server
cmake -S . -B build-tests -DHEXAPOD_SERVER_BUILD_TESTS=ON
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

Run one test binary directly:

```bash
./build-tests/test_robot_runtime_loop
```

Useful focused test binaries:

- `./build-tests/test_control_pipeline_sanity`
- `./build-tests/test_safety_supervisor_faults`
- `./build-tests/test_motion_intent_through_ik_fk`

### 4) Log markers for pass/fail interpretation

Healthy run indicators:

- `Runtime.Mode=sim`
- `Running scenario: <name>`
- expected scenario event logs
- process exits with status `0`

Failure indicators:

- `Failed to load scenario file '<path>'`
- `Scenario driver requires sim runtime (SimHardwareBridge)`
- `LOG_ERROR(...)` entries during execution
- non-zero process exit status

## Configuration (`config.txt`)

Expected TOML fields:

- `title = "Hexapod Config File"`
- `Schema = "hexapod.server.config"`
- `SchemaVersion = 1`
- `SerialDevice` (for example `/dev/ttyACM0`)
- `BaudRate` (for example `115200`)
- `Timeout_ms`
- `MotorCalibrations` with exactly 18 entries:
  - format: `["<JointID>", <min_pulse>, <max_pulse>]`
  - no duplicates or missing joint IDs
  - `500 <= min_pulse < max_pulse <= 2500`

### Stream freshness contract

Control-stage decisions are accepted only when **both** estimator and intent streams satisfy the same freshness contract:

- timestamp rule: timestamp must be present (`timestamp_us != 0`) when `*RequireTimestamp=true`
- sample-id rule: sample id must be present (`sample_id != 0`) when `*RequireSampleId=true`
- monotonicity rule: sample id must never move backward (non-decreasing) across accepted/rejected samples when
  `*RequireMonotonicSampleId=true`
- age rule: `(now - timestamp_us) <= *MaxAgeUs`

If either stream violates the contract at the control boundary:

- control output is gated to `SAFE_IDLE`
- joint targets are zeroed
- `ControlStatus.active_fault` becomes:
  - `ESTIMATOR_INVALID` when estimator stream is invalid/stale
  - `COMMAND_TIMEOUT` when only intent stream is invalid/stale
- runtime diagnostics increment stale/invalid counters and emit structured freshness logs.

Runtime tuning keys for the contract:

- `Tuning.EstimatorMaxAgeUs`
- `Tuning.IntentMaxAgeUs`
- `Tuning.EstimatorRequireTimestamp`
- `Tuning.EstimatorRequireSampleId`
- `Tuning.EstimatorRequireMonotonicSampleId`
- `Tuning.IntentRequireTimestamp`
- `Tuning.IntentRequireSampleId`
- `Tuning.IntentRequireMonotonicSampleId`

> Operator policy: keep freshness max ages aligned with `Tuning.CommandTimeoutUs` unless you intentionally want
> different control-gating vs. safety-latching behavior.

## Control pipeline

Per control step, `ControlPipeline` performs:

1. `GaitScheduler::update(...)`
2. `BodyController::update(...)`
3. `LegIK::solve(...)`
4. control status synthesis (`ControlStatus`)

`SafetySupervisor` runs independently in the safety loop. Cross-loop exchange uses `DoubleBuffer<T>`.

## Protocol bridge notes

- Shared wire constants: `../hexapod-common/include/hexapod-common.hpp`.
- `SimpleHardwareBridge::write()` sends `SET_JOINT_TARGETS` and expects `ACK`.
- `SimpleHardwareBridge::read()` requests `GET_FULL_HARDWARE_STATE` and decodes joints, contacts, voltage, and current.

## Troubleshooting

- **Cannot open serial device**: verify `SerialDevice` path and Linux permissions (`dialout`/udev).
- **ACK timeout / handshake failures**: confirm firmware is running and protocol versions match.
- **Unexpected joint behavior**: re-check calibration ordering and pulse bounds.

## Safety checklist

- Keep robot mechanically unloaded during initial bring-up after calibration edits.
- Validate E-stop path and relay defaults before enabling walking gaits.
- Start with low-amplitude commands when testing new hardware changes.
