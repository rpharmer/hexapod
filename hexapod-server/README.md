# hexapod-server

Linux host-side control process for the hexapod robot. It runs multi-loop control/safety logic against serial hardware or simulator backends (`sim`, `physics-sim`) and streams joint targets.

## Responsibilities

- Load runtime, transport, and calibration settings from `config.txt`.
- Construct the selected runtime hardware bridge (`SimpleHardwareBridge`, `SimHardwareBridge`, or `PhysicsSimBridge`) from config mode.
- Execute `RobotControl` loops (default rates shown; tunable via `Tuning.*LoopPeriod*`):
  - bus loop (500 Hz default)
  - estimator loop (500 Hz default)
  - safety loop (500 Hz default)
  - control loop (250 Hz default)
  - diagnostics loop (2 Hz default)
- Emit `MotionIntent` updates while monitoring health and fault state.

## Directory layout (high level)

```text
hexapod-server/
├── CMakeLists.txt
├── CMakePresets.json
├── config.txt
├── config.sim.txt
├── config.physics-sim.txt
├── README.md
├── include/        # control, config, kinematics, hardware, telemetry interfaces
├── src/            # app entrypoint, runtime loops, parsers, modules
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

Run from `hexapod-server/`:

```bash
cd hexapod-server
cmake -S . -B build
cmake --build build -j
```

Build with tests enabled:

Run from `hexapod-server/`:

```bash
cd hexapod-server
cmake -S . -B build-tests -DHEXAPOD_SERVER_BUILD_TESTS=ON
cmake --build build-tests -j
```

## Run

Run from `hexapod-server/`:

```bash
cd hexapod-server
./build/hexapod-server
```

Stop with `Ctrl+C`.

Run with an explicit config file:

Run from `hexapod-server/`:

```bash
cd hexapod-server
./build/hexapod-server --config config.txt
```

Log file behavior:

- Default remains `app.log` when no log settings are provided.
- Configure defaults in config via `Runtime.Log.FilePath` and `Runtime.Log.EnableFile`.
- Override path at runtime with `--log-file <path>`.
- Disable file logging entirely with `--console-only` (useful for CI/sim/test workflows).
- Optional replay logging is separate from the app log and is configured with `Runtime.ReplayLog.EnableFile` and `Runtime.ReplayLog.FilePath`.
- Diagnostics output now includes periodic `process_resource=...` snapshots with CPU and RSS/VMS data, alongside the existing control-loop and transport health metrics.

Examples:

```bash
./build/hexapod-server --log-file /tmp/hexapod.log
./build/hexapod-server --console-only
```

Controller-driven loop (optional):

Run from `hexapod-server/`:

```bash
cd hexapod-server
./build/hexapod-server --xbox-device /dev/input/eventX
```

Button mappings in controller mode:

- `A` => cycle controller mode:
  1) heading-walk mode
  2) body-pose mode
  3) calibration mode

### 1) Heading-walk mode

- Left stick => walk heading + speed (`heading_rad`, `speed_mps`)
  - stick axes (`LX`,`LY`) are radial-deadzone filtered and normalized to `[-1, 1]` for downstream consumers.
- Right stick => yaw rate (`cmd_yaw_radps`) from horizontal stick deflection.
- Triggers (`LT`/`RT`) => body height down/up (`body_trans_m.z`).
- `X` => RIPPLE gait.
- `Y` => TRIPOD gait.

### 2) Body-pose mode

- Left stick => body XY translation relative to the support legs (`body_trans_m.x/.y`).
- Right stick => body roll/pitch (`twist_pos_rad.x/.y`).
- Triggers (`LT`/`RT`) => body yaw (`twist_pos_rad.z`).
- `LB` / `RB` => increment/decrement body height (`body_trans_m.z`).
- `B` => reset body translation and rotation to neutral.

### 3) Calibration mode

- `B` => run base-height detection probe entrypoint.
- `X` => run servo calibration probe entrypoint.
- `LB` => run servo speed calibration probe entrypoint.
- `Y` => run both probe entrypoints.


### Extending input and hardware integrations

For step-by-step instructions on adding new controller/input drivers and new hardware transport backends, see `../docs/EXTENDING_IO_AND_HARDWARE.md`.

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

> Scenario runs call `setSimFaultToggles(...)`; supported bridges are `SimHardwareBridge` and `PhysicsSimBridge` (`Runtime.Mode = "physics-sim"`). Fault/contact overrides apply only to the lightweight sim bridge; the physics UDP path accepts the hook but ignores injected toggles for now.

### 2) Execute scenario runs

Current scenarios in `hexapod-server/scenarios/`:

- `01_nominal_stand_walk.toml`
- `02_command_timeout_fallback.toml`
- `03_power_fault_triggers.toml`
- `04_contact_loss_edge_cases.toml`
- `05_long_walk_observability.toml`
- `06_map_aware_navigation.toml`
- `07_single_leg_probe.toml`

Convenience script from repository root:

```bash
cd <repo-root>
scripts/run_server_scenarios.sh
```

Launch server + visualiser together (sim mode, telemetry linked):

```bash
cd <repo-root>
scripts/run_sim_stack.sh
```

Run server only (serial or sim) while streaming telemetry to a visualiser IP:

```bash
cd <repo-root>
scripts/run_server_with_telemetry.sh --mode serial --telemetry-host <VISUALISER_IP> --telemetry-port 9870
```

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

Run from `hexapod-server/`:

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests
```

Equivalent explicit configure/build commands:

Run from `hexapod-server/`:

```bash
cd hexapod-server
cmake -S . -B build-tests -DHEXAPOD_SERVER_BUILD_TESTS=ON
cmake --build build-tests -j
ctest --test-dir build-tests --output-on-failure
```

Run one test binary directly:

Run from `hexapod-server/`:

```bash
cd hexapod-server
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
- `Scenario driver requires sim runtime (SimHardwareBridge)` (only if the hardware bridge is neither sim nor physics-sim)
- `LOG_ERROR(...)` entries during execution
- non-zero process exit status

## Configuration (`config.txt`)

Expected TOML fields:

- `title = "Hexapod Config File"`
- `Schema = "hexapod.server.config"`
- `SchemaVersion = 1`
- `MotorCalibrations` with exactly 18 entries:
  - format: `["<JointID>", <min_pulse>, <max_pulse>]`
  - no duplicates or missing joint IDs
  - `500 <= min_pulse < max_pulse <= 2500`

Serial transport keys are required only in `Runtime.Mode = "serial"`:

- `SerialDevice` (for example `/dev/ttyACM0`)
- `BaudRate` (for example `115200`)
- `Timeout_ms`

For the complete key reference (all `Runtime.*`, `Geometry.*`, `Tuning.*`, bounds/defaults, and override semantics), see:

- `../docs/SERVER_CONFIG_REFERENCE.md`

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

1. `CommandGovernor::apply(...)`
2. `LocomotionCommandProcessor::update(...)`
3. `GaitScheduler::update(...)`
4. `LocomotionStability::apply(...)`
5. `BodyController::update(...)`
6. `LegIK::solve(...)`
7. control status synthesis (`ControlStatus`)

`LegIK::solve(...)` evaluates both valid knee branches in the planar chain and keeps the one closest
to the estimator so mirrored legs stay on the physically consistent bend.

`SafetySupervisor` runs independently in the safety loop. Freshness gating also runs independently
before the pipeline in `RobotRuntime::controlStep()`. Cross-loop exchange uses `DoubleBuffer<T>`.

`CommandGovernor` is currently default-constructed inside `ControlPipeline`; verify effective governor
tuning behavior against current code when changing `Tuning.Governor.*` keys.

For a full architecture reference (roles and interactions of supervisor/governor/modules),
see `../docs/ALGORITHMS_SERVER_LOCOMOTION.md` and `../docs/ALGORITHMS_SERVER_CONFIG_TELEMETRY.md`.

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
