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

For the simulator-only swing-rate screen and offline coupled response predictor,
see [Testing functionality](../docs/TESTING_FUNCTIONALITY.md). The predictor is
diagnostic only and does not change runtime commands or solver selection.

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

For the Xbox Wireless Adapter under WSL2, see `docs/WSL_XBOX_CONTROLLER.md`
(`scripts/attach_xbox_wsl.ps1` then `scripts/find_xbox_evdev.sh`). Power the pad
**off** before attaching the dongle or usbipd reports Device busy.

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

- Left stick => walk heading + speed (`heading_rad`, `speed_mps`), up to `0.06 m/s`.
  - stick axes (`LX`,`LY`) are radial-deadzone filtered and normalized to `[-1, 1]` for downstream consumers.
- Right stick => yaw rate (`cmd_yaw_radps`) from horizontal stick deflection, including turning in place, up to `0.45 rad/s`.
- Walk activation uses centre hysteresis so small stick fluctuations do not repeatedly restart the gait.
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
- `05_long_walk_contact_health.toml`
- `06_map_aware_navigation.toml`
- `07_single_leg_probe.toml`

For live-physics regression A/B runs, from `hexapod-server/` use
`./build-tests/test_locomotion_regression_suite --profile canonical --case long_walk_contact_health --solver-mode pinocchio-compliant`.

The unvalidated simulator-only coordinated swing-rate experiment is opt-in with
`HEXAPOD_SWING_LINK_RATE_EXPERIMENT=1`. It retains the production solver and speed
guards, and does not change stance or STAND recovery. The bounded command budget
is `HEXAPOD_SWING_LINK_RATE_BUDGET_RADPS=10|9|8` (default 10); optional per-step
diagnostics use `HEXAPOD_SWING_LINK_RATE_TRACE=1`. See
[`PLAN_COORDINATED_SWING_RATE_GOVERNOR.md`](../docs/PLAN_COORDINATED_SWING_RATE_GOVERNOR.md).
From repository root after loading `scripts/lib/pinocchio_env.sh`, use
`python3 tools/run_swing_link_rate_screen.py --screen aggressive --runs 3 --output-dir /tmp/hexapod-link-rate-screen`
for machine-readable results. Add `--baseline` for the unchanged path; other
screens are `straight`, `turn`, and `sequential`.
The regression harness defaults to rigid mode 1; its explicit selector and
reported `solver_mode` make the opt-in mode auditable. This does not change WSL
production configuration. Unset the compliant experiment environment override
when comparing protocol modes.

`01_nominal_stand_walk.toml` is the smoke scenario for both bridges. Its walk phases stay at
0.04 m/s and use modest yaw changes so it is suitable for the articulated physics stack. Keep
high-speed or abrupt-heading experiments in dedicated stress scenarios; a synthetic `sim` pass
does not establish physics-sim safety.

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

### Visualiser command channel (opt-in)

Interactive mode can listen for UDP JSON commands (default port **9872**) so the
OpenGL visualiser can list/run/stop scenarios and send thin nav/motion commands.
The binary default is off; stack launchers enable it:

```bash
# from repo root — command channel on 9872
scripts/run_physics_stack.sh --controller-optional
scripts/run_sim_stack.sh

# binary / telemetry helper (explicit)
cd hexapod-server
./build/hexapod-server --telemetry-enable --command-enable \
  --command-port 9872 --command-scenarios-dir scenarios
```

See [`docs/VISUALISER_COMMAND_CHANNEL.md`](../docs/VISUALISER_COMMAND_CHANNEL.md).

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

### Self-weight-only physics experiment

From the repository root, after rebuilding the simulator and both
`test_physics_sim_walk_distance` and `test_locomotion_regression_suite` together:

```bash
tools/run_gait_feasibility_batch.sh /tmp/selfweight-new 5 baseline selfweight
python3 tools/summarize_gait_feasibility.py /tmp/selfweight-new
```

The `selfweight` arm sets test-only `HEXAPOD_WALK_TEST_SELF_WEIGHT=1`: Bounded
feedforward, link self-weight on, foot-reaction off, femur/tibia scale 1,
stiffness scale 1 and an 80 ms filter. Existing maximum angle offsets and
IMU/feedback thresholds are retained. The correction goes through normal target
limits and uses per-joint nominal stiffness reported in the physics response,
not a private simulator oracle or direct torques. Rejected IMU inputs decay
through the configured filter; disabling FF clears it immediately. It is
not a production preset. True measured clearance and unchanged behavioural
gates, not error to the deliberately biased servo target, decide acceptance.
Do not combine this experiment with other tuning arms when attributing results.
See [leftovers §3.22](../docs/SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md#322-explicit-actuator-stiffness-and-gatefilter-continuity)
for the conversion fix, repeated screens and remaining held states.

The same screen runner accepts `velocitylead` and `velocityleadfiltered` for
**rejected, test-only** counterfactuals, not production presets. Both act in the
walk-distance/regression bridge after normal target processing. The former adds
`0.08 s * reference rate` to the motor target; the latter smooths that offset
with the existing 80 ms motor timescale. Neither preserves the reference slew
bound on the final biased motor target, and neither enables gravity FF. Do not
use lower foot dragging alone as an acceptance result: both failed safety/live
walking screens. `tools/report_gait_screen.py RUN_DIR NEW_REPORT.json
--experiment-note "..."` retains a non-overwriting scorecard with binary/log
hashes. See [§3.23](../docs/SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md#323-moving-damping-balance-rejected-velocity-lead-final-target-rate-repair).

Runtime target-speed metadata in STAND/WALK describes the final emitted angle
difference after all safety adjustments. This repair changes no target angle
and does not enable motor velocity feedforward or an additional slew clamp.

In physics-sim, a pure turn-in-place now holds its entry XY using the existing
bounded correction (0.20/s, at most 0.03 m/s). This requires the bridge's explicit
absolute-position capability; hardware and the simple simulator remain excluded.
Intentional translating turns are not anchored. `HEXAPOD_TURN_INPLACE_HOLD=0`
disables it for comparison. Normal launches need no experiment flags.
The explicit simulator's speed-limit retries now retain motor damping while
reducing position drive; no speed/torque or test limit was raised.

`storedmotion` remains a rejected test-only screen: it bounds the PD error's
equivalent velocity request, not the contact-driven physical velocity. The
`capture` arm retains the first speed-limit state and preceding accepted history
without overwriting. `legacyrecovery` disables both the retry damping fix and
turn hold for old/new comparisons. Batch outputs include `.exit` status files;
machine reports require a successful process exit as well as passing metrics.
For historical runs without sidecars, pass `--progress-log` to the report tool;
unknown process status is not a qualified pass.

- **Cannot open serial device**: verify `SerialDevice` path and Linux permissions (`dialout`/udev).
- **ACK timeout / handshake failures**: confirm firmware is running and protocol versions match.
- **Unexpected joint behavior**: re-check calibration ordering and pulse bounds.

## Safety checklist

Walking clearance diagnostics (repo-root commands): create an empty output
directory, then set `HEXAPOD_MOTION_TRACE_DIR=/absolute/output/directory` when
running `test_physics_sim_walk_entry_tracking` or
`test_motion_performance_suite --profile full`. Each writes its collected
trace after simulation and refuses to overwrite existing files. Inspect with:

```bash
python3 tools/analyze_walk_support.py /absolute/output/directory/walk_entry.ndjson
python3 tools/analyze_walk_support.py /absolute/output/directory/walk_entry.ndjson --window 2500 2600 --leg 4
```

These are diagnostic traces, **not executable replay fixtures**: motion-suite
traces do not record bus joint commands. Both measured-contact swing-height
correction and the post-reach, contact-referenced liftoff floor are enabled by
default. `HEXAPOD_SWING_CONTACT_HEIGHT=0` disables only the former;
`HEXAPOD_SWING_CONTACT_CLEARANCE_SCREEN=0` disables only the latter for
diagnostic comparisons (`=1` explicitly enables it). Normal launches need no
experiment flags.
See [walking campaign §3.26](../docs/SEQUENTIAL_WALK_DISTANCE_LEFTOVERS.md).

Tilt regression reporting distinguishes pre-fault walking from post-fault
drift. `tilt_safety_trip` checks normal travel followed by an unsafe command;
`tilt_safety_immediate` checks the original immediately unsafe command without
a distance quota. Neither changes the runtime safety thresholds. To inspect
an existing replay (from repo root):

```bash
python3 tools/audit_tilt_trip.py /absolute/path/to/tilt_safety_trip/replay.ndjson
```

The audit also needs the recording's sibling `metrics.json`. See
[testing reference](../docs/TESTING_FUNCTIONALITY.md) for metric semantics.

The live locomotion regression binary accepts `--perturbation-seed N` for a
bounded initial-body-pose perturbation and a matching simulator contact-order
seed. This is **not** the frozen exact-command replay: the live controller and
simulator still run as separate processes, so a seed does not guarantee
identical timing or outcomes between repetitions. From the repository root,
with the Pinocchio environment and `HEXAPOD_PHYSICS_SIM_EXE` set, for example:

```bash
hexapod-server/build-tests/test_locomotion_regression_suite \
  --profile stress --case long_walk_observability \
  --perturbation-seed 1 --artifact-dir /var/tmp/hexapod-walk-seed-1 \
  --emit-metrics-json
python3 scripts/analyze_rare_locomotion_failure.py \
  /var/tmp/hexapod-walk-seed-1/long_walk_observability/replay.ndjson --leg 2
```

For a traced first held sample, set `HEXAPOD_LOCOMOTION_CHILD_STDIO=1`,
`HEXAPOD_PROXIMAL_TRACE_FAILURES=1` and
`HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT=1`, then pipe combined test output through
`python3 scripts/extract_first_hold_trace.py` with `set -o pipefail`. The filter
drains the child stream, retains the immediate pre-hold lines and prints the
machine-readable suite result; it does not change the test gate.

- Keep robot mechanically unloaded during initial bring-up after calibration edits.
- Validate E-stop path and relay defaults before enabling walking gaits.
- Start with low-amplitude commands when testing new hardware changes.
