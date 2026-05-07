# Testing Functionality Reference

This document describes the current testing surface in the monorepo:

- where tests are defined
- how tests are built and run
- what each major suite validates
- what outputs/artifacts are produced
- which tests provide quantitative motion/performance data (not just pass/fail)

## Motion Performance Quick Index

Use this section as the fast path for before/after motion benchmarking.

- Best primary suite:
  - `hexapod-server/tests/test_locomotion_regression_suite.cpp`
  - Run:
    - `cd hexapod-server`
    - `./build-tests/test_locomotion_regression_suite --sim ../hexapod-physics-sim/build/hexapod-physics-sim --profile canonical --emit-metrics-json`
  - Output:
    - per-case summary lines
    - JSON lines (`--emit-metrics-json`)
    - artifact bundle with per-case `metrics.json`
- Best secondary numeric check:
  - `hexapod-server/tests/test_physics_sim_walk_distance.cpp`
  - CTest name: `physics_sim_walk_distance`
  - Output includes direct motion numbers (distance/path/yaw rate)
- Most useful comparison metrics:
  - `path_length_m`
  - `net_displacement_m`
  - `mean_horizontal_speed_mps`
  - `peak_horizontal_speed_mps`
  - `yaw_delta_rad`
  - `mean_yaw_rate_radps`
  - `peak_yaw_rate_radps`
  - `stride_count`
  - `max_abs_roll_rad`
  - `max_abs_pitch_rad`
  - `max_body_rate_radps`
  - `first_fault`, `first_fault_step`, `final_fault`
- One-command focused run (existing helper):
  - `./scripts/run_locomotion_regression_suite.sh`
- Fast comparison workflow:
  - run suite on baseline and save JSONL
  - run suite on candidate and save JSONL
  - diff selected metrics (prefer medians across multiple runs if flaky)
- Direction / strafe matrix (tripod, physics-backed):
  - `hexapod-server/tests/test_motion_performance_suite.cpp`
  - Cases `compass_forward`, `compass_backward`, `compass_strafe_left`, `compass_strafe_right`, `compass_diag_fwd_left`, `compass_diag_fwd_right` (smoke runs forward + strafe-left only; full profile runs all headings).
  - Each walking case in that binary (compass, slow tripod, gait compare, etc.) applies **plausibility gates** on `WALK` samples: FK foot-tip minimum world Z vs ground; global commanded-vs-measured foot error; **stance** contact-anchor drift and stance tracking error (`locomotion_debug`, same idea as the long-walk regression case); measured foot world Z vs ground; plus **stride kinematics** when enough walk samples exist—median horizontal touchdown span vs commanded `step_length_m`, and a lower percentile of per-swing vertical lift vs `swing_height_m`. Touchdown/liftoff are now derived from explicit `fused_support` transitions in `locomotion_debug`, while `planned_stance` remains a diagnostic comparison channel. Limits are **direction-aware** (tighter forward-like vs strafe/diagonal headings) and sized for long **full** profiles on the UDP sim—not the 0.08 m anchor cap from the intentional long-walk stress case.

## Physics fidelity: `PhysicsSimBridge` (UDP sim) vs `SimHardwareBridge` (synthetic)

- `**PhysicsSimBridge` + `hexapod-physics-sim --serve`**: full `RobotRuntime` loop drives the articulated rigid-body sim over UDP. Locomotion tests that fork the sim (`test_locomotion_regression_suite`, `test_physics_sim_*`, `test_motion_performance_suite`, navigation acceptance, etc.) use this path. This is where **unrealistic commands** show up as penetration, instability, faults, or excessive tracking error.
- `**SimHardwareBridge`**: lightweight in-process state for protocol/runtime tests (e.g. parts of `test_navigation_runtime`). It does **not** enforce rigid-body dynamics or contact physics; passing there does **not** prove motions are physically plausible.
- **Implication:** there is no separate “sim mode” binary for the server—**the same server runtime** runs against either bridge. For “are we driving unrealistic poses?” rely on **physics-sim integration tests** (and FK-based guards like the compass suite), not on `SimHardwareBridge`-only runs.

## Repository-wide test entrypoints

- Primary umbrella command from repo root:
  - `./scripts/verify.sh`
  - Location: `scripts/verify.sh`
  - What it runs (fail-fast):
    - physics sim build (`hexapod-physics-sim`)
    - server tests preset (`hexapod-server`, excludes `locomotion-stress` label by default)
    - firmware host tests (`hexapod-client` host-test preset)
    - server scenario smoke run (`scenarios/01_nominal_stand_walk.toml`)
  - Returns:
    - exit `0` on success
    - non-zero on first failing stage
- Locomotion regression-focused script:
  - `./scripts/run_locomotion_regression_suite.sh`
  - Location: `scripts/run_locomotion_regression_suite.sh`
  - What it runs:
    - rebuilds sim
    - configures/builds server tests preset
    - runs only `locomotion_regression_suite` and `locomotion_regression_suite_stress` CTest entries

## How tests are defined (by component)

## `hexapod-server`

- Definitions:
  - CMake test registration in `hexapod-server/CMakeLists.txt` under `if(HEXAPOD_SERVER_BUILD_TESTS)`
  - Test sources in `hexapod-server/tests/*.cpp`
- Build:
  - `cmake --preset tests`
  - `cmake --build --preset tests -j`
- Run all:
  - `ctest --preset tests --output-on-failure`
- Discover tests:
  - `ctest --preset tests -N`

### Major `hexapod-server` test groups

- Motion performance (metrics tiers, scriptable JSON):
  - `test_motion_performance_suite` — CTest `motion_performance_suite` (smoke), `motion_performance_suite_long` (full + single-leg scenario; label `motion-performance-long`)
- Core control/safety/estimation:
  - `test_control_pipeline_sanity`
  - `test_safety_supervisor_faults`
  - `test_state_fusion`
  - `test_body_controller_height_hold`
  - `test_gait_params`
  - `test_foot_planners_velocity_blend`
  - `test_locomotion_pose_and_stability`
- Kinematics and mappings:
  - `test_motion_intent_through_ik_fk`
  - `test_physics_sim_mapping_roundtrip`
  - `test_physics_sim_ik_wire_roundtrip`
  - `test_physics_sim_bridge_frame_conversion`
- Hardware/transport/protocol:
  - `test_hardware_bridge_transport`
  - `test_transport_components`
  - `test_bridge_command_api`
  - `test_bridge_link_manager`
  - `test_protocol_command_metadata`
- Scenario and runtime integration:
  - `test_scenario_driver_validation`
  - `test_robot_runtime_loop`
  - `test_command_flow_integration`
  - `test_runtime_`* (helpers/diagnostics/teardown/shutdown)
- Navigation/local map:
  - `test_nav_primitives`
  - `test_nav_locomotion_bridge`
  - `test_local_map`
  - `test_local_planner`
  - `test_navigation_*`

### Physics-sim integration tests inside `hexapod-server/tests`

These use `PhysicsSimBridge` + `RobotRuntime` against a live UDP sim process:

- `test_physics_sim_walk_distance`
- `test_physics_sim_walk_stability`
- `test_physics_sim_walk_entry_tracking`
- `test_physics_sim_turn_foot_clearance`
- `test_physics_sim_oblique_walk_clearance`
- `test_physics_sim_turn_raw_contact_loss`
- `test_physics_sim_slow_fwd_walk_foot_clearance`
- `test_physics_sim_wave_slow_walk_foot_clearance`
- `test_physics_sim_slow_fwd_walk_contact_loss`
- `test_physics_sim_tripod_support_baseline`
- `test_physics_sim_nav_waypoints`
- `test_physics_sim_navigation_acceptance`
- `test_physics_sim_serve_zero_g_udp`
- `test_physics_sim_server_initial_layout`

## `hexapod-physics-sim`

- Definitions:
  - CMake: `hexapod-physics-sim/CMakeLists.txt`
  - Most tests registered via helper function `add_minphys3d_test(...)`
- Build + run:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo`
  - `cmake --build build -j`
  - `ctest --test-dir build --output-on-failure`
- Scope:
  - low-level physics engine (contacts, solver blocks, broadphase/narrowphase)
  - articulated/servo behavior
  - scene and protocol checks
  - serve-mode IPC and preview async tests
  - regression scenes and stability probes

Representative tests include:

- `test_regression_step`, `regression_scene_suite`
- `test_block2_solver`, `test_block4_solver`, `test_island_ordering`
- `test_servo_chain_stability`, `test_servo_stability_regression`, `test_servo_regression_matrix`
- `test_servo_torque_saturation_matches_inertia`, `test_servo_stall_under_overload`, `test_servo_chain_vertical_lift_under_gravity`
- `test_hexapod_live`, `test_hexapod_live_pose_hold`, `test_hexapod_planted_foot_drift`, `test_hexapod_zero_g`
- `test_serve_ipc`, `test_serve_ipc_preview`, `test_state_correction_protocol`

### Zero-g no-terrain robustness procedure (self-collision ON)

Use this when validating articulated solver robustness without terrain contacts.

- Enable serve-mode no-contact test:
  - `MINPHYS_HEXAPOD_NO_CONTACT_TEST=1 ./build/hexapod-physics-sim --serve --serve-port 9871`
- Behavior under this mode:
  - gravity is set to zero
  - terrain heightfield is cleared (no terrain/ground contact generation)
  - robot self-collision remains enabled (intentionally, to expose internal blow-ups)
- Pair with server scenario runs (`hexapod-server`) to check for late instability/fault transitions.
- For regression gating, run `test_hexapod_zero_g`:
  - asserts bounded chassis drift/velocity in zero-g
  - asserts tight servo target tracking
  - asserts bounded peak joint speed (`peak_joint_speed <= 8.2 rad/s`)

### Constraint/articulation/torque validation tiers

- **Default-fast CI subset (new; sub-2-minute target):**
  - `test_servo_torque_saturation_matches_inertia`
  - `test_servo_stall_under_overload`
  - `test_servo_chain_vertical_lift_under_gravity`
- **Extended stress/payload subset (`physics-long` label):**
  - `test_hexapod_pose_hold_with_payload`
  - `test_constraint_solver_extreme_mass_ratio_stress`
  - `test_articulation_impulse_response_decay`
- Intent:
  - default-fast focuses on servo torque-limit enforcement and bounded tracking/lift behavior
  - `physics-long` focuses on payload robustness, extreme mass-ratio contact stability, and articulated impulse decay

## `hexapod-client`

- Host-native tests (not on device):
  - CMake option `HEXAPOD_CLIENT_ENABLE_HOST_TESTS=ON`
  - Tests:
    - `hexapod-client.command_router`
    - `hexapod-client.command_handlers`
    - `hexapod-client.command_dispatch`
- Recommended command:
  - `cmake --preset host-tests`
  - `cmake --build --preset host-tests -j`
  - `ctest --preset host-tests --output-on-failure`

## `hexapod-opengl-visualiser`

- CTest-enabled suite in `hexapod-opengl-visualiser/CMakeLists.txt`
- Focus:
  - frame math, transforms, JSON parsing, kinematics, camera, scene bounds, binary packet decode
- Representative tests:
  - `test_visualiser_frame_math`
  - `test_body_pose_transform`
  - `test_json_packets`
  - `test_server_visualiser_roundtrip`
  - `test_kinematics`

## Quantitative motion/performance outputs (most useful)

These are the highest-value tests for tracking improvements across commits.

### 1) `test_locomotion_regression_suite` (server)

- Location:
  - source: `hexapod-server/tests/test_locomotion_regression_suite.cpp`
  - CTest names:
    - `locomotion_regression_suite` (`--profile canonical`)
    - `locomotion_regression_suite_stress` (`--profile stress`)
- Run directly:
  - `./build-tests/test_locomotion_regression_suite --sim <path-to-hexapod-physics-sim>`
  - Optional selectors:
    - `--case <case_name>`
    - `--profile canonical|stress|all`
    - `--artifact-dir <dir>`
    - `--emit-metrics-json` (prints one JSON object per case to stdout)
- What it returns:
  - process exit `0` only when all selected cases pass
  - per-case summary line on stdout (`passed`, `stride_count`, `path_m`, `disp_m`, `roll_max`, `pitch_max`, etc.)
  - failure reason to stderr for failed cases
- Artifacts (default under `/tmp/hexapod_locomotion_regression/<timestamp>/<pid>/`):
  - `manifest.json` (bundle index)
  - per-case:
    - `replay.ndjson` (time-series trace)
    - `geometry.json`
    - `summary.json`
    - `metrics.json` (full scalar metrics)
- Important metrics recorded:
  - global motion: `path_length_m`, `net_displacement_m`, `mean/peak_horizontal_speed_mps`
  - turning: `yaw_delta_rad`, `mean/peak_yaw_rate_radps`
  - stability/safety: `max_abs_roll_rad`, `max_abs_pitch_rad`, `max_body_rate_radps`, `fault` info
  - gait/governor: `stride_count`, command/cadence scales, governed speed limits
  - foot/contact diagnostics: tracking errors, anchor drift, measured foot min-Z

### 2) `test_physics_sim_walk_distance` (server)

- Location: `hexapod-server/tests/test_physics_sim_walk_distance.cpp`
- CTest name: `physics_sim_walk_distance`
- Purpose:
  - quick numeric checks for forward/reverse/straight/turn behavior
- Output:
  - prints useful numeric values (distance deltas, path, yaw delta, avg/peak yaw rate, etc.)
  - exits non-zero if assertions fail

### 3) `test_physics_sim_walk_stability` and related motion checks

- Locations:
  - `hexapod-server/tests/test_physics_sim_walk_stability.cpp`
  - `test_physics_sim_turn_foot_clearance.cpp`
  - `test_physics_sim_*contact_loss*.cpp`
  - `test_physics_sim_tripod_support_baseline.cpp`
- Purpose:
  - targeted acceptance checks around stability, clearance, and contact behavior under specific motion envelopes
- Output:
  - usually assertion-style `FAIL: ...` with non-zero exit
  - some tests print intermediate values useful for triage

## Scenario-driven functional checks

- Scenario definitions:
  - `hexapod-server/scenarios/*.toml`
- Runner:
  - `scripts/run_server_scenarios.sh`
  - or direct server run with `--scenario <file>`
- Purpose:
  - end-to-end functional behavior under scripted events (mode/gait/fault/navigation/sensor overrides)
- Output:
  - process exit code + runtime logs (no unified scalar artifact schema by default)

## Return conventions and failure surface

- Most test binaries follow:
  - `0` success
  - non-zero failure
  - `FAIL: <reason>` style stderr messages
- CTest:
  - marks test failed on non-zero exit or timeout
  - use `--output-on-failure` for detailed logs
- Locomotion regression suite adds richer behavior:
  - pass/fail at process level plus per-case metrics artifacts

## Common commands

- Run all default repo gates:
  - `./scripts/verify.sh`
- Run only server tests (excluding locomotion stress label):
  - `cd hexapod-server && ctest --preset tests -LE locomotion-stress --output-on-failure`
- Run locomotion suites only:
  - `./scripts/run_locomotion_regression_suite.sh`
- Run one locomotion case and print JSON metrics to stdout:
  - `cd hexapod-server`
  - `./build-tests/test_locomotion_regression_suite --sim ../hexapod-physics-sim/build/hexapod-physics-sim --case steady_forward_walk --emit-metrics-json`
- Discover tests:
  - `cd hexapod-server && ctest --preset tests -N`
  - `cd hexapod-physics-sim && ctest --test-dir build -N`
  - `cd hexapod-client && ctest --preset host-tests -N`

## Recommended way to compare motion improvements

For before/after changes in sim/control behavior:

- use `test_locomotion_regression_suite --emit-metrics-json` and/or `metrics.json` artifacts
- compare at least:
  - `path_length_m`
  - `net_displacement_m`
  - `peak_horizontal_speed_mps`
  - `peak_yaw_rate_radps`
  - `yaw_delta_rad`
  - `stride_count`
  - `max_abs_roll_rad`, `max_abs_pitch_rad`, `max_body_rate_radps`
  - fault fields (`first_fault`, `first_fault_step`, `final_fault`)

This provides a repeatable, script-friendly performance envelope, rather than relying on binary pass/fail only.

## Test Taxonomy Table


| Layer                        | Intent                                            | Typical Scope                                          | Representative Tests                                                                                     | Primary Output                         |
| ---------------------------- | ------------------------------------------------- | ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------- | -------------------------------------- |
| Unit math/logic              | Validate deterministic low-level behavior         | Single class/function                                  | `test_gait_params`, `test_nav_primitives`, `test_block2_solver`, `test_json_packets`                     | pass/fail + assertion message          |
| Subsystem integration        | Validate interactions inside one component        | Multiple modules in-process                            | `test_control_pipeline_sanity`, `test_state_fusion`, `test_physics_sim_estimator`                        | pass/fail, occasional debug prints     |
| Bridge/protocol integration  | Validate serialization/transport/wire mapping     | Host <-> bridge/protocol boundaries                    | `test_hardware_bridge_transport`, `test_physics_sim_mapping_roundtrip`, `test_state_correction_protocol` | pass/fail + protocol diagnostics       |
| Runtime-loop integration     | Validate loop choreography and safety transitions | `RobotRuntime` bus/estimator/control/safety sequencing | `test_robot_runtime_loop`, `test_robot_runtime_freshness_gate_matrix`, `test_command_flow_integration`   | pass/fail + fault/status checks        |
| Scenario integration         | Validate scripted end-to-end behavior             | Scenario parser + runtime + bridge                     | `test_locomotion_regression_suite`, `scripts/run_server_scenarios.sh`                                    | summary lines + JSON artifacts (suite) |
| Motion performance benchmark | Quantify movement quality and limits              | Full stack with physics sim                            | `test_locomotion_regression_suite`, `test_physics_sim_walk_distance`                                     | numeric metrics + artifacts            |
| Long/stress behavior         | Catch late-onset instability and regressions      | Extended-duration runs                                 | `locomotion_regression_suite_stress`, `regression_scene_suite`                                           | pass/fail + long-window metrics        |
| Firmware host tests          | Validate firmware logic natively                  | command routing/dispatch                               | `hexapod-client.command_*`                                                                               | pass/fail                              |
| Visualiser tests             | Validate packet decode and render math            | visualiser core parser/math                            | `test_server_visualiser_roundtrip`, `test_kinematics`, `test_camera`                                     | pass/fail                              |


## Coverage Matrix (Behavior vs Current Tests)

Legend:

- `Strong` = direct targeted coverage with clear assertions
- `Partial` = indirect coverage or limited envelope
- `Gap` = little/no direct targeted test


| Behavior                                          | Current Coverage | Primary Tests                                                                                                                                                  | Notes                                                                                                   |
| ------------------------------------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------- |
| Stand/pose hold                                   | Strong           | `test_hexapod_live_pose_hold`, `test_physics_sim_server_initial_layout`                                                                                        | low drift / settle behavior covered                                                                     |
| Three-leg static support strength                 | Strong           | `test_physics_sim_tripod_support_baseline`, `single_leg_masked_stand` in `test_motion_performance_suite`                                                      | tracks body creep, joint drift, and support-foot tracking under asymmetric load                         |
| Straight walking distance/speed                   | Strong           | `test_physics_sim_walk_distance`, `steady_forward_walk` case in `test_locomotion_regression_suite`                                                             | quantitative path/speed metrics available                                                               |
| Strafing / diagonal walking (body-frame headings) | Strong           | `test_motion_performance_suite` (`compass_*` cases), oblique clearance in `test_physics_sim_oblique_walk_clearance`, lateral gait checks in `test_gait_params` | compass + other walk cases: FK ground, tracking, stance anchor drift, measured stride span / swing lift |
| Reverse walking                                   | Strong           | `test_physics_sim_walk_distance`                                                                                                                               | explicit reverse case                                                                                   |
| Turn-in-place/yaw performance                     | Strong           | `turn_in_place` case in `test_locomotion_regression_suite`, `test_physics_sim_walk_distance`                                                                   | yaw delta and yaw-rate metrics                                                                          |
| Gait transitions (tripod/ripple/wave)             | Strong           | `gait_transition_stability` case in `test_locomotion_regression_suite`                                                                                         | phase-segment metrics captured                                                                          |
| Governor shaping under aggressive commands        | Strong           | `aggressive_governor` case in `test_locomotion_regression_suite`                                                                                               | command/cadence scaling metrics                                                                         |
| Contact-loss during swing                         | Strong           | `test_physics_sim_turn_raw_contact_loss`, `test_physics_sim_slow_fwd_walk_contact_loss`                                                                        | explicitly expects loss events                                                                          |
| Foot clearance envelope                           | Strong           | `test_physics_sim_turn_foot_clearance`, `test_physics_sim_*_foot_clearance`                                                                                    | clearance checks in targeted profiles                                                                   |
| Low-support/sparse-support locomotion             | Strong           | `low_support_walk` case in `test_locomotion_regression_suite`                                                                                                  | support margin + faults tracked                                                                         |
| Long-horizon observability/late faults            | Strong           | `long_walk_observability` (`stress`)                                                                                                                           | delayed instability envelopes                                                                           |
| Navigation + locomotion coupling                  | Partial          | `test_physics_sim_navigation_acceptance`, `test_physics_sim_nav_waypoints`, `test_navigation_runtime`                                                          | coverage exists but fewer rich motion metrics                                                           |
| Per-leg single-foot placement primitive           | Partial          | `single_leg_masked_stand` in `test_motion_performance_suite` + `scenarios/07_single_leg_probe.toml`                                                            | IK mask via `safety.legs_enabled`; not a full placement primitive                                       |
| Hardware-in-the-loop real robot metrics           | Gap              | outside CI/unit tests                                                                                                                                          | currently sim-dominant metrics                                                                          |


## Metrics Glossary

Metrics below are primarily emitted by `test_locomotion_regression_suite` into `metrics.json` and optional `--emit-metrics-json`.

- `sample_count`: total control-loop samples captured in a case.
- `walk_sample_count`: count of samples where mode is `WALK`.
- `stride_count`: estimated completed strides from stride phase accumulation.
- `planned_stance`: gait-scheduler stance intent per leg in `locomotion_debug`.
- `raw_contact`: raw contact bit per leg in `locomotion_debug`.
- `fused_support`: controller-facing physical support bit per leg in `locomotion_debug`.
- `fused_contact_phase`, `fused_contact_confidence`: fused contact-state diagnostics per leg.
- `path_length_m`: accumulated planar path (`sum(horizontal_speed * dt)`).
- `net_displacement_m`: start-to-end planar displacement magnitude.
- `lateral_deviation_m`: off-track lateral deviation relative to travel direction.
- `mean_horizontal_speed_mps`, `peak_horizontal_speed_mps`: planar speed summary.
- `yaw_delta_rad`: wrapped start-to-end yaw change.
- `mean_yaw_rate_radps`, `peak_yaw_rate_radps`: yaw-rate summary.
- `max_abs_roll_rad`, `max_abs_pitch_rad`: max absolute body tilt.
- `max_body_rate_radps`: max planar body angular rate (`sqrt(gyro_x^2 + gyro_y^2)` when IMU valid).
- `min_support_margin_m`: minimum static stability margin reported by gait stability logic.
- `min_model_trust`: minimum fusion trust over case.
- `max_contact_mismatch_ratio`: max contact mismatch ratio from fusion diagnostics.
- `min_command_scale`, `min_cadence_scale`: minimum governor scales reached.
- `max_governor_severity`: maximum governor severity.
- `max_governed_speed_mps`, `min_governed_speed_mps`: governed speed bounds.
- `max_governed_yaw_rate_radps`, `min_governed_yaw_rate_radps`: governed yaw-rate bounds.
- `max_step_length_m`, `min_step_length_m`: gait step length bounds.
- `max_swing_height_m`, `min_swing_height_m`: gait swing-height bounds.
- `max_contact_anchor_drift_m`, `max_contact_anchor_max_drift_m`: contact anchor drift diagnostics.
- `max_commanded_tracking_error_m`, `max_contact_tracking_error_m`: commanded vs measured foot tracking error diagnostics.
- `tripod_body_height_creep_m`, `tripod_max_support_joint_drift_rad`, `tripod_max_support_commanded_tracking_error_m`: phase-0 static-support metrics from `test_physics_sim_tripod_support_baseline`.
- `first_fault`, `first_fault_step`, `final_fault`, `saw_fault`: fault lifecycle summary.
- `mode_segments`: contiguous mode windows with start/end step.
- `gait_segments`: contiguous phase windows with mean step/duty fields.

## A/B Benchmark Protocol

Use this protocol for reproducible before/after comparisons.

1. Lock environment:
  - same machine type
  - same config (`hexapod-server/config.physics-sim-wsl.txt` or `config.physics-sim.txt`)
  - same build type (e.g. RelWithDebInfo)
2. Build baseline sim + server tests.
3. Run baseline N times (recommended N>=10 for flaky cases):
  - `test_locomotion_regression_suite --profile canonical --emit-metrics-json`
  - store JSONL as `baseline.jsonl`
4. Build candidate (new commit) with same options.
5. Run candidate N times and store `candidate.jsonl`.
6. Compare robust statistics per case/metric:
  - median
  - p90/p95
  - min/max
7. Flag regressions with explicit tolerances (project-defined), for example:
  - distance/speed/yaw metrics degrade beyond threshold
  - fault frequency increases
  - body-rate/tilt envelopes increase beyond tolerance
8. Keep raw artifacts:
  - JSONL
  - manifest and per-case metrics files
  - commit SHA + config file hash

Minimal command skeleton:

```bash
# baseline
./build-tests/test_locomotion_regression_suite --sim ../hexapod-physics-sim/build/hexapod-physics-sim --profile canonical --emit-metrics-json 2>/dev/null | grep -E '^\{"(suite|name)"' > baseline.jsonl

# candidate
./build-tests/test_locomotion_regression_suite --sim ../hexapod-physics-sim/build/hexapod-physics-sim --profile canonical --emit-metrics-json 2>/dev/null | grep -E '^\{"(suite|name)"' > candidate.jsonl
```

### Unified metrics JSONL schema (`--emit-metrics-json`)

Quantitative integration binaries emit **one JSON object per case** on stdout (often filtered with `grep -E '^\{"(suite|name)"'` or `grep '^{'` to strip logs). Consumers should treat unknown fields as forward-compatible.


| Field            | Type    | Description                                                                                                                                                |
| ---------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `suite`          | string  | Stable id: `motion_performance`, `locomotion_regression`, `physics_sim_navigation_acceptance`, `physics_sim_walk_distance`, etc.                           |
| `name`           | string  | Case id (matches `--case` or catalog name).                                                                                                                |
| `passed`         | boolean | Whether gates passed.                                                                                                                                      |
| `metrics`        | object  | Observed scalars (and nested objects) for analytics and tightening.                                                                                        |
| `limits_applied` | object  | Numeric floors/ceilings **used for the pass/fail decision** in that run (self-describing baselines). May include `gate_profile` (e.g. forward vs lateral). |


Legacy lines may omit `suite` or `limits_applied`; tools should default `suite` from context or infer from binary.

**Tightening workflow:** run the same binary **N** times (N ≥ 5 for noisy sim metrics) with fixed sim/config/build, append lines to a single JSONL (for example with `[scripts/capture_physics_sim_metrics_jsonl.sh](scripts/capture_physics_sim_metrics_jsonl.sh)` in a shell loop), then run `python3 scripts/suggest_gate_limits.py [--slack 1.05] combined.jsonl` to summarize per `(suite, name)` percentiles and suggested ceilings. Update compile-time defaults in the matching test source and/or the shared manifest when metrics improve; prefer `--only-passed` if failed runs inject partial `metrics` that would skew percentiles.

### P3: External limits manifests (`schema_version` 1)

Quantitative integration tests load optional numeric gates from a **versioned JSON manifest** so CI and developers can tune thresholds without recompiling. If no manifest is loaded, each test keeps its historical compile-time defaults (behavior unchanged).

- **Default file:** `[docs/testing-baselines/gates/default-v1.json](../testing-baselines/gates/default-v1.json)` — `schema_version` must be `1`. The server test build records its absolute path as `HEXAPOD_DEFAULT_LIMITS_MANIFEST` so binaries auto-load it when the file exists.
- **Overrides (highest precedence first):** `--limits-manifest <path>` on argv, then `HEXAPOD_TEST_LIMITS_MANIFEST`. If argv or env points at a missing or invalid file, the binary exits with an error before starting the sim.
- **Layout:** `suites.<suite_id>.<case_name>` holds flat numeric (or boolean) keys, merged with optional `suites.<suite_id>._defaults` and optional per-profile objects (for example `motion_performance.walk_gates.forward_like`, or `physics_sim_nav_waypoints.follow_waypoints.strict` vs `.loose` selected by `HEXAPOD_STRICT_PHYSICS_NAV`).
- **Alignment:** Keys should match the `suite` / `name` strings in JSONL and the fields documented in `limits_applied` for each case. Missing keys fall back to compile defaults; when a manifest was explicitly requested (argv or env), missing keys emit a **one-time stderr warning** per `(suite, name, profile, key)`.
- **CTest:** Configure with `-DHEXAPOD_CTEST_USE_LIMITS_MANIFEST=ON` to inject `HEXAPOD_TEST_LIMITS_MANIFEST` into manifest-aware `add_test` entries (see `hexapod-server/CMakeLists.txt`). Leave off for the historical default (no env injection).
- **Tooling:** `python3 scripts/suggest_gate_limits.py --emit-manifest-fragment --fragment-suite SUITE --fragment-name CASE [--slack …] file.jsonl` prints a small JSON document you can merge under `suites` by hand.

Manifests are local test data only; do not auto-fetch them from the network.

## Artifacts Schema Reference

Primary schema source: artifacts produced by `test_locomotion_regression_suite`.

- `manifest.json`
  - top-level bundle index
  - contains `cases` array with summary objects
- `<case>/summary.json`
  - case metadata
  - pass/fail and reason
  - pointers to replay/metrics/geometry files
- `<case>/metrics.json`
  - serialized `LocomotionMetrics`
  - scalar and segmented metrics listed in glossary above
- `<case>/replay.ndjson`
  - per-step replay telemetry records
  - intended for detailed post-mortem and visual replay tooling
- `<case>/geometry.json`
  - geometry snapshot used for the run

Practical consumer contract:

- rely on `metrics.json` for scalar analytics
- use `replay.ndjson` only when deeper timeline diagnostics are needed
- treat unknown JSON fields as forward-compatible additions

## Command Cookbook

- Configure + build server test preset:
  - `cd hexapod-server && cmake --preset tests && cmake --build --preset tests -j`
- List available server tests:
  - `cd hexapod-server && ctest --preset tests -N`
- Run one server test by name:
  - `cd hexapod-server && ctest --preset tests -R '^physics_sim_walk_distance$' --output-on-failure`
- Run all server tests except locomotion stress:
  - `cd hexapod-server && ctest --preset tests -LE locomotion-stress --output-on-failure`
- Run locomotion canonical only:
  - `cd hexapod-server && ctest --preset tests -R '^locomotion_regression_suite$' --output-on-failure`
- Run locomotion stress only:
  - `cd hexapod-server && ctest --preset tests -R '^locomotion_regression_suite_stress$' --output-on-failure`
- Run motion performance suite (smoke CTest; full tier + gait compare + `single_leg_masked_stand` is `motion_performance_suite_long`):
  - `cd hexapod-server && ctest --preset tests -R '^motion_performance_suite$' --output-on-failure`
  - `cd hexapod-server && HEXAPOD_PHYSICS_SIM_EXE=../hexapod-physics-sim/build/hexapod-physics-sim ./build-tests/test_motion_performance_suite --profile full --emit-metrics-json`
- A/B JSONL capture helper:
  - `./scripts/run_motion_benchmark_ab.sh ./hexapod-server/build-tests/test_motion_performance_suite ./hexapod-server/build-tests/test_motion_performance_suite`
- Tier-B physics sim binaries (`test_physics_sim_`*): pass `--emit-metrics-json` and the sim path (or set `HEXAPOD_PHYSICS_SIM_EXE`); capture lines with `./scripts/capture_physics_sim_metrics_jsonl.sh OUT.jsonl ./hexapod-server/build-tests/<binary> [<sim path>]`
- Navigation acceptance with JSON metrics lines:
  - `cd hexapod-server && HEXAPOD_PHYSICS_SIM_EXE=../hexapod-physics-sim/build/hexapod-physics-sim ./build-tests/test_physics_sim_navigation_acceptance --emit-metrics-json`
- Run suite binary directly with specific case:
  - `cd hexapod-server && ./build-tests/test_locomotion_regression_suite --sim ../hexapod-physics-sim/build/hexapod-physics-sim --case aggressive_governor --emit-metrics-json`
- Run physics sim test catalog:
  - `cd hexapod-physics-sim && ctest --test-dir build --output-on-failure`
- Run focused zero-g robustness regression:
  - `cd hexapod-physics-sim && ctest --test-dir build -R '^test_hexapod_zero_g$' --output-on-failure`
- Run focused default-fast solver/torque validation subset:
  - `cd hexapod-physics-sim && ctest --test-dir build -R 'test_servo_torque_saturation_matches_inertia|test_servo_stall_under_overload|test_servo_chain_vertical_lift_under_gravity' --output-on-failure`
- Run extended payload/robustness subset:
  - `cd hexapod-physics-sim && ctest --test-dir build -L physics-long --output-on-failure`
- Start no-terrain contact serve-mode run (self-collision enabled):
  - `cd hexapod-physics-sim && MINPHYS_HEXAPOD_NO_CONTACT_TEST=1 ./build/hexapod-physics-sim --serve --serve-port 9871`
- Run firmware host tests:
  - `cd hexapod-client && cmake --preset host-tests && cmake --build --preset host-tests -j && ctest --preset host-tests --output-on-failure`
- Run full repo verification:
  - `./scripts/verify.sh`

## Historical Baselines

Store benchmark snapshots as JSONL + metadata for traceability.

Recommended baseline record structure:

- `baseline_name`: human label (e.g. `canonical-wsl2-2026-05`)
- `server_commit`: commit SHA of `hexapod-server`
- `sim_commit`: commit SHA of `hexapod-physics-sim`
- `config_path`: config file used
- `config_digest`: optional hash of config content
- `build_type`: e.g. `RelWithDebInfo`
- `runs`: number of repetitions
- `jsonl_path`: metrics JSONL output path
- `notes`: anomalies, known caveats

Suggested storage location:

- `docs/testing-baselines/`
- Example files:
  - `docs/testing-baselines/README.md`
  - `docs/testing-baselines/baseline-<date>-<serverSHA>-<simSHA>.json`
  - `docs/testing-baselines/baseline-<date>-<serverSHA>-<simSHA>.jsonl`

If exact commit IDs are unavailable for a run, use best alternatives:

- nearest tagged release
- branch + timestamp
- archived build artifact checksum

## Implementation backlog (gap → deliverable)

Working plan to close taxonomy/coverage gaps. **Concrete execution items subsume overlapping Roadmap bullets** below; the Roadmap stays as a short wish-list.


| Priority | Gap (from taxonomy / coverage matrix)                 | Deliverable                                                                 | CTest / binary                                                             | Key files                                                                                                                                                                                                                                                                                                                                                                                                            | Output                                               | Dependencies                            |
| -------- | ----------------------------------------------------- | --------------------------------------------------------------------------- | -------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------- | --------------------------------------- |
| P0       | Doc traceability                                      | This table + design notes                                                   | (doc only)                                                                 | [docs/TESTING_FUNCTIONALITY.md](TESTING_FUNCTIONALITY.md)                                                                                                                                                                                                                                                                                                                                                            | —                                                    | —                                       |
| P1       | Duplicated motion metrics logic                       | Shared `LocomotionMetrics` + JSON + aggregation                             | (library header used by suites)                                            | [hexapod-server/tests/locomotion_metrics.hpp](hexapod-server/tests/locomotion_metrics.hpp), [hexapod-server/tests/locomotion_motion_sequence.hpp](hexapod-server/tests/locomotion_motion_sequence.hpp)                                                                                                                                                                                                               | same `metrics.json` fields as regression             | —                                       |
| P2       | Motion performance / per-leg stats + compass headings | `test_motion_performance_suite` (`compass_`*, FK/tracking gates on UDP sim) | `motion_performance_suite`, optional `motion_performance_suite_long`       | [hexapod-server/tests/test_motion_performance_suite.cpp](hexapod-server/tests/test_motion_performance_suite.cpp), [hexapod-server/CMakeLists.txt](hexapod-server/CMakeLists.txt)                                                                                                                                                                                                                                     | stdout JSON lines (`--emit-metrics-json`), pass/fail | built sim via `HEXAPOD_PHYSICS_SIM_EXE` |
| P3       | Per-leg primitive / placement                         | Scenario `safety.legs_enabled` + runtime merge                              | case `single_leg_masked_stand` in motion suite                             | [hexapod-server/include/scenario/scenario_driver.hpp](hexapod-server/include/scenario/scenario_driver.hpp), [hexapod-server/src/scenario/scenario_driver.cpp](hexapod-server/src/scenario/scenario_driver.cpp), [hexapod-server/src/control/robot_runtime.cpp](hexapod-server/src/control/robot_runtime.cpp), [hexapod-server/scenarios/07_single_leg_probe.toml](hexapod-server/scenarios/07_single_leg_probe.toml) | metrics + soft gates                                 | scenario schema change                  |
| P4       | Navigation quantitative A/B                           | JSON line metrics on nav acceptance                                         | `physics_sim_navigation_acceptance` (flags)                                | [hexapod-server/tests/test_physics_sim_navigation_acceptance.cpp](hexapod-server/tests/test_physics_sim_navigation_acceptance.cpp)                                                                                                                                                                                                                                                                                   | optional `--emit-metrics-json`                       | —                                       |
| P5       | Benchmark automation + wire robustness                | A/B shell script; extra transport cases                                     | `scripts/run_motion_benchmark_ab.sh`; `hardware_bridge_transport_failures` | [scripts/run_motion_benchmark_ab.sh](scripts/run_motion_benchmark_ab.sh), [hexapod-server/tests/test_hardware_bridge_transport_failures.cpp](hexapod-server/tests/test_hardware_bridge_transport_failures.cpp)                                                                                                                                                                                                       | JSONL + text summary                                 | —                                       |
| P6       | HIL / field parity                                    | Checklist only in CI                                                        | —                                                                          | (document field parity vs `LocomotionMetrics`)                                                                                                                                                                                                                                                                                                                                                                       | future robot logs                                    | hardware                                |


**Single-leg isolation design:** Prefer **scenario-driven** `safety.legs_enabled` (six booleans) on timeline events, merged into effective safety after `SafetySupervisor` when present, so cases stay data-defined like other scenario toggles. Alternative: a test-only `RobotRuntime::setTestLegEnabledMask`—rejected as primary path to avoid ad hoc C++-only coverage.

**Motion performance gates:** Tier 1–2 use **hard** fail only on severe faults (`TIP_OVER`, `ESTIMATOR_INVALID`, etc.); tilt/body-rate bands start as **warnings** on stderr until baselines exist under [docs/testing-baselines/](docs/testing-baselines/).

**HIL note:** Out of scope for automated CI here; when adding robot runs, mirror scalar fields from `LocomotionMetrics` / `metrics.json` where possible (see Metrics Glossary).

## Roadmap

Higher-level ideas not tied to a single PR (see **Implementation backlog** for scheduled work):

- Add percentile-based regression gates (median/p95) for flaky motion cases.
- Add artifact schema docs generated from code (keep JSON fields in sync).
- Add dashboard export (CSV/JSON) for trends over commits.
- Extend HIL with the schema parity checklist in the backlog table.
