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

`test_physics_sim_walk_entry_tracking` is the regression guard for STAND→WALK. It runs the
production command shaper at 0.04 m/s and requires a bounded measured joint-rate peak as well as
the height, support, contact-mismatch, and tracking checks. It should be run whenever gait timing,
IK, or servo dynamics change.

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
  - verifies signed projection onto the commanded body heading and commanded yaw direction, so backwards travel cannot pass as forward progress
- Output:
  - prints useful numeric values (distance deltas, path, yaw delta, avg/peak yaw rate, etc.)
  - JSON metrics include the body-height envelope and maximum/RMS applied-target
    servo tracking error, allowing gait collapse to be separated from contact slip
    or solver failure
  - JSON also reports the maximum applied target rate overall and by joint type,
    plus the number of target samples above the MG996R no-load speed; this exposes
    gait commands that no torque-limited motor can track under load
  - exits non-zero if assertions fail
  - `HEXAPOD_WALK_TEST_CHILD_STDIO=1` exposes simulator diagnostics during
    failure tracing; child output remains quiet by default
  - proximal sweeps can override `HEXAPOD_WALK_TEST_ABSOLUTE_TOLERANCE` and
    `HEXAPOD_WALK_TEST_RELATIVE_TOLERANCE`; these are diagnostic controls and
    do not relax the production defaults

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

### 4) `test_physics_sim_exact_command_replay` (proximal acceptance gate)

- Location: `hexapod-server/tests/test_physics_sim_exact_command_replay.cpp`
- Fixture: `hexapod-server/tests/fixtures/hexapod-commands-v16.txt` (hash
  `ddc6008e0cc1ac97`, 720 frames, capture 5000 µs). Do not recapture for CTest.
- Default live-physics CTests (cap 24, both enforce flags, seed 0):
  - `physics_sim_exact_command_replay` (fixture capture period)
  - `physics_sim_exact_command_replay_120hz` (`PERIOD_US=8333`)
  - `physics_sim_exact_command_replay_240hz` (`PERIOD_US=4167`)
  - `physics_sim_exact_command_replay_480hz` (`PERIOD_US=2083`)
- 100-seed perturbation remains a diagnostic override
  (`HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS`), not a default CTest.
- Purpose:
  - capture the exact `JointTargets` produced by a physically coupled legacy
    reference run
  - replay the identical target stream through `pinocchio-proximal`, or through
    `legacy-pgs` as a control experiment
  - separate controller/safety feedback from contact-solver failures
  - report healthy, recovered, held, and unsupported samples for stand,
    forward, reverse, strafe, diagonal, turn-in-place, and stand transitions
  - report segment-correct body-frame progress, lateral drift, horizontal path,
    yaw change, and body-height error for quantitative behaviour gating
  - report capture-side motion inhibition, WALK-mode residency, and joint-target
    variation so a frozen or safety-inhibited command stream cannot masquerade as
    a contact-solver locomotion result
  - count reverse stance-target steps (`|Δp_xy| > 5 mm` against the command)
    split by planned stance, held swing, held stance, and unclassified
    (late-swing contact is invisible to the planned/hold flags)
  - split those reverse steps into onset (new plant or `φ/duty ≤ 0.05`) vs
    continuing mid-stance (`0.05 < φ/duty < 0.95` and already planned)
  - report capture-side mid-stance commanded stroke (opposition / counter-yaw)
    overall and split by tripod (`n_planned == 3`), overlap (`n_planned ≥ 4`),
    and high-duty walk-entry (`duty > 0.70`), plus mean duty and frame counts
  - report replay-side mid-stance planar contact world speed (plant slip) overall
    and split by tripod vs overlap; onset world steps are excluded
  - JSON keys (diagnostic, not behaviour gates):
    `captured_onset_reverse_steps`, `captured_midstance_reverse_steps`,
    `captured_midstance_reverse_f_increase_steps` (`Δf/f > 5%`),
    `captured_midstance_reverse_phase_drop_steps` (`Δφ < 0` while still planned),
    `captured_midstance_reverse_other_steps`,
    `captured_mean_duty_factor`, `captured_high_duty_frames`,
    `captured_tripod_frames`, `captured_overlap_frames`,
    `captured_mean_midstance_opposition_speed_mps`,
    `captured_mean_midstance_counter_yaw_rate_radps`, and the same means with
    `_tripod_`, `_overlap_`, and `_high_duty_` infixes;
    `captured_mean_midstance_stride_hz`,
    `captured_mean_midstance_command_scale`,
    `captured_mean_midstance_cadence_scale`,
    `captured_midstance_stroke_clamp_hit_fraction`,
    `captured_midstance_workspace_xy_hit_fraction`,
    `captured_mean_midstance_tripod_cartesian_opposition_speed_mps_plant_hit` /
    `_plant_miss` / `_workspace_xy_hit` / `_workspace_xy_miss` / `_neither_hit`,
    `captured_mean_midstance_cartesian_opposition_speed_mps` (BodyController
    targets, same `-x` conversion as FK) plus `_tripod_` split,
    `captured_mean_midstance_cartesian_counter_yaw_rate_radps` plus `_tripod_`,
    `captured_mean_midstance_ik_opposition_speed_mps` (pre-slew FK of pipeline
    joints) plus `_tripod_` and matching counter-yaw keys,
    `captured_mean_midstance_aligned_fk_opposition_speed_mps` (post-slew FK from
    the same control step as Cartesian) plus `_tripod_` and matching counter-yaw,
    `captured_midstance_ik_reach_hit_fraction`,
    `captured_midstance_slew_hit_fraction`,
    `captured_mean_midstance_post_clamp_distortion_m`,
    `captured_mean_midstance_governed_command_speed_mps` (`|v| * command_scale`);
    `mean_midstance_contact_world_speed_mps`,
    `mean_midstance_tripod_contact_world_speed_mps`,
    `mean_midstance_overlap_contact_world_speed_mps`;
    contact census and commanded vs uncommanded slip (replay-side live
    `foot_contacts` plus capture planned/hold/`L` flags, no fixture recapture):
    `mean_n_raw_contact`, `mean_n_planned`, `mean_n_hold`,
    `mean_n_late_swing_extra` (`contact && !planned && !hold`),
    `mean_n_L_parked_contacted`, `n_raw_contact_histogram` (counts for `n=0..6`),
    `fraction_mixed_parked_stroking`, `fraction_n_contact_ge_5`,
    `clean_tripod_frames` / `clean_tripod_frame_fraction` (the three planned feet
    are the only contacts and none of those contacted stance feet are on plant
    `L`). Slip uses planar `v_body + R v_cmd_body` as `commanded_world` and
    `v_foot_world - commanded_world` as uncommanded slip:
    `mean_contact_commanded_world_speed_mps`,
    `mean_contact_uncommanded_slip_speed_mps`,
    `mean_midstance_commanded_world_speed_mps`,
    `mean_midstance_uncommanded_slip_speed_mps`,
    `mean_clean_tripod_body_speed_mps`,
    `mean_clean_tripod_cartesian_opposition_speed_mps`,
    `mean_clean_tripod_cartesian_counter_yaw_rate_radps`,
    `mean_clean_tripod_commanded_world_speed_mps`,
    `mean_clean_tripod_uncommanded_slip_speed_mps`,
    `mean_clean_tripod_contact_world_speed_mps`. Attitude and friction proxies:
    `mean_abs_body_pitch_rad`, `mean_abs_body_roll_rad`,
    `mean_peak_normal_impulse_ns`, `mean_peak_friction_impulse_ns`,
    `mean_friction_to_normal_impulse_ratio`
  - report p99 and maximum local step round-trip time; this includes loopback
    transport and is therefore a conservative proxy for the 4 ms physics gate
  - break solver p99 time into whole-body dynamics, contact setup, ADMM, and
    integration/writeback; contact setup is further divided into collision/manifold
    generation, constraint assembly, and articulated Delassus preparation
  - emit an exact iteration histogram plus non-healthy failure-reason counts;
    recovered samples preserve the reason that triggered their bounded retry
  - report p50/p90/p99 iterations per motion phase and per active contact count,
    making redundant-contact convergence tails visible without enabling verbose
    simulator logging
  - classify frames above the 20-iteration acceptance target by whether the
    persistent contact-ID set changed on that frame. This separates topology and
    warm-start churn from slow convergence on an unchanged multi-contact system
  - report iteration percentiles by contact-set age buckets (`0`, `1`, `2-3`,
    `4-7`, `8-15`, and `16+` frames) to reveal convergence tails that persist
    beyond the immediate topology-change frame
- Run from the repository root after building both projects:
  - `source scripts/lib/pinocchio_env.sh`
  - CTest: `cd hexapod-server && ctest --preset tests -R 'physics_sim_exact_command_replay' --output-on-failure`
  - Manual: `HEXAPOD_PHYSICS_SIM_EXE=hexapod-physics-sim/build/hexapod-physics-sim hexapod-server/build-tests/test_physics_sim_exact_command_replay --emit-metrics-json`
- Useful diagnostic selectors:
  - `HEXAPOD_EXACT_REPLAY_MOTION_CASE=forward|reverse|strafe|diagonal|turn_in_place`
  - `HEXAPOD_EXACT_REPLAY_STAND_FRAMES`, `HEXAPOD_EXACT_REPLAY_MOTION_FRAMES`,
    and `HEXAPOD_EXACT_REPLAY_TRANSITION_FRAMES`
  - `HEXAPOD_EXACT_REPLAY_BODY_HEIGHT_M` (default `0.14`)
  - `HEXAPOD_EXACT_REPLAY_COMMANDS_OUT=/tmp/hexapod-commands.txt` saves the
    a version-8 fixture containing capture cadence, phase lengths, commanded
    height, initial pose, reference-solver settings, phase annotations, gait
    `φ` / `duty_factor` / stride rate, governor `command_scale` / `cadence_scale`,
    per-leg stroke-clamp / workspace-XY / IK-reach / servo-slew hits, BodyController Cartesian
    targets, pre-slew and post-slew FK feet, post-clamp distortion, and every joint
    position/velocity target at
    round-trip-safe precision. Set
    `HEXAPOD_EXACT_REPLAY_COMMANDS_IN` on later runs to skip command generation
    and replay that exact fixture. Default CTests load
    `hexapod-server/tests/fixtures/hexapod-commands-v16.txt`. Metadata is restored
    automatically; conflicting explicit phase, frame-count, or body-height
    overrides are rejected. Version-1 through version-7 fixtures must be
    regenerated. Captures with
    inhibited or non-WALK motion frames are rejected before they can be saved.
    Fixture reload is the required mode for solver-parameter A/B comparisons
    because a newly generated reference run can produce a different
    feedback-dependent command stream.
  - Command capture uses a high-cap (500 iteration) proximal reference solve by
    default. This keeps the captured controller in WALK and prevents a legacy
    `BODY_COLLAPSE` fault from silently turning the fixture into inhibited stand
    commands. `HEXAPOD_EXACT_REPLAY_CAPTURE_SOLVER_ITERATIONS` changes that
    offline reference cap. `HEXAPOD_EXACT_REPLAY_CAPTURE_LEGACY=1` retains the
    legacy capture path for diagnosis, and
    `HEXAPOD_EXACT_REPLAY_TRACE_CAPTURE=1` prints each phase's final safety state.
  - `HEXAPOD_EXACT_REPLAY_TRACE_SEGMENTS=1` prints each replay segment's
    body-frame displacement and start/end body velocity, which separates a
    phase's own tracking from incoming command-transition momentum.
  - `HEXAPOD_EXACT_REPLAY_LEGACY=1` replays through the legacy solver instead
    of the proximal solver, using the same captured targets
  - `HEXAPOD_DISABLE_STABILITY_HOLDS=1` skips per-leg liftoff/tilt holds and the
    all-stance freeze in `LocomotionStability` (test-only A/B; not a production
    config key)
  - `HEXAPOD_FOOT_ESTIMATOR_BLEND=0..1` overrides `Tuning.FootEstimatorBlend` for
    capture and replay (harness default is already `0`)
  - `HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS`
  - `HEXAPOD_EXACT_REPLAY_PERIOD_US` sets the replay timestep. Behaviour gates
    score commanded metres and yaw with `min(replay_period_us, capture_period_us)`
    so a slower `PERIOD_US` than the fixture capture (for example 8333 µs on a
    5000 µs capture) does not inflate the 70% bar. Faster replay (4167 / 2083 µs)
    still uses the replay dt. JSON reports `replay_period_us`, `capture_period_us`,
    and `command_score_period_us`.
  - `HEXAPOD_EXACT_REPLAY_PROXIMAL_MU` and
    `HEXAPOD_EXACT_REPLAY_CONTACT_REGULARIZATION`

### 4b) `test_physics_sim_tripod_stroke_probe` (diagnostic, not CTest)

- Location: `hexapod-server/tests/test_physics_sim_tripod_stroke_probe.cpp`
- Forced 3-stance / 3-swing stroke with extra-stance off. After stand warmup
  the probe captures measured foot world poses, plants support feet at those
  XY with Z at the 18 mm sphere radius, and ramps the other three feet to a
  reachable world `+Z` (radius + 40 mm). Support feet then integrate
  world-fixed `v_stance` at 0.12 m/s; raised feet stay at the captured world
  XY. No plant-`L` clamp and no gait scheduler. Use this only when
  exact-replay `clean_tripod_frames` is empty, to ask whether the plant can
  collect a legal tripod.
- Run from the repository root after building tests:
  - `source scripts/lib/pinocchio_env.sh`
  - `HEXAPOD_PHYSICS_SIM_EXE=hexapod-physics-sim/build/hexapod-physics-sim hexapod-server/build-tests/test_physics_sim_tripod_stroke_probe`
- Useful overrides: `HEXAPOD_EXACT_REPLAY_PERIOD_US`,
  `HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS`, `HEXAPOD_EXACT_REPLAY_LEGACY=1`,
  `HEXAPOD_TRIPOD_STROKE_FRAMES`, `HEXAPOD_TRIPOD_STROKE_VX_MPS`
- JSON splits raised vs support feet after raise-warmup and during stroke:
  commanded Cartesian world Z, post-IK FK world Z, measured FK world Z,
  clearance vs the 18 mm foot sphere (`measured_z - radius`), contact
  fraction, and IK reach-hit fraction. Prefixes:
  `raise_warmup_raised_`, `raise_warmup_support_`, `stroke_raised_`,
  `stroke_support_`. `named_unload_cause` is `unloaded`, `reach_ik`,
  `shaft_contact_bit`, or `true_plant`. Stroke-frame plant telemetry (from
  `PhysicsSimBridge::latestSolverTelemetry` plus joint tracking):
  `mean_peak_normal_impulse_ns`, `mean_peak_friction_impulse_ns`,
  `mean_friction_to_normal_impulse_ratio`, `mean_servo_torque_utilization`,
  `max_servo_tracking_error_rad` (support legs only), `mean_cone_residual`,
  `mean_max_contact_penetration`. Chassis coupling (probe-side): `mean_body_vx_mps`,
  `mean_support_foot_world_vx_mps`, `expected_com_delta_v_from_friction_mps`
  (upper bound: `n_support × peak_friction_impulse / mass` summed over stroke
  frames), `friction_com_coupling_ratio` (`last body vx / expected Δv`),
  `named_coupling` (`decoupled` if `|ratio| < 0.10` and expected Δv > 0.05 m/s).
  World-axis mapping (proximal): `mean_sum_friction_impulse_world_x_ns` /
  `_z_ns` (signed net), `mean_sum_abs_friction_impulse_world_x_ns` / `_z_ns`,
  `mean_sum_friction_impulse_world_y_ns` (tangent leak into vertical),
  `mean_contact_delta_vx_mps` (free-flyer `M⁻¹ Jᵀ λ` in world X),
  `friction_axis_x_fraction`, `friction_horizontal_capture`,
  `friction_x_cancellation`, `jacobian_com_ratio`, `named_mapping`
  (`tangent_not_world`, `tangent_not_stroke`, `opposing_tangents`,
  `jacobian_decoupled`, `write_or_servo_absorb`, `applied`, or `unknown`).
  Per-leg census (scene/server order): `mean_leg_friction_impulse_world_x_ns`,
  `mean_leg_pinocchio_drift_tx_mps`, `mean_leg_world_slip_tx_mps`,
  `mean_leg_contact_count`, `mean_contacts_per_planted_tibia`,
  `drift_slip_agree_legs` / `drift_slip_disagree_legs`, `named_tangent_census`
  (`jacobian_parity`, `dual_tibia_contact`, `same_j_opposite_lambda`, or
  `unknown`). Contact-point split (last constraint per tibia):
  `mean_leg_tibia_vx_mps`, `mean_leg_spin_vx_mps`, `mean_leg_t0_x`,
  `mean_leg_foot_vx_mps`, `named_slip_split` (`t0_flip`, `spin_offset`,
  `tibia_linear_opposite`, `sphere_vs_contact`, or `unknown`). Signs need
  `|value| > 1e-4` to count. `t0_flip` / `spin_offset` / `sphere_vs_contact`
  name a stand-preserving Pinocchio lever; `tibia_linear_opposite` does not.
  FK vs physics sphere (support `{1,2,5}`): `mean_leg_cmd_foot_vx_mps`,
  `mean_leg_fk_foot_vx_mps`, `mean_leg_foot_x_m`, `mean_leg_foot_pos_vx_mps`,
  `named_fk_physics` (`qv_stale`, `fk_vs_sphere`, `ik_physics_axis`,
  `true_ik_split`, or `unknown`). `qv_stale` names a sphere position-rate
  tangent overwrite; `ik_physics_axis` names one hinge/zero mapping.
  `fk_vs_sphere` / `true_ik_split` do not name a Pinocchio lever. `ik_physics_axis`
  needs command-scale physics position-rate and instantaneous `foot_vx`.
  Stroke axis is sim Z (server X = −sim Z). Fields: `mean_leg_foot_vz_mps`,
  `mean_leg_foot_z_m`, `mean_leg_foot_pos_vz_mps`,
  `mean_leg_expected_sim_vz_mps` (−FK X), `mean_leg_friction_impulse_world_z_ns`,
  `mean_leg_world_slip_ty_mps`, `named_stroke_axis`
  (`sphere_follows_stroke`, `sphere_opposite_stroke`, `sphere_split_stroke`,
  `sphere_tiny_stroke`, `axis_mix`, or `unknown`), `named_stroke_friction`
  (`aligned_z`, `opposing_z`, or `unknown`). FK vs C-mapped sphere
  (`x_srv = -z_sim`, `y_srv = x_sim`): `mean_leg_fk_minus_mapped_x_m`,
  `mean_leg_fk_minus_mapped_y_m`, `stroke_fk_dx_m`, `stroke_mapped_dx_m`,
  `named_fk_sphere_pos` (`match`, `opposite_stroke`, `constant_offset`,
  `diverge`, or `unknown`). `sphere_opposite_stroke` / `opposite_stroke`
  name one stand-preserving coxa-axis trial; a global flip is not the lever
  if support then splits. Frame C (first telemetry frame + stroke Δ):
  `rest_fk_minus_bridge_*`, `rest_fk_minus_align_*`, `stroke_align_dx_m`,
  `named_rest_c` (`rest_bridge_match`, `rest_align_match`, or
  `rest_both_offset`), `named_stroke_c` (`stroke_bridge_follows`,
  `stroke_align_follows`, `stroke_bridge_opposite`, `stroke_align_opposite`,
  `stroke_90`, or `unknown`), `named_frame_c` (`align_c_match`,
  `bridge_c_match`, `align_rest_stroke_90`, `both_opposite`, or `unknown`).
  Rest match needs `|residual| < 0.025 m`. `align_c_match` /
  `align_rest_stroke_90` name one foot_body→bridge-C composition;
  `both_opposite` / `bridge_c_match` do not. Per-joint tracking:
  `max_coxa/femur/tibia_tracking_error_rad`, `named_tracking`
  (`tracking_coxa`, `tracking_femur`, `tracking_tibia`, or `tracking_mixed`).
  Body-frame rest (FK `footInBodyFrame` vs chassis-relative sphere, both Cs):
  `rest_body_minus_bridge_*`, `rest_body_minus_align_*`, `named_body_c`
  (`body_align_match`, `body_bridge_match`, or `body_both_offset`).
  `body_align_match` plus `stroke_90` is treated as `align_rest_stroke_90`.
  Per-leg rest vs hip (`footInBodyFrame` / plant XY vs align-C sphere):
  `rest_leg_class`, `plant_leg_class`, `plant_minus_align_*`,
  `named_rest_offset` (`match`, `translation`, `swap_90`, `left_right`,
  `per_leg_split`, or `unknown`). `left_right` names one side mapping;
  `per_leg_split` / `unknown` do not. Geometry, commands and TwistField+IK share
  the canonical server frame; `legacyKinematicTwistFromServerBody` is identity.
  `HEXAPOD_TRIPOD_STROKE_TORQUE_SCALE` sets child env `HEXAPOD_SERVO_TORQUE_SCALE`
  (serve-only max-torque scale, default 1). `named_h4_cause` is a single-run label:
  `coulomb_skate`, `light_normal`, `servo_cone`, or `unknown`.
  `proximal_mu_mix` is named only after a PGS vs proximal A/B, not by one run.
  PGS does not fill impulse/cone fields; compare body progress and slip instead.

### 5) `test_physics_sim_proximal_stand_acceptance` (proximal acceptance gate)

- Location: `hexapod-server/tests/test_physics_sim_proximal_stand_acceptance.cpp`
- This is a default live-physics CTest. Exact-replay locomotion CTests share the
  same physics-sim child and cap-24 production settings.
- Runs the complete controller, estimator, safety, bridge, collision, and
  Pinocchio contact path in STAND mode for 60 simulated seconds at the 0.14 m
  production body-height command.
- Enforces zero held/unsupported/non-converged states and rollbacks, body-height
  error at or below 10 mm, reconstructed stance-foot drift below 3 mm RMS,
  pre-integration speed limits, and strict ADMM p99 at or below 20 iterations.
- Run from the repository root:
  - `source scripts/lib/pinocchio_env.sh`
  - `hexapod-server/build-tests/test_physics_sim_proximal_stand_acceptance hexapod-physics-sim/build/hexapod-physics-sim`
  - append `--emit-metrics-json` for a machine-readable limits-and-metrics record
- Useful diagnostic overrides:
  - `HEXAPOD_PROXIMAL_STAND_DURATION_S` and `HEXAPOD_PROXIMAL_STAND_WARMUP_S`
  - `HEXAPOD_PROXIMAL_STAND_ITERATIONS`
  - `HEXAPOD_PROXIMAL_STAND_MU`, `HEXAPOD_PROXIMAL_STAND_ABSOLUTE_TOLERANCE`,
    `HEXAPOD_PROXIMAL_STAND_RELATIVE_TOLERANCE`, and
    `HEXAPOD_PROXIMAL_STAND_CONTACT_REGULARIZATION`
  - `HEXAPOD_PROXIMAL_STAND_BODY_HEIGHT_M`
  - `HEXAPOD_EXACT_REPLAY_ABSOLUTE_TOLERANCE` and
    `HEXAPOD_EXACT_REPLAY_RELATIVE_TOLERANCE`
  - `HEXAPOD_EXACT_REPLAY_PERIOD_US=4166` measures the approximately 240 Hz
    production command cadence while preserving the captured targets; proximal mode
    advances it as two internal substeps capped at `1/480 s`. When `PERIOD_US` is
    slower than the fixture `capture_period_us`, commanded metres and yaw for the
    70% behaviour gate use the capture period so extra wall-clock does not inflate
    the bar. Faster cadences keep the replay dt.
  - `HEXAPOD_EXACT_REPLAY_PERTURBATION_SEEDS=100` repeats the same command stream
    from 100 deterministic initial chassis perturbations (by default up to 0.75 mm
    horizontal, 0.375 mm vertical, 0.1875 degrees roll/pitch, and 0.25 degrees yaw).
    `HEXAPOD_EXACT_REPLAY_PERTURBATION_SCALE` scales that envelope; `4` exercises
    up to 3 mm horizontal and 1 degree yaw. Each seed also
    permutes contact-constraint ordering, and every replay runs in a fresh simulator
    process so no state leaks between seeds. Seed zero remains the unperturbed baseline.
    `HEXAPOD_EXACT_REPLAY_PERTURBATION_SEED_OFFSET` selects a later deterministic
    range. For diagnosis, `HEXAPOD_EXACT_REPLAY_FIXED_INITIAL_POSE=1` or
    `HEXAPOD_EXACT_REPLAY_FIXED_CONTACT_ORDER=1` isolates the other perturbation.
    Multi-seed timing summaries report the worst per-seed p99, rather than a pooled
    p99, so a single slow initial condition remains visible.
  - `HEXAPOD_EXACT_REPLAY_CHILD_STDIO=1` preserves simulator diagnostics; combine
    it with `HEXAPOD_PROXIMAL_TRACE_FAILURES=1` to inspect held-state failures
  - advanced solver diagnostics can override `HEXAPOD_PINOCCHIO_ANDERSON_CAPACITY`,
    `HEXAPOD_PINOCCHIO_RETRY_ANDERSON_CAPACITY` (overrides the retry history independently),
    `HEXAPOD_PINOCCHIO_RATIO_PRIMAL_DUAL`, `HEXAPOD_PINOCCHIO_ADMM_TAU`, and
    `HEXAPOD_PINOCCHIO_SPECTRAL_POWER`, while `HEXAPOD_PINOCCHIO_WARMSTART_RHO=0`
    disables spectral-penalty persistence; `HEXAPOD_PINOCCHIO_SERVO_GAIN_SCALE`
    isolates constrained-load calibration without changing the motor torque-speed
    envelope. Production PD already uses stance-loaded reflected inertia
    (`1 / P_ii` from six planted feet at the initial pose, other joints locked,
    clamped to `[M_ii, 1.5 M_ii]`) on every servo. A contact-aware swing/stance
    split of that table recovered tracking but lost closed-loop turn yaw, so
    production keeps the clamped value on swing as well. Isolated
    turn-in-place at cap 500 (2026-09-13): a global scale `1.5` on the older
    unconstrained `M_ii` cut foot-tracking RMS from 9.9 cm to 5.2 cm with zero
    speed-limit recoveries; scale `2.0` improved yaw further (0.21 → 0.35 rad of
    1.08 commanded) but recovered on the 10 rad/s guard. Do not change the
    production default (`1.0`) from those diagnostics; they motivated the
    per-joint stance-loaded `I` replacement, not a shipped global scale.
  - `HEXAPOD_PINOCCHIO_DENSE_ADMM=1` is an A/B performance experiment. The
    articulated rigid-body operator still computes the whole-body Delassus response
    and applies the final generalized impulse, while ADMM uses a materialized dense
    contact-space matrix for its repeated products. The exact replay JSON records
    `dense_admm`; the production path remains the articulated operator by default.
  - `HEXAPOD_PINOCCHIO_CONTACT_PRECONDITION=1` is an A/B conditioning experiment.
    It scales each three-axis contact block by its effective mass, solves the
    equivalent dense NCP, converts impulses and velocities back to physical units,
    and requires the original unscaled physical residual check to pass. Exact replay
    records `contact_precondition`; this path is opt-in and is not a production
    default. Unless explicitly overridden, this normalized path recomputes rho each
    frame and uses a `0.5` spectral-power start; the frozen gait replay showed these
    settings avoid the held-state cascade seen with the articulated defaults.
  - exact-replay phase records include maximum servo tracking error and torque
    utilisation, peak actuator/contact impulses and pre-integration speeds, plus
    accumulated actuator work and mechanical-energy change. These distinguish
    actuator saturation and load-induced tracking loss from contact-solver energy
    injection when a gait makes poor progress or loses body height.
  - recovery is restricted to solver non-convergence and speed-limit rejection.
    Non-convergence retries the same `dt` once before two half-substeps;
    if those still miss, a last-resort pair of half-substeps starts from a
    cold contact warm-start at twice `SolverIterations`. Speed-limit goes
    straight to half-substeps. State validity, penetration,
    energy, and write failures hold the last-good state immediately because they
    cannot be repaired by another contact solve. Servo target jumps do not wipe
    foot-contact warm starts.
  - `HEXAPOD_EXACT_REPLAY_ENFORCE_GATES=1` makes any recovered, held,
    unsupported, or failed-read sample fail the executable. Without it, the
    executable validates capture/replay accounting and emits diagnostic results.
  - `HEXAPOD_EXACT_REPLAY_ENFORCE_SAFETY_GATES=1` permits the documented usable
    `RecoveredRetry` outcome but fails on a held state, unsupported island, or
    failed read. This is the appropriate gate for the 100-seed perturbation campaign.
  - `HEXAPOD_EXACT_REPLAY_ENFORCE_BEHAVIOR_GATES=1` evaluates every perturbation
    seed independently. In accordance with the acceptance requirement, it excludes
    the first 120 ms acceleration transient from each motion phase; the excluded
    frame count is derived from the replay timestep. Translation and yaw must then
    reach at least 70% of their integrated commands, scored at
    `min(replay_period_us, capture_period_us)`, lateral travel must remain below
    10% of path length plus 10 mm, and turn-in-place translation must remain below
    50 mm. Full-phase displacement and phase-boundary velocity remain in the JSON for
    transition diagnosis. Each phase also reports its evaluated-window values and
    `behavior_gate_passed`. A fixture selected with
    `HEXAPOD_EXACT_REPLAY_MOTION_CASE` evaluates that motion alone rather than failing
    because the other motion phases are absent. The top-level JSON reports
    `behavior_gate_failures`; default exact-replay CTests enable this gate.
    100-seed perturbation remains a diagnostic override, not a default CTest.
  - A 10-minute randomized gait soak is post-default, not a switch or CTest gate.
  - `HEXAPOD_EXACT_REPLAY_BODY_HEIGHT_M` defaults to the production 0.14 m body
    height. Lower crouched-height experiments must opt in explicitly.

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
| Straight walking distance/speed                   | Strong           | `test_physics_sim_walk_distance` (including `slow_forward_walk`), `steady_forward_walk` case in `test_locomotion_regression_suite`                            | quantitative path/speed metrics, including a low-speed net-progress gate                                 |
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
