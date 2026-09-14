# `hexapod-physics-sim` Configuration Reference

This is the canonical configuration reference for `hexapod-physics-sim`.

## Source of truth

- CLI parsing and defaults: `hexapod-physics-sim/src/main.cpp`
- Scene JSON schema/accepted keys: `hexapod-physics-sim/src/demo/scene_json.cpp`
- Terrain patch config structure: `hexapod-physics-sim/src/demo/terrain_patch.hpp`
- Serve mode behavior: `hexapod-physics-sim/src/demo/serve_mode.cpp`
- Sim UDP protocol structs: `hexapod-common/include/physics_sim_protocol.hpp`

## CLI flags

From `src/main.cpp`.

## Run mode and outputs

- `--sink dummy|udp` (default: `dummy`)
- `--model default|hexapod`
- `--scene-file PATH`
- `--frames N` (positive int; default `1200`)
- `--realtime`
- `--zero-gravity`
- `--solver-iterations N` (positive int)

## UDP output

- `--udp-host HOST` (default `127.0.0.1`)
- `--udp-port PORT` (`1..65535`, default `9870`)

## Interactive mode

- `--interactive` / `-i`
- `--autonext`
- `--autonext-run`

## Serve mode

- `--serve`
- `--serve-port PORT` (`1..65535`, default `9871`)
- `--serve-preview-stride N` (positive int, default `1`)
- `--resource-monitoring full|top-level|off` (default `full`; accepts `toplevel` and `coarse` as top-level aliases)

The server-side `Runtime.PhysicsSim.*` keys are copied into the binary
`ConfigCommand`. `SolverMode = 0` keeps the legacy PGS path; `SolverMode = 1`
selects Pinocchio 4.1 whole-body proximal contact dynamics. The remaining keys
are `SolverIterations`, `ProximalMu`, `AbsoluteTolerance` (default `1e-8`; this
is the standing ADMM stop),
`RelativeTolerance`, and `ContactRegularization`. Sliding contacts (tangential
free speed above 2 cm/s) use ADMM stop `1e-3` and NCP accept
`max(AbsoluteTolerance, 1e-3)`. Do not raise the standing ADMM stop to `1e-3`:
stand then reports Healthy while the body sags ~10 cm. WSL
`config.physics-sim-wsl.txt` defaults to mode `1` and 24 iterations after the
proximal stand CTest and exact-replay gates. `config.physics-sim.txt` and
`config.physics-sim-test-harness.txt` stay on mode `0` so comparison CTests
remain legacy PGS. `legacy-pgs` (`= 0`) is still a valid explicit setting.

Built-in hexapod serve materials (`BuildHexapodScene`): tibia/foot **2.0 / 2.0**,
ground plane **2.0 / 2.0**. Pinocchio Coulomb μ is the average of the two
**dynamic** coefficients (**2.0**). Legacy PGS uses `max` of the static and
dynamic averages (also **2.0** on this scene). `ProximalMu` is ADMM
regularization, not ground friction. The hexapod scene also sets
`penetrationSlop = 0.002` and `penetrationBiasFactor = 0.20` so velocity-level
contact keeps depth near 2 mm across 120/240/480 Hz.

Proximal mode limits internal integration substeps to `1/480 s`, independently
of the command cadence; legacy PGS retains its `1/240 s` maximum. The spectral
ADMM defaults use a primal/dual ratio of `5`, relaxation `tau = 0.7`, and an
Anderson history of `5`; half-step recovery uses the same history length.

In proximal mode, `RecoveredRetry` samples remain usable but are logged as
warnings. `HeldLastGood` and `UnsupportedIsland` responses are rejected by the
server bridge as invalid sensor reads, so they cannot feed the estimator or gait
controller as fresh motion state. A physics-simulator bus fault can clear while
the controller remains inhibited after 30 distinct consecutive `Healthy`
responses; a repeated, recovered, held, or unsupported response resets that
recovery streak. Other safety faults retain the normal operator-reset policy.

Only timestep-sensitive `SolverNotConverged` and `SpeedLimit` failures retry.
`SolverNotConverged` first repeats the same `dt` from the snapshot (warm-started
from the failed iterate, minus the worst contact), then falls back to two
half-substeps if that still fails. If those half-substeps also miss, a last-resort
pair of half-substeps starts from gravity-based contact guesses at **twice**
`SolverIterations` (at least 48). That path is not used on healthy first attempts.
`SpeedLimit` skips the same-`dt` attempt and goes straight to half-substeps,
because the integrated speed is a function of `dt`. Invalid or non-finite state,
excessive penetration, non-finite energy, and write failures restore the last-good
state immediately; repeating collision and contact solves cannot repair those
conditions. Servo target jumps reset the rate-limited command filter but do
**not** wipe foot-contact warm starts.

Every proximal response also carries the final solver residuals, physical NCP
and friction-cone residuals, peak contact and actuator impulses, servo torque
utilisation, pre-integration speed, peak contact penetration, mechanical energy/work, contact counts,
ADMM/Delassus diagnostics, cumulative warm-start/retry/rollback/held counters,
the persistent ID of the contact with the largest physical feasibility
residual, and a stable signature of the active contact-ID set. For a
multi-substep command, maxima and cumulative counters are
aggregated across the complete command; energy delta and actuator work are
summed.

The pre-integration guard rejects any non-finite contact penetration or a
penetration deeper than `0.05 m`. The complete last-good pose and velocity are
restored before the response is marked `HeldLastGood`; the invalid pose is
never published.

## Help

- `-h` / `--help`

## Scene JSON schema (minphys)

Parser implementation: `src/demo/scene_json.cpp`.

## Top-level keys

- `schema_version` (supported: `<=2`)
- `solver_iterations` (integer-like, clamped/validated by loader; parser path accepts and applies with internal bounds)
- `terrain_patch` (object, optional)
- `bodies` (array, required)
- `joints` (array, optional)

## `bodies[]` common keys

- `shape` (required)
- `static`
- `mass`
- `position`
- `velocity`
- `angular_velocity`
- `orientation`
- `static_friction`
- `dynamic_friction`
- `restitution`
- `linear_damping`
- `angular_damping`
- `collision_group`
- `collision_mask`

## Shape-specific body keys

- `sphere`: `radius`
- `box`: `half_extents`
- `plane`: `plane_normal`, `plane_offset`
- `capsule`: `radius`, `half_height`
- `cylinder`: `radius`, `half_height`
- `half_cylinder` / `halfcylinder`: `radius`, `half_height`
- `compound`: `children`

## `compound.children[]` keys

- `shape` (required)
- `local_position`
- `local_rotation`
- `radius`
- `half_height`
- `half_extents`

## `joints[]` keys

Common:

- `type` (required)
- `body_a` (required)
- `body_b` (required)

Distance joint:

- `anchor_a`, `anchor_b`
- `stiffness`, `damping`

Hinge joint:

- `anchor`
- `axis`
- `enable_limits`
- `lower_angle`, `upper_angle`
- `enable_motor`
- `motor_speed`
- `max_motor_torque`

Ball socket:

- `anchor`

Fixed:

- `anchor`

Prismatic:

- `anchor`
- `axis`
- `enable_limits`
- `lower_translation`, `upper_translation`
- `enable_motor`
- `motor_speed`
- `max_motor_force`

Servo:

- `anchor`
- `axis`
- `target_angle`
- `max_servo_torque`
- `max_servo_speed`
- `position_gain`
- `damping_gain`
- `integral_gain`
- `integral_clamp`
- `position_error_smoothing`
- `max_correction_angle`
- `angle_stabilization`

## `terrain_patch` keys

From parser + `TerrainPatchConfig`.

Grid and base model:

- `rows`
- `cols`
- `cell_size_m`
- `base_margin_m`
- `min_cell_thickness_m`
- `influence_sigma_m`
- `plane_confidence`
- `confidence_half_life_s`
- `base_update_blend`
- `decay_update_boost`

Sampling/collision policy:

- `use_sample_binning`
- `sample_bin_size_m`
- `use_conservative_collision`
- `scroll_world_fixed`

LiDAR fusion:

- `lidar_fusion_enable`
- `lidar_sample_stride`
- `lidar_sample_weight`
- `lidar_min_surface_confidence`
- `lidar_contact_arbitration_radius_m`
- `lidar_contact_disagreement_m`

Seed values:

- `center`
- `plane_normal`
- `plane_height_m` (alias supported: `plane_offset`)

## Serve protocol configurable surfaces

Definitions in `hexapod-common/include/physics_sim_protocol.hpp`.

- `ConfigCommand`: `gravity[3]`, solver mode, iteration limit, proximal
  regularisation, absolute/relative tolerances, and contact regularisation
- `StepCommand`: `sequence_id`, `dt_seconds`, `joint_targets[18]`
- `StateCorrection`: pose/twist/contact/terrain fields + flags and `correction_strength`
- `StateResponse`: body/joint/contact/sensor state followed by proximal solver
  status, failure reason, residual, impulse, energy, speed, contact, and recovery
  diagnostics, including world-mapped friction impulse sums (`*_world_x/y/z`),
  free-flyer contact Δv, and per-leg arrays
  (`solver_leg_friction_impulse_world_x`, `solver_leg_friction_impulse_world_z`,
  `solver_leg_pinocchio_drift_tx`,
  `solver_leg_world_slip_tx`, `solver_leg_world_slip_ty`,
  `solver_leg_contact_count`,
  `solver_leg_tibia_vx`, `solver_leg_spin_vx`, `solver_leg_t0_x`,
  `solver_leg_foot_vx`, `solver_leg_foot_x`, `solver_leg_foot_pos_vx`,
  `solver_leg_foot_vz`, `solver_leg_foot_z`, `solver_leg_foot_pos_vz`).
  Server and simulator binaries must be rebuilt together after any protocol
  change.

## Environment variables

- `MINPHYS_JSON_SCENE_DIAG=1`
  - Runtime scene diagnostic emission in JSON-scene runs (`scene_json.cpp`)
- `MINPHYS_SERVO_JSON_TEST_VERBOSE=1`
  - Verbose output for servo visual preset test (`tests/test_servo_visual_presets_json.cpp`)
- `HEXAPOD_PROXIMAL_TRACE_FAILURES=1`
  - Emit bounded diagnostics for the first held proximal samples and then every
    250th held sample. Intended for test diagnosis, not normal production logs.
- `HEXAPOD_SERVO_TORQUE_SCALE`
  - Serve-only multiplier on built-in hexapod `maxServoTorque` and on the
    Pinocchio stall clip (`kServoMaxTorqueNm`). Applied after
    `RelaxBuiltInHexapodServos`. Default 1. Used by the tripod stroke probe via
    `HEXAPOD_TRIPOD_STROKE_TORQUE_SCALE`; not a production config key.

## Notes

- CLI parse errors are strict for missing values, invalid enum values, and non-positive integer constraints where required.
- Scene JSON loader is permissive for many optional fields and applies fallback defaults when absent.
