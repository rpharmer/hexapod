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
are `SolverIterations`, `ProximalMu`, `AbsoluteTolerance`,
`RelativeTolerance`, and `ContactRegularization`. The shipped configurations
keep mode `0` until the locomotion acceptance gates pass.

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

Only timestep-sensitive `SolverNotConverged` and `SpeedLimit` failures attempt
the two-half-substep recovery. Invalid or non-finite state, excessive
penetration, non-finite energy, and write failures restore the last-good state
immediately; repeating collision and contact solves at half `dt` cannot repair
those conditions.

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
  diagnostics. Server and simulator binaries must be rebuilt together after any
  protocol change.

## Environment variables

- `MINPHYS_JSON_SCENE_DIAG=1`
  - Runtime scene diagnostic emission in JSON-scene runs (`scene_json.cpp`)
- `MINPHYS_SERVO_JSON_TEST_VERBOSE=1`
  - Verbose output for servo visual preset test (`tests/test_servo_visual_presets_json.cpp`)
- `HEXAPOD_PROXIMAL_TRACE_FAILURES=1`
  - Emit bounded diagnostics for the first held proximal samples and then every
    250th held sample. Intended for test diagnosis, not normal production logs.

## Notes

- CLI parse errors are strict for missing values, invalid enum values, and non-positive integer constraints where required.
- Scene JSON loader is permissive for many optional fields and applies fallback defaults when absent.
