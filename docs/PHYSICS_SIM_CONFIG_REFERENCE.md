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

The state-response actuator tail (revision 1) reports 18 nominal position-loop
stiffnesses in Nm/rad, protocol joint order. They include articulated/contact
load scaling, but not temporary retry gain reductions. Values describe the
last accepted substep; they are not contact-state forecasts. Legacy PGS reports
all zeros (unavailable). Malformed gains/unknown tail revisions are rejected.
Rebuild server and simulator together after this protocol extension; old state
packets are too short. No new solver mode, torque feedforward channel or
automatic recovery equation is introduced.

- `--serve`
- `--serve-port PORT` (`1..65535`, default `9871`)
- `--serve-preview-stride N` (positive int, default `1`)
- `--resource-monitoring full|top-level|off` (default `full`; accepts `toplevel` and `coarse` as top-level aliases)

The server-side `Runtime.PhysicsSim.*` keys are copied into the binary
`ConfigCommand`. Parser bounds for `SolverMode` are `0..2` with default `1`.
`SolverMode = 0` keeps the legacy PGS path; `SolverMode = 1`
selects Pinocchio 4.1 whole-body proximal contact dynamics (WSL/production
default). `SolverMode = 2` selects the same proximal plant with the explicit
compliant-contact equations (projected-gradient, live 1e-3 residual, 1.0 N·s
impulse cap). Mode 2 is opt-in as a session setting. Healthy Mode 1 steps never
switch to that law after a rigid reject. After rigid last-resort NCP still
misses, a logged cone-QP recovery may apply the same law on two cold `dt/2`
half-steps if residual, impulse, and speed guards pass (`RecoveredRetry`, not
`HeldLastGood`). `HEXAPOD_PINOCCHIO_DISABLE_NCP_CCP_RECOVERY=1` turns that
recovery off. `HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT` remains a local
override that enables the same equations for the whole session without changing
config. The remaining keys
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
If that last-resort pair still misses the Signorini NCP floor (`SolverNotConverged`
only, not `SpeedLimit`), two further cold `dt/2` half-steps run the cone QP
(same law as Mode 2). A passing write is `RecoveredRetry` with a real integrated
state; `[proximal-ncp-ccp-recovery]` is always logged. Impulse, projected-residual,
and 10 rad/s / 2 m/s guards still apply.
`SpeedLimit` also retries the same `dt`, then half-substeps. For the production
explicit motor, retries reduce the proportional position request to the existing
0.5 scale while preserving nominal damping. Halving damping too can remove
needed braking. Healthy first attempts, torque-speed limits, NCP acceptance and
all-body speed guards are unchanged. `HEXAPOD_PINOCCHIO_RETRY_KEEP_DAMPING=0`
restores the old retry law for diagnostic comparison; it is not the default.
The separate implicit-damping experiment retains its existing coupled law.
Invalid or non-finite state,
excessive penetration, non-finite energy, and write failures restore the last-good
state immediately; repeating collision and contact solves cannot repair those
conditions. Servo target jumps reset the rate-limited command filter but do
**not** wipe foot-contact warm starts.

The proximal servo controller calibrates nominal 25 rad/s PD stiffness from the
six-foot reflected inertia. For a sustained static three-leg pose it
uses a reduced-support correction: a six-foot command reference must first be
latched, exactly three leg command groups must then be retargeted, targets must
remain unchanged for 250 ms, and fewer than six tibias must be in contact. The
gain ramps to 1.85× over 100 ms for the complete pose. Moving commands and
six-foot stand retain nominal gain; stall torque and the torque-speed curve are
unchanged. `HEXAPOD_PINOCCHIO_STATIC_REDUCED_SUPPORT_GAIN_SCALE=1..2` is a
test-only A/B override.

Both rigid and opt-in compliant proximal paths retain the all-body WORLD_ALIGNED
2 m/s / 10 rad/s guard at the old pose and additionally check the proposed
integrated pose with the resulting generalized velocity before writing any body.
Unsafe candidates use the existing SpeedLimit retry/rollback policy. There is no
velocity clamp of `vNew` after ABA. Scaled-impulse recovery after a speed-guard
trip is not production: a 2026-09-16 λ-scale leftover regressed isolated reverse
from 5/5 to 4/5 femur SpeedLimit and was reverted. Unsafe/non-finite external
corrections restore
the previous validated rollback state and cannot replace it. Legacy-PGS correction
handling is unchanged. The existing `preintegration_*` diagnostic field names are
retained for protocol compatibility; their peaks now include both old-pose and
proposed-pose validation. Rejected/recovered attempts can contribute to diagnostic
peaks, so a peak is not the velocity of the final accepted sample.
`HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT=1` additionally emits
`[proximal-integrated-speed-limit]` for a proposed-pose rejection after the original
guard passed. Safety limits, protocol size/revision and WSL solver default are unchanged.

Every proximal response also carries the final solver residuals, physical NCP
and friction-cone residuals, peak contact and actuator impulses, servo torque
utilisation, pre-integration speed, peak contact penetration, mechanical energy/work, contact counts,
ADMM/Delassus diagnostics, cumulative warm-start/retry/rollback/held counters,
the persistent ID of the contact with the largest physical feasibility
residual, a stable signature of the active contact-ID set, and
`solver_compliant_projected_residual` (populated on `SolverMode = 2`,
`HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT`, or a Mode 1 last-resort
cone-QP recovery; otherwise 0).
For a
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
- `StepCommand`: `sequence_id`, `dt_seconds`, `joint_targets[18]`, optional
  `joint_target_velocities[18]` (legacy packets omit the trailer; serve then
  reconstructs Δq/Δt). Live server and exact-replay send the commanded rates
  after servo slew. Do not treat them as a velocity constraint.
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
    Leftover §3.16: 3-leg gravity at 0.14 m is 0.43 N·m (30% stall). Do not
    raise this scale as a walking fix. Host table
    `tools/stance_torque_authority.py`.
- `HEXAPOD_PINOCCHIO_DISABLE_NCP_CCP_RECOVERY=1`
  - Skip last-resort cone-QP recovery after a rigid NCP miss. Default is on so
    a 5-contact NCP miss can still write a residual/impulse/speed-guard-passing
    integrated state (`RecoveredRetry`) instead of `HeldLastGood`.
- `HEXAPOD_PINOCCHIO_DISABLE_CONTACT_INERTIA=1`
  - Keep spawn six-foot servo inertias instead of ramping the load-bearing
    contact CRBA. Exact-replay sets this so fixture v16 stays on the capture
    plant. Live closed-loop tests leave it unset.
- `HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH=<file>`
  - One-shot schema-2 dump of the first full-PD SpeedLimit (q/v, mass, Jacobians,
    incoming/free/post-contact rates). Includes `implicit_damping` (whether
    `H = M + h D` produced `v_free`). Also embeds `accepted_history` when the
    pre-failure ring is populated. Diagnostic only; does not change the solver.
    Existing frozen snapshots are never recaptured. New hunts
    `docs/contact-snapshots/sl-abort-stand-untilt-v1.json` sha256
    `96683c07…118c644f` (2.98 rad tibia ABA-over after STAND untilt) and
    `docs/contact-snapshots/sl-abort-after-1p5-v1.json` sha256
    `24bf7261…bdbeebab` (0.71 rad femur ABA-over after the 1.5 rad cap) and
    `docs/contact-snapshots/sl-abort-near-cap-v1.json` sha256
    `11eeaef2…5e48e3dd` (0.63 rad tibia ABA-over, requested 1.89; leftover
    §3.12 named winner wire 17; tibia-only remainder reverted; do not
    recapture `p3-seq-first-trip-*`, `sl-abort-stand-untilt-*`,
    `sl-abort-after-1p5-*`, or leftover §3.14
    `docs/contact-snapshots/sl-abort-default-straight-v1.json` sha256
    `57e243fe…aca63584`, implicit-off sequential reverse femur swing).
    Skip-existing: if the path already exists, log
    `[proximal-speed-limit-snapshot] skip existing` and do not recapture.
- `HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH=<file>`
  - One-shot dump of the preceding accepted-state ring (up to 8 samples: q/v,
    chassis ω, τ/targets/errors, dt, contact IDs/normals/impulses, effective
    inertias, rigid vs `ncpCcpRecovery`) on the first full-PD SpeedLimit or
    first NCP/`HeldLastGood`. New hunt paths only. Skip-existing.
- `HEXAPOD_SUPPORT_DIVERGENCE_DUMP_PATH=<file>`
  - Test-only never-overwrite trailing WALK ring (cap 256) from
    `test_physics_sim_walk_distance`. Per tick, all six legs: commanded /
    measured foot world vs terrain Z, surface closing `measured_dz_dt`, gait
    `in_stance` / `phase` / `duty_factor`, raw manifold `raw_contact`, fused
    phase / `fused_load_bearing`, contact-anchor valid/drift, 18-wire target /
    measured / error, and `leg_contact_count`. Writes on the first held or
    non-WALK case. Optional `HEXAPOD_SUPPORT_DIVERGENCE_DUMP_CASE` restricts
    the label. Log `[support-divergence-dump]`. Host classifier
    `tools/classify_support_divergence.py`. Do not recapture frozen first-trip
    files. Intended fixture
    `docs/contact-snapshots/support-divergence-default-straight-v1.json`.
  - Also records the body-frame commanded foot at three pipeline stages —
    `planned_body` (planner Cartesian output), `pre_slew_body` (FK of the IK
    solution) and `post_clamp_body` (after the per-joint slew clamp) — plus
    per-tick `body_v`. This attributes an infeasible foot-speed demand to a
    stage. Frozen attribution fixture
    `docs/contact-snapshots/plan-stage-attrib-v1.json` sha256
    `4c0f8875…7fa93a51`: planner swing demand p99 8.70 m/s / max 18.35 m/s,
    identical after IK, cut to 1.71 / 2.25 by the slew clamp (leftover §3.15).
- `HEXAPOD_SWING_PLAN_COMMIT=1`
  - Opt-in screen, **default off and rejected** (leftover §3.15). Commits one
    swing plan per swing (`resolveSwingPlan` once, then `evalSwingPlan` on phase)
    instead of re-resolving every control sample, and bounds the emitted
    Cartesian foot step to `max(1.5 · |planner velocity|, 0.5 · ω_noload · r) · dt`.
    Removes the demand entirely — swing commanded foot speed p99 8.70 → 0.735 m/s
    with no sample stepping more than 3.7 mm — and still regressed isolated
    reverse 4/5 → 1/5 and sequential 3/5 → 0/5 with drag unchanged. Kept as
    characterization; do not enable in production.
- `HEXAPOD_HEIGHT_HOLD_SCALE=<0..1]`
  - Screen scale on the body-height hold (proportional plus integral), **default
    1.0 = production** (leftover §3.18). At 0.0 the hold is off and isolated
    reverse improves drag 64.4 → 59.6%, femur stance error 0.125 → 0.107 rad and
    liftoff delay 138 → 125 at 5/5 with zero holds, costing distance
    0.696 → 0.650 m. Heave is **unchanged** (28.2 vs 25.7 mm), which falsifies
    the hold as the heave driver. 0.5 is worse than both endpoints (3/5,
    `held=950`). Not screened on sequential/turn/`aggressive_governor`; do not
    enable without the full remesure.
- `HEXAPOD_TURN_INPLACE_HOLD=1`
  - **Default on only with absolute body-position feedback capability**, supplied
    by PhysicsSimBridge, not serial hardware or the simple simulator. Set `0`
    to disable for comparison. An override cannot bypass that capability.
    The first demonstrated lever for the turn-net
    leftover (§4.2.1). A yaw-dominant turn commands zero planar velocity, so
    body translation — the quantity the 0.21 m gate measures — is open loop.
    Latches body XY at turn entry and adds a bounded body-frame planar command
    opposing measured drift (gain 0.20 s⁻¹, bound 0.03 m/s), only while
    a nonzero turn with zero commanded planar speed is requested in WALK.
    Translating arcs are excluded. Invalid feedback, mode exit and recovery
    reset the origin. Gait timing, duty,
    stroke and phase are untouched, and it is inert on straight walking.
    Isolated turn net **0.184 → 0.075 m while yaw rises 2.152 → 2.351 rad**;
    sequential turn-net failures **3/8 → 0/8** with all scored turns
    0.047-0.092 m; isolated reverse 5/5 and `aggressive_governor` 3/3 unchanged.
    Hardware remains excluded: a drifting fused pose must not be chased instead
    of real translation. See sequential leftovers §3.24 for the current combined
    retry/turn screen; the numbers above describe the earlier isolated lever.
- `HEXAPOD_PINOCCHIO_TRANSPORT_WARM_START=1`
  - Diagnostic-only, default off. Transports cached contact-coordinate impulse
    and velocity into the new contact frame before existing timestep scaling
    and cone projection; normal reversal clears the seed. Coordinate-invariance
    tests pass, but a moving-lift equilibrium regression prevents promotion.
- `HEXAPOD_WALK_SLEW_FRACTION=<0..1]`
  - Opt-in command-side screen, **default off** (unset = 1.0 = previous 100% of
    no-load reference; leftover §3.17). Scales the WALK-only per-joint slew
    envelope in `clampJointTargetsToServoDynamics` so the reference reserves
    `stall · (1 − fraction)` of torque for tracking. STAND keeps the full
    envelope. Sequential scored pass is **worse** (baseline 7/15 processes →
    2/10 at 0.6) and turn net degrades (max 0.234 → 0.320 m); response is
    non-monotonic in the fraction, and `snapSwingTargetsNearMeasuredLinkCap` can
    still exceed the fraction in one sample. An earlier note claimed holds fell
    1735 → 4; that was summed across processes, which is heavy-tailed (baseline
    is `held==0` in 12/15 processes) and is withdrawn. Characterization only.
- `HEXAPOD_WALK_LOAD_PHASE=1`
  - Opt-in command-side screen, **default off** (leftover §3.17). Advances the
    gait stride integrator at 0.25 while any leg past a quarter of its swing is
    still in fused contact, budgeted to one swing duration. Reduces isolated
    reverse drag 65.1% → 58.0% at the cost of distance (0.708 → 0.574 m),
    liftoff delay (139 → 206) and heave; the single global phase integrator also
    slows propulsive stance legs. Combined with the slew cap it makes isolated
    turn net worse (max 0.195 → 0.208 m) and sequential 0/5 (`held=4187`). Its
    drag ratio is confounded because the hold changes the planned-swing sample
    count. Characterization only; do not enable.
- `HEXAPOD_WALK_TEST_GRAVITY_FF=1`
  - Opt-in walk-distance / locomotion-regression screen, **default off and
    rejected** (leftover §3.16). Copies the existing Bounded gravity-FF
    numbers already used by the locomotion-regression long-walk case onto the
    walk-distance plant and `aggressive_governor`: femur/tibia scale 0.30,
    coxa 0.0, stiffness 0.62, LPF 0.08 s, foot reaction on, self-weight off.
    Isolated reverse 4/5 → 2/5, sequential 4/5 → 2/5; femur sag 0.125 →
    0.128 rad, heave 34 → 49 mm. Kept as characterization; do not enable.
    Helpers: `tools/run_gravity_ff_screen.sh`,
    `tools/run_gravity_ff_batch.sh`, `tools/summarize_gravity_ff_ab.py`.
- `HEXAPOD_LOADED_SWING_HOLD=1`
  - Opt-in screen, **default off and rejected** (leftover §3.15). Bounds the
    commanded joint error of a planned-swing leg that is still in raw contact to
    `kLoadedSwingTrackingErrorRad` (0.362 rad, derived as
    `guard / (kJointsPerLeg · 0.3679 · kServoOmegaN)`). Never touches planned
    stance, so it cannot withdraw support. Screened A/B on identical binaries:
    sequential 3/5 → 0/5, isolated reverse 5/5 → 4/5, drag 54.7% → 57.6%. The
    stored error is what frees a dragging leg, so bounding it extends the drag.
    Kept as characterization only; do not enable in production.
  - `HEXAPOD_LOADED_SWING_HOLD_ERROR_RAD=0.25|0.5|0.75` selects a bounded
    screen alternative to the derived default.
  - `HEXAPOD_LOADED_SWING_HOLD_TRACE=1` logs
    `[loaded-swing-hold] step=… leg=… joint=… scale=… max_error=…` per firing leg.
- `HEXAPOD_NEAR_CAP_SNAP_TRACE=1`
  - Env-gated log of production `snapSwingTargetsNearMeasuredLinkCap`:
    `[near-cap-snap] step=… leg=… peak=… in_stance=… status=` when the snap
    fires (`fired`) or is skipped because planned stance is already over the
    production near-cap (`skipped_stance`). Also logs swing `unchanged` /
    `unavailable`. Diagnostic only; no hysteresis and no threshold change.
- `HEXAPOD_PINOCCHIO_STAND_CUTPOINT_PATH=<file>`
  - One-shot dump of last-good q/v, warm starts, load-bearing mask, inertias and
    reduced-support blend at the first post-stance command change after six-foot
    stand latch. New dumps include the warm-start contact `frame` 3×3; frozen
    stand-end files omit it. Test-only restore is
    `PinocchioHexapodModel::restoreDiagnosticCutpoint` (missing `frame` loads as
    `have_frame=false`). Existing frozen cutpoints are never recaptured.
    Sequential turn-entry never retriggers this one-shot (`standCutpointCaptured`).
- `HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_REQUEST=<trigger-file>` /
  `HEXAPOD_PINOCCHIO_CUTPOINT_DUMP_PATH=<file>`
  - Test-only file IPC polled at the start of `stepProximal`. If the request
    file exists, dump schema-1 `stand_cutpoint` to `DUMP_PATH` unless that
    path already exists, then unlink the request. Log
    `[proximal-cutpoint-dump]`. Walk-distance `checkTurnCase` arms the
    request at stand-end. New P5 fixtures
    `docs/contact-snapshots/p5-turn-entry-cutpoint-isolated-v1.json` sha256
    `d4cf9a80…132d5c1` and
    `docs/contact-snapshots/p5-turn-entry-cutpoint-sequential-v1.json` sha256
    `ee3f1e06…2f26774`. Host `test_p5_turn_entry_identity` restore+redump
    (skip `missing_fixture`). Do not recapture frozen `p0-*` cutpoints.
- `HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_REQUEST=<trigger-file>` /
  `HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_PATH=<json>`
  - Test-only file IPC: if the request file exists, restore from
    `RESTORE_PATH` (or the request path when `RESTORE_PATH` is unset), then
    unlink the trigger only when `RESTORE_PATH` is set so fixture JSON is
    never unlinked. Optional `HEXAPOD_PINOCCHIO_CUTPOINT_RESTORE_CLEAR_WARMS=1`
    calls `resetWarmStarts` after restore. Log `[proximal-cutpoint-restore]`.
    Walk-distance rebases `start_xy` after the first WALK step when restore
    is armed. Legal pose patches (`tools/p5_patch_turn_entry_cutpoint.py`):
    `xy-only` / `yaw-only` isolated plant at sequential XZ (and heading).
    Illegal: isolated world `q` teleport, isolated joint `q` on the full
    sequential free-flyer, or isolated warm-start IDs on sequential contacts.
- `HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH=<file>`
  - Test-only one-shot capture of accepted proximal substeps `{dt, targets[18],
    q, v}` after the stand-end cutpoint fires, until first `HeldLastGood` /
    SpeedLimit hold or 720 samples. Never overwrites an existing file. Log
    `[proximal-command-stream]`. Replay is restore cutpoint, set the 18 World
    `targetAngle` values, `stepProximal(dt)` — no UDP/wall-clock. Do not point
    this env at frozen cutpoint or first-trip files.
- `HEXAPOD_SWING_PLANNER_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of R2 walking `planSwingFoot` inputs
    (`SwingFootInputs`, kinematic/est twists, `planned_pre_rot`,
    `target_clamped` after body rotation + reach clamp). Stance-recovery
    `planSwingFoot` is not recorded. Cap 256 samples. Log
    `[swing-planner-dump]`. Replay is host `test_p0_swing_planner_replay`.
    Do not point this env at frozen cutpoint, command-stream, or first-trip
    files. New fixture `docs/contact-snapshots/p0-r2-swing-planner-v1.json`
    sha256 `f1550c2a…cdc669412` (diagnostic `forward_walk`, implicit **on**).
- `HEXAPOD_R2_COMMANDED_FOOT_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of R2 WALK commanded feet: BodyController
    `planned`, IK `pre_slew_fk`, post-slew `post_slew_fk`, `source`, hold /
    slew / IK-reach flags, and `nominal`. Trailing ring (cap 256) logs 40 mm
    sag-nominal `departed` without starting `post_cap`. Deep latch is
    `decomp_valid` walking swing with planned Y ≤ 0.15; then keep samples
    until `bus_ok` false, FAULT, 1024 post-event samples, or 2048 WALK
    ticks (`walk_seen`; `no_deep_tuck` if that count is reached with no
    deep latch, even while the unlatched ring stays at 256). Schema 3 adds
    `decomp_valid` plus untilted `planned_pre_rot` / `after_terrain`,
    `origin_rot` vs `coxa_rot`, foothold split, `clamp_dxy`, terrain XY,
    pose roll/pitch/yaw, `deep_tuck_y`, and `walk_seen`. Log `[r2-commanded-foot-dump]`
    (`departed`, `deep-latched`, `freeze`). Replay is host
    `test_p0_commanded_foot_replay` and `test_p0_swing_decomp_replay`.
    Do not point this env at frozen cutpoint, command-stream, first-trip,
    or swing-planner files. Trailing-ring fixture
    `docs/contact-snapshots/p0-r2-commanded-foot-v1.json` sha256
    `ffaa4142…9f23dda2`. Departure fixture
    `docs/contact-snapshots/p0-r2-nominal-departure-v1.json` sha256
    `2e083b4b…455b7344`. Through-hold fixture
    `docs/contact-snapshots/p0-r2-departure-through-hold-v1.json` sha256
    `089bedae…da4f8bee` (`planned_tucks_to_vin`). Decomp v1
    `docs/contact-snapshots/p0-r2-swing-decomp-v1.json` sha256
    `799d0566…ef62905` (healthy Y 0.18, `mixed`). Decomp v2
    `docs/contact-snapshots/p0-r2-swing-decomp-v2.json` sha256
    `782740df…53a0e2` (vin-tuck `untilted`). Decomp v3
    `docs/contact-snapshots/p0-r2-swing-decomp-v3.json` sha256
    `1ab8d8a6…ff454b5a` (`not_vin_tuck` / `floor_working`).
- `HEXAPOD_TIP_OVER_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of the first control tick with
    `active_fault==TIP_OVER`. JSON `schema_version` 1 `kind=tip_over_dump`:
    loop, requested mode, `bus_ok`, torque cut, roll/pitch/yaw, tilt mag,
    height, IMU gyro x/y and planar hypot, support / confirmed support,
    measured planar speed, yaw cmd, `max_tilt_rad` /
    `rapid_body_rate_radps` / `rapid_body_rate_max_contacts` from config,
    and `rule=angle|rate|both|unknown`. Log `[tip-over-dump]`. Replay is
    host `test_p0_tip_over_replay` (skip `missing_fixture`). Do not point
    this env at frozen cutpoint, command-stream, first-trip, or swing-decomp
    files. Intended fixture `docs/contact-snapshots/p0-seq-tip-over-forward-v1.json`
    was not written: isolated `forward_walk` ×5 and sequential ×5 with this
    env set did not trip TIP_OVER. Walk-distance harness keeps `MaxTiltRad`
    0.60 and `RapidBodyRateRadps` 2.50; do not loosen those or regression
    0.25 / 0.45 / path 0.10.
- `HEXAPOD_TURN_TRAJ_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of a scored `turn_in_place` trajectory from
    `test_physics_sim_walk_distance` (`checkTurnCase`). JSON `schema_version` 1
    `kind=turn_traj_dump`: net, yaw_delta, chord `r_equivalent`, path_per_rad,
    `held`, `cmd_yaw`, start/end xy, and per-tick `{x,y,z,yaw,vx,vy,wz,support,
    fused_count,raw_count,planned_count,left_fused,right_fused,fused_support,
    raw_contact,planned_stance,leg_contact_count}`. New dumps add height and
    per-leg support for sequential pass/fail asymmetry; do not recapture
    `p0-turn-traj-*`.     Host `tools/classify_turn_support_asymmetry.py` labels
    `support_asymmetric` / `orbit` / `unknown`. Log `[turn-traj-dump]`.
    `HEXAPOD_TURN_TRAJ_DUMP_MIN_NET_M` / `HEXAPOD_TURN_TRAJ_DUMP_MAX_NET_M`
    select a scored fail or pass donor by net window, as for the entry dump. Replay is host `test_p0_turn_traj_replay`
    (`orbit|translation|entry|late|unknown`; skip `missing_fixture`). Do not
    point this env at frozen turn-census, command-stream, or first-trip files.
    Isolated fixture `docs/contact-snapshots/p0-turn-traj-isolated-v1.json`
    sha256 `9329a791…6618b3d` (`orbit`, net 0.186 m, fit R 0.099 m). Sequential
    fixture `docs/contact-snapshots/p0-turn-traj-sequential-v1.json` sha256
    `eb90d576…5d9f46` (`orbit`, net 0.216 m, fit R 0.118 m). Do not loosen the
    0.21 m turn-net gate.
- `HEXAPOD_TURN_ENTRY_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of stand-end and first WALK pose/feet/support
    from `test_physics_sim_walk_distance` (`checkTurnCase`). JSON
    `schema_version` 1 `kind=turn_entry_dump`: `net_horizontal_distance_m` (new
    dumps), `held`, `cmd_yaw`, start xy, and `stand_end` / `first_walk` objects
    with pose, body vel, fused support, stance width, support centroid,
    body-frame centroid offset, and per-foot world/body XY. Log
    `[turn-entry-dump]`. `HEXAPOD_TURN_ENTRY_DUMP_MIN_NET_M` skips unless scored
    net ≥ the value; `HEXAPOD_TURN_ENTRY_DUMP_MAX_NET_M` skips unless net ≤ the
    value. The two windows select a scored fail or a scored pass donor from the
    same plant (leftover §4.2 hold-free census). Replay is host `test_p0_turn_entry_replay`
    (`entry_pose|entry_stance|entry_match|unknown`; fail vs pass
    `more_extreme|similar|milder`; skip `missing_fixture`). Do not point this
    env at frozen turn-census, turn-traj, command-stream, first-trip, or
    existing turn-entry files. Isolated fixture
    `docs/contact-snapshots/p0-turn-entry-isolated-v1.json` sha256
    `4dfd9329…79c10149` (6-support, untilted). Sequential fixture
    `docs/contact-snapshots/p0-turn-entry-sequential-v1.json` sha256
    `32506a54…4bb1c923` (3-support, tilt 0.078 rad; scored turn passed 0.200 m).
    Host **`entry_stance`**. Intended fail fixture
    `docs/contact-snapshots/p0-turn-entry-sequential-fail-v1.json` was not
    written (sequential ×5: 3 scored passes under 0.21, 2 abort-before-turn
    SpeedLimit). Do not loosen the 0.21 m turn-net gate.
- `HEXAPOD_TURN_ENTRY_CONTROLLER_DUMP_PATH=<file>`
  - Test-only never-overwrite dump of gait phase/offsets/duty, fused support,
    contact anchors, and governor snapshot from walk-distance `checkTurnCase`
    stand-end (`kind=turn_entry_controller`). Log `[turn-entry-controller-dump]`.
    Isolated fixture
    `docs/contact-snapshots/p5-turn-entry-controller-isolated-v1.json` sha256
    `c5768f8f…f780adc`. Sequential fixture
    `docs/contact-snapshots/p5-turn-entry-controller-sequential-v1.json` sha256
    `8771ed6a…ea405e1`. Sequential traj captured on the same run:
    `docs/contact-snapshots/p5-turn-traj-sequential-v1.json` sha256
    `6b385b53…b49766d` (net 0.223 m, fit R 0.128 m).
- `HEXAPOD_TURN_ENTRY_CONTROLLER_RESTORE_PATH=<file>`
  - Test-only `RobotRuntime::debugRestoreTurnEntryController` of a dumped
    controller snapshot at `checkTurnCase` stand-end (not a production STAND
    reset). Log `[turn-entry-controller-restore]`.
- Test-only `PinocchioHexapodModel::restoreAcceptedHistorySample` loads one
  prefailure `accepted_history` sample (q/v, targets, effective inertias, mask,
  blend, contacts). Warm-start velocity is not in the fixture (zero); the
  contact frame is reconstructed from the dumped normal. Host
  `test_pinocchio_p0_p1` may copy that 18-vector and patch `targets[8]` locally
  (`zero_error` / `rate_limit_10`) before `applyServoTargets`; that mutation is
  not an env or production command path.
- `HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING=1`
  - Opt-in bounded implicit actuator damping: `H = M + h D` with existing PD
    `Kd = 2 ζ ωn I_eff` on the 18 servo tangents, stall/speed envelope clips,
    and contact `G_H = J H⁻¹ Jᵀ` plus `v ← v_free + H⁻¹ Jᵀ λ`. Default **off**;
    healthy Mode 1 stays explicit ABA + `J M⁻¹ Jᵀ`. Does not raise armature,
    stall, or the 10 rad/s guard. Logs `[proximal-implicit-damping]` while
    active. Host algebra: `test_pinocchio_p3_implicit_damping`. Leftover
    screens that enable this env must say **leftover-on**; they do not
    establish WSL-default (unset / implicit-off) behaviour. Envelope and
    actuator-work checks at `v_free` do not by themselves prove final-contact
    torque compliance after `v ← v_free + H⁻¹ Jᵀ λ`.

## Notes

- CLI parse errors are strict for missing values, invalid enum values, and non-positive integer constraints where required.
- Scene JSON loader is permissive for many optional fields and applies fallback defaults when absent.
