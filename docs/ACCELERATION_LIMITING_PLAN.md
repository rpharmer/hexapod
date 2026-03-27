# Hexapod Server Plan: Intelligent Acceleration Limiting and Body/Leg Start-Stop Coordination

## Goal

Implement acceleration limiting across:

- body linear motion,
- body angular motion,
- foot target trajectories,
- implied joint motion,

while avoiding crude hard-clamp behavior by preferring dynamic gait-parameter adaptation (cadence, step length, swing height, duty) and explicit motion-phase coordination.

---

## Investigation Summary (Current Code Path)

### 1) Existing rate limits and where they live

- **Cadence slew-limiting already exists** in `GaitScheduler` via `applyCadenceSlew`, with asymmetric up/down limits (`150 Hz/s` up, `200 Hz/s` down). This constrains stride phase-rate changes but not body command acceleration directly. 
- **Body yaw command slew-limiting exists** in `BodyController` (`kYawCommandSlewLimitRadPerSec = 1.2`) and low-passes `orientation_rad.z`.
- **Per-foot positional slew-limiting exists** in `BodyController` with a normal limit (`0.9 m/s`) and a conservative transition limit (`0.35 m/s`) during walk/non-walk mode transitions.
- **Servo overspeed mitigation exists** in `GaitPolicyPlanner::applyServoVelocityConstraint`, but it is reactive and based on observed peak servo velocity. It scales cadence/step/swing and can downshift gait family; it is not a predictive acceleration budget.

### 2) Current behavior relevant to “body may move before legs move”

- `GaitScheduler` sets `stride_phase_rate_hz=0` and all legs in stance when not walking/stable/safe.
- `BodyController` still applies body translation/rotation setpoints before foothold planning, even when `walking == false`.
- `FootholdPlanner` returns nominal footholds immediately when `walking == false`.

**Implication:** body-pose commands can begin affecting foot targets while gait stepping is halted; this is close to the desired “body starts moving first” behavior, but currently it is incidental and not coordinated by explicit startup/shutdown phases or acceleration envelopes.

### 3) Gaps against requested functionality

- No unified acceleration budget that spans **body linear + body angular + feet + joints**.
- No predictive conversion from requested motion to expected joint acceleration/velocity before commanding gait.
- No explicit startup/stop sequencing policy (body-leads-on-start, legs-lead-on-stop) with deterministic state transitions.
- Existing limits are mainly hard clamps/slews in isolated modules, not a coordinated “intelligent adaptation” layer.

---

## Proposed Design

## A) Introduce a centralized `MotionLimiter` stage in the control pipeline

Add a new control-pipeline component, executed after dynamic gait policy selection and before gait/body execution:

1. Inputs:
   - `MotionIntent` (operator intent),
   - `RuntimeGaitPolicy` (planner output),
   - estimated state (including body pose and joint velocities),
   - loop `dt`.
2. Outputs:
   - **limited/reshaped** `MotionIntent`,
   - **adapted** `RuntimeGaitPolicy` (cadence/step/swing/duty/family),
   - limiter diagnostics (active constraints, scale factors, phase state).

This keeps acceleration logic coherent and testable, and avoids scattering additional clamps.

## B) Use hierarchical acceleration envelopes (intelligent adaptation first, clamps second)

### Priority order

1. **Adapt gait parameters** to satisfy constraints:
   - reduce cadence,
   - shorten step length,
   - reduce swing height,
   - increase duty cycle,
   - optionally downshift gait family.
2. **Reshape body velocity commands** (linear and angular) with jerk-aware/slew-limited trajectories.
3. **Only then** apply per-foot hard positional/velocity clamps as a final safety rail.

### Envelope dimensions

Maintain configurable limits for:

- `a_body_lin_max` (m/s²),
- `a_body_ang_max` (rad/s²),
- `a_foot_max` (m/s²),
- `v_foot_max` (m/s),
- `a_joint_max` (rad/s²),
- `v_joint_soft_max` (rad/s).

The joint limits are evaluated from predicted kinematics using local Jacobians around current joint state (approximation is acceptable for first pass).

## C) Explicit startup / shutdown phase coordinator

Add a small state machine (within `MotionLimiter` or adjacent helper):

- `IDLE`
- `BODY_PRELOAD`
- `LOCOMOTING`
- `BODY_SETTLE`

### Startup behavior (requested)

`IDLE -> BODY_PRELOAD -> LOCOMOTING`

- Begin body pose/velocity ramp immediately under body accel limits.
- Hold gait cadence near zero (or keep stance) during preload window until body motion reaches a threshold or timeout.
- Then ramp gait cadence/step in with acceleration budgets.

### Stop behavior (requested inverse)

`LOCOMOTING -> BODY_SETTLE -> IDLE`

- First ramp gait cadence/step to zero and settle feet in stance.
- Keep controlled body motion briefly to dissipate residual error, then settle body command to hold.

This formalizes the desired sequencing rather than relying on incidental module interactions.

## D) Integrate with dynamic gait policy rather than replacing it

Do not remove current `GaitPolicyPlanner` dynamic logic; extend it:

- Keep region/family selection and safety fallback.
- Add limiter feedback (e.g., `accel_scale`, `joint_margin`) so planner can pre-emptively choose less aggressive family/settings when margins shrink.
- Preserve existing servo overspeed scaling as a secondary safety net.

---

## Implementation Plan (Incremental)

## Phase 1 — Infrastructure and config

1. Add `MotionLimiter` class and wire into `ControlPipeline::step` between planner and gait/body execution.
2. Extend `control_config::GaitConfig` (or new `MotionLimitConfig`) with acceleration/velocity envelope parameters and startup/shutdown timings.
3. Extend telemetry/status payload with limiter diagnostics (active phase, scaling factors, dominant constraint).

**Exit criteria:** feature compiles; default behavior is equivalent (limits permissive by default).

## Phase 2 — Body linear/angular acceleration limiting

1. Track prior commanded body linear velocity and angular velocity.
2. Apply bounded delta per tick (`dv <= a_max * dt`) for body command outputs.
3. Route limited body command into `BodyController` and ensure yaw-specific slew in `BodyController` is either removed or aligned to avoid double-filter mismatch.

**Exit criteria:** deterministic ramp-up/down in body motion with unit tests for step commands.

## Phase 3 — Intelligent gait adaptation against foot/joint budgets

1. Predict near-term foot velocity/accel from candidate gait params (cadence, step length, swing).
2. Estimate joint velocity/accel margin using Jacobian-based mapping and current state.
3. Solve for a single conservative `scale in (0,1]` (or axis-specific scales) to satisfy all budgets.
4. Apply scale to gait params first; only hard-clamp residual violations.

**Exit criteria:** overspeed/overaccel scenarios reduce gait aggressiveness proactively without abrupt clipping.

## Phase 4 — Startup/shutdown phase coordinator

1. Implement `IDLE/BODY_PRELOAD/LOCOMOTING/BODY_SETTLE` transitions.
2. Gate cadence/step progression by phase and thresholds.
3. Validate “body leads on start, legs lead on stop” behavior in simulation scenario tests.

**Exit criteria:** phase sequencing observed in telemetry and scenario assertions.

## Phase 5 — Hardening and tuning

1. Tune defaults in sim: smoothness vs responsiveness.
2. Add safety watchdog hooks (if limiter saturates persistently, request derating/fallback).
3. Document operations tuning guide and expected telemetry signatures.

---

## Suggested File-Level Changes

- `hexapod-server/src/control/pipeline/control_pipeline.cpp`
  - instantiate and invoke new `MotionLimiter` stage.
- `hexapod-server/include/control/` + `src/control/pipeline/`
  - add `motion_limiter.hpp/.cpp`.
- `hexapod-server/include/control/control_config.hpp`
  - add config structs/defaults for acceleration limits and phase timings.
- `hexapod-server/src/config/tuning_section_parser.cpp`
  - parse new tuning keys.
- `hexapod-server/src/control/telemetry/telemetry_publisher.cpp`
  - expose limiter diagnostics.
- `hexapod-server/tests/`
  - add unit tests for limiter envelopes and phase transitions,
  - add integration tests for gait adaptation under forced joint-velocity pressure,
  - add scenario tests for startup/shutdown ordering.

---

## Test Plan

1. **Unit tests**
   - body linear accel clamp,
   - body angular accel clamp,
   - foot accel/velocity scaler,
   - startup/shutdown state machine transitions.
2. **Pipeline integration tests**
   - command step inputs and verify monotonic bounded ramps in outputs,
   - verify gait parameter adaptation before hard-clamp activation.
3. **Regression tests**
   - ensure existing gait-policy and safety-fallback tests still pass.
4. **Scenario tests**
   - start command: body starts moving before gait phase progression,
   - stop command: gait cadence reaches zero before body settles.

---

## Risks and Mitigations

- **Risk:** Too many cascaded filters create sluggish response.
  - **Mitigation:** consolidate primary velocity/accel shaping into `MotionLimiter`; simplify duplicated slews.
- **Risk:** Joint-accel estimate inaccuracies from Jacobian approximation.
  - **Mitigation:** conservative margins + fallback hard-clamps + telemetry for model error.
- **Risk:** Phase coordinator causes deadlocks under noisy commands.
  - **Mitigation:** hysteresis + timeout-based progress guarantees + explicit telemetry state.

---

## Recommendation

Proceed with Phases 1–2 first (infrastructure + body acceleration limits) behind a feature flag, then add Phase 3 intelligent gait adaptation, followed by Phase 4 startup/shutdown sequencing. This minimizes behavior risk while delivering immediate safety improvement.
