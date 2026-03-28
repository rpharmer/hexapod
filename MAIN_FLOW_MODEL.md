# MAIN FLOW MODEL

## Purpose
This document captures the runtime data/control flow from sensors to actuator commands, then lists concrete investigation findings (potential bugs/risks) discovered while tracing that flow.

---

## End-to-end runtime flow

### 1) Input acquisition (`RobotRuntime::busStep`)
1. Read hardware bridge state into `raw`.
   - If bridge read fails, mark bus unhealthy (`raw.bus_ok = false`).
2. If IMU reads are enabled, read IMU sample and, when valid:
   - copy orientation/angular velocity into `raw.body_pose_state`
   - copy body linear velocity when available
   - set `raw.has_body_pose_state = true`
3. Assign sample id if missing.
4. Publish `raw` into shared state (`raw_state_`).
5. Read latest `joint_targets_`, apply a write-side slew limiter, and write to hardware bridge.

**Key implications**
- Raw body pose availability is determined here.
- Output command rate limiting exists at this final write boundary.

---

### 2) Estimation (`RobotRuntime::estimatorStep` -> `SimpleEstimator::update`)
1. Estimator starts from raw joint/contact/body-pose data.
2. Computes joint velocities by differentiation.
3. Uses stance-foot kinematics/contact history to estimate planar body velocity.
4. If IMU pose is present (`raw.has_body_pose_state`), uses IMU orientation/angular velocity and fuses linear velocity.
5. Solves support ground plane from contact points when possible and derives roll/pitch/height.
6. If no raw IMU pose is present but ground-plane solve succeeds, estimator sets inferred orientation and marks `est.has_body_pose_state = true`.
7. Writes estimated state (`estimated_state_`).

**Key implications**
- Estimator can provide body pose even when no IMU sample is present (ground-plane path).
- Current semantics intentionally/explicitly treat inferred pose as available.

---

### 3) Safety (`RobotRuntime::safetyStep` -> `SafetySupervisor::evaluate`)
1. Read `raw`, `est`, `intent`.
2. Evaluate lenient freshness for estimator/intent.
3. Compute safety fault decision and stability state.
4. TIP_OVER path:
   - tilt-based trigger can use body pose availability
   - support-polygon trigger is independent
   - tilt-only trips are debounced by sample count
   - hard electrical fault context can bypass debounce behavior
5. Write `safety_state_`.

**Key implications**
- Motion gating and torque-cut decisions are centralized here.
- TIP_OVER combines geometric support and tilt/electrical context.

---

### 4) Control (`RobotRuntime::controlStep`)
1. Read `est`, `intent`, `safety`, `bus_ok`.
2. Run strict freshness gating:
   - reject path emits safe output without normal pipeline progression.
3. Resolve autonomy overlay intent (if autonomy enabled).
4. Run `ControlPipeline::runStep(est, intent, safety, loop_dt, bus_ok, loop_counter)`.
5. Write resulting joint targets/status.
6. Record diagnostics and publish telemetry.

---

## Control pipeline internals (`ControlPipeline::runStep`)
1. `GaitPolicyPlanner::plan` -> runtime gait policy
2. `MotionLimiter::update` -> limited intent + adapted gait policy
3. `GaitScheduler::update` -> gait phase/state
4. `ContactManager::update` -> contact-adjusted gait/policy (unless bypassed)
5. `BodyController::update` -> leg-space targets/velocities
6. `IK::solve` -> joint targets
7. Status assembly and return

---

## Major side channels
- Telemetry publish path (`maybePublishTelemetry`) sends estimated state, intent, outputs, and status.
- Diagnostics report path logs runtime metrics, variability, freshness counters, and safety snapshots.
- Autonomy path can override/shape intent before entering pipeline.

---

## Investigation findings: potential issues/bugs

## A) **Diagnostics phase-rate spike due phase wrap math (likely bug)**
`runtime_diagnostics_reporter` computes leg-phase rate via plain absolute subtraction between wrapped phase samples.
When phase wraps `~0.99 -> ~0.01`, naive subtraction looks like a large jump, inflating `peak_phase_delta_per_s`.

**Why it matters**
- Misleading observability: can look like gait instability when it is normal wrap behavior.
- Can distract triage from actual root causes.

**Suggested fix**
- Use shortest circular distance on unit interval, e.g. `min(|d|, 1-|d|)` before dividing by dt.

---

## B) **Pose availability semantics are overloaded (design risk)**
`has_body_pose_state` now indicates either:
1) measured pose from IMU, or
2) inferred pose from ground-plane solver.

This works with current request, but it conflates provenance/quality and may complicate downstream logic (e.g., safety thresholds vs controller stabilization confidence).

**Suggested improvement**
- Introduce explicit provenance/quality flags, e.g.:
  - `has_measured_body_pose_state`
  - `has_inferred_body_pose_state`
  - optional confidence scalar.

---

## C) **Control loop dt is configured period, not measured period (modeling risk)**
Pipeline `loop_dt` currently uses configured control period rather than measured jittered dt.
This is stable and deterministic, but may under/over-estimate limits during timing excursions.

**Suggested improvement**
- feed measured control-loop dt into limiter/scheduler with guardrails and sane clipping.

---

## D) **Level-hold can command against inferred pose quality (tunable risk)**
Body level-hold currently gates on `est.has_body_pose_state`; after blind flip this includes inferred ground-plane poses.
In rough/noisy contact conditions this may inject corrective motion from weaker pose estimates.

**Suggested mitigation**
- keep gain conservative (already done)
- optionally gate on minimum support-contact count / stability margin
- optionally low-pass orientation before correction.

---

## Recommended next debugging steps
1. Fix phase wrap-rate diagnostic math (A) first; low risk, high observability value.
2. Add pose provenance fields (B), then route safety/controller gates by provenance.
3. Evaluate measured-dt pipeline behavior under load (C).
4. Add confidence-based level-hold gating (D) if oscillation seen in uneven-contact scenarios.

