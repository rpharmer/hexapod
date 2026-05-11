# Locomotion Kinematic Pathway Audit

This note records the ownership boundaries used by the locomotion feasibility redesign.
It is intentionally separate from the implementation plan so the plan file remains stable.

## Baseline Evidence

- Latest focused stress artifact: `/tmp/hexapod_locomotion_regression/1778455026737/50977`
- Failing case: `long_walk_observability`
- Failure summary: fast tripod transition does not reduce sag promptly; first fault reported as `BODY_COLLAPSE` at step 2943 with a `TIP_OVER` summary.

## Current Ownership

- `CommandGovernor`: command scale, cadence scale, swing floor, body-height squat, recovery entry.
- `GaitScheduler`: nominal phase timing, gait transitions, duty factor, step length, swing height.
- `LocomotionStability`: post-gait support assessment, tilt/body-rate gait shaping, lift permission, all-stance hold.
- `BodyController`: body pose feedback, height hold, height integral, tilt squat, terrain stance bias, contact grace, final leg targets.
- `FootPlanners`: stance velocity, swing touchdown placement, measured capture, stability foothold bias.
- `SupportAssessment`: nominal/actual support metrics and control-facing static margin.
- `LegIK` and gravity feedforward: post-planner joint realization and optional stance-biased joint offsets.
- `Estimator`: realized-body height, tilt, contact state, and fusion diagnostics from raw joint/contact history.

## Conflicts To Remove

- Body height is changed by governor squat, body height hold, height integral, tilt squat, stability swing boost, and terrain stance Z bias.
- Support is assessed on preview gait, previous gait, post-stability gait, and final gait within a single control step.
- Stance is derived independently from gait phase, stability hold flags, raw contacts, fused phases, and gravity-feedforward stance checks.
- Nominal and actual support margins are both computed, but control still uses nominal by default.
- High-demand, emergency-tilt, sparse-support, and contact-grace thresholds are duplicated across modules.

## Migration Rule

New behavior should first be emitted as telemetry while compatibility behavior remains the default.
Behavior changes should route through explicit products:

- `LocomotionFeasibility`: support, margins, lift gates, deadlock, dynamic risk, recovery recommendation.
- `LegContactMode`: the per-leg stance/swing/hold/grace decision consumed by planning and feedforward.
- `HeightPolicy`: commanded height plus separate risk, compliance, terrain, and clearance terms.
