# Locomotion Foundations Plan

Date: 2026-05-07

## Objective

Improve locomotion quality before returning to navigation by fixing the foundational support, telemetry, and gait behaviors that currently drive foot dragging, weak static support, and unstable oblique walking.

## Scope

- `hexapod-server` locomotion support, gait, stance/swing, telemetry, and physics-backed test coverage
- Docs and quantitative gates that define the new support/debug expectations

## Non-scope

- Navigation behavior changes beyond the observability needed to keep existing acceptance tests meaningful
- Firmware, visualiser rendering, or unrelated physics-engine refactors

## Ordered Phases

### Phase 0: Three-leg static support strength

Goal:
- Make the robot hold a difficult three-leg support pose with low body creep, low joint drift, and bounded commanded-vs-measured leg error.

Acceptance:
- `physics_sim_tripod_support_baseline` stays well above collapse height.
- Tripod support height creep remains bounded.
- Tripod support joint drift remains small under load.
- Tripod support commanded-vs-measured foot error remains bounded.
- Support-foot world drift is emitted and tracked, even if future tightening is still needed.

### Phase 1: Locomotion debug signal quality

Goal:
- Separate planner intent from physical support so tests and replay can explain why a gait is good or bad.

Acceptance:
- `LocomotionDebugSnapshot` exposes:
  - `planned_stance`
  - `raw_contact`
  - `fused_support`
  - `fused_contact_phase`
  - `fused_contact_confidence`
- Replay and telemetry include those fields.
- Stride/touchdown logic consumes fused support instead of `gait.in_stance` edges alone.

### Phase 2: Stance-anchor / contact-tracking quality

Goal:
- Reduce visible foot dragging and improve stance-world anchoring quality.

Acceptance:
- Contact anchors follow fused support semantics instead of being reset by weaker contact evidence.
- Late-swing touchdown shaping only pushes feet down when touchdown evidence exists.
- Motion-performance and locomotion-regression cases show better support-event fidelity and cleaner drag behavior.

### Phase 3: Lateral+yaw interaction review

Goal:
- Review module ownership before further oblique-walk tuning so the stack stops fighting itself.

Acceptance:
- `docs/PLAN_LATERAL_YAW_REVIEW.md` captures:
  - failure modes
  - ownership matrix
  - conflicting loops
  - agreed hierarchy

### Phase 4: Lateral+yaw pose stability implementation

Goal:
- Reduce body-height overshoot and late instability in oblique walking using the ownership model from phase 3.

Acceptance:
- `physics_sim_oblique_walk_clearance` passes repeatedly with improved stability margin.
- Oblique walking shows less stacked roll command from multiple modules.

### Phase 5: Wave and diagonal gait tuning

Goal:
- Improve low-speed wave clearance and reduce diagonal/compass scrape-first behavior.

Acceptance:
- Wave gait retains stronger clearance at low speed.
- Diagonal and lateral headings avoid being the first cases to scrape or destabilize.
- Tripod remains the reference gait and must not regress.

## Metrics To Watch

- `tripod_mean_body_height_m`
- `tripod_body_height_creep_m`
- `tripod_max_support_joint_drift_rad`
- `tripod_max_support_commanded_tracking_error_m`
- `tripod_max_support_foot_world_drift_m`
- `max_contact_anchor_max_drift_m`
- `max_contact_tracking_error_m`
- `min_measured_foot_world_z_m`
- `touchdown_events`
- `planned_touchdown_events`
- `fused_touchdown_events`
- `max_abs_roll_rad`
- `max_abs_pitch_rad`
- `max_body_rate_radps`

## Tests After Each Phase

- Phase 0:
  - `test_physics_sim_tripod_support_baseline --emit-metrics-json`
- Phase 1:
  - `test_telemetry_json_serialization`
  - `test_replay_logging`
  - `test_motion_performance_suite --profile full --emit-metrics-json`
- Phase 2:
  - `test_contact_foot_response`
  - `test_locomotion_regression_suite --profile canonical --emit-metrics-json`
  - `test_motion_performance_suite --profile full --emit-metrics-json`
- Phase 3:
  - docs review only
- Phase 4:
  - `test_physics_sim_oblique_walk_clearance --emit-metrics-json`
- Phase 5:
  - `test_motion_performance_suite --profile full --emit-metrics-json`
  - `./scripts/verify.sh`

## Docs To Keep Updated

- `docs/ALGORITHMS_SERVER_LOCOMOTION.md`
- `docs/ALGORITHMS_SERVER_CONFIG_TELEMETRY.md`
- `docs/VISUALISER_TELEMETRY.md`
- `docs/TESTING_FUNCTIONALITY.md`
- `docs/PLAN_LATERAL_YAW_REVIEW.md`

## Open Risks

- Tripod support foot world drift is still materially larger than the ideal target, so this metric is currently informative before it is aggressively tightened.
- Oblique stability remains sensitive to multi-module pose shaping and may require more than one control-pass refinement.
- Stride-event quality now depends on fused support transitions being credible; if support fusion regresses, gait-quality failures will become louder instead of silently skipping.

## Assumptions

- Navigation stays out of scope until the locomotion foundation metrics are stable.
- Fused support is the authoritative physical-support signal for metrics and replay.
- Visible foot dragging is treated as a real locomotion defect, not just a telemetry artifact.
