# Lateral + Yaw Stability Review

Date: 2026-05-07

## Problem Statement

`physics_sim_oblique_walk_clearance` has been a long-running weak spot. The current behavior suggests more than one module is responding to the same disturbance, which creates stacked corrections instead of a clean recovery hierarchy.

## Observed Failure Modes

- Late ripple/wave oblique phases accumulate roll and body-height error instead of settling.
- Combined lateral velocity and yaw can stack roll commands beyond what the support state can comfortably hold.
- Swing-clearance, stance hold, governor scaling, and safety reactions can all activate on the same disturbance window.

## Module Ownership Matrix

| Module | Current practical role | Review note |
| --- | --- | --- |
| `body_pose_controller` | Adds nominal lean/pose setpoint from command and support margin | Can over-stack lateral and yaw roll requests |
| `body_controller` | Converts gait + pose into stance/swing foot targets | Also adds measured tilt feedback and height-hold behavior |
| `locomotion_stability` | Applies support-margin hold logic, stride/cadence scaling, swing-height boosts | Influences the same disturbance window as pose shaping |
| `command_governor` | Scales command magnitude, cadence, squat, swing floor | Another layer reacting to tilt/support stress |
| `contact_foot_response` | Alters late-swing touchdown timing and downward extension | Can turn uncertainty into foot drag if overused |
| `gait_scheduler` / `gait_params` | Phase timing and gait envelope | Shapes how much disturbance the lower layers must absorb |
| `safety_supervisor` | Trips when tilt/body-rate/collapse exceed limits | Should be last-resort, not part of nominal recovery |

## Conflicting Control Loops To Watch

- Nominal roll lean from `body_pose_controller` versus corrective roll feedback in `body_controller`
- Swing-height boosts in `locomotion_stability` versus late-swing downward push in `contact_foot_response`
- Governor squat/command scaling versus body-height hold trying to keep the chassis up
- Stability hold preserving stance while other modules still assume the next lift cadence

## Agreed Control Hierarchy

1. `body_pose_controller`
   - Owns nominal command-derived pose shaping.
   - Must stay conservative under combined lateral+yaw demand.
2. `locomotion_stability`
   - Owns support-margin preservation, cadence moderation, and swing-height boosts.
   - Should be the primary locomotion stability layer.
3. `body_controller`
   - Owns foot target realization and measured pose correction.
   - Should support the first two layers, not reintroduce a second large nominal lean.
4. `command_governor`
   - Owns envelope reduction when the commanded motion is too aggressive.
   - Should reduce demand, not fight detailed pose control.
5. `safety_supervisor`
   - Owns faulting when recovery failed.

## Implementation Decisions

- Keep explicit support telemetry in replay so lateral+yaw failures can be separated into planner, contact-fusion, and foot-placement causes.
- Reduce additive roll stacking from simultaneous yaw and lateral lean in `body_pose_controller`.
- Prefer evidence-based late-touchdown shaping only when contact fusion says touchdown is plausible.
- Keep wave/diagonal gait tuning as a later step after support and pose ownership are cleaner.

## Validation Checklist

- `physics_sim_oblique_walk_clearance --emit-metrics-json` passes repeatedly.
- Oblique failures, if any, can be attributed to one dominant layer instead of several stacked responses.
- Replay shows fewer late-phase roll spikes and less body-height overshoot.
- No regression in forward tripod locomotion while tuning oblique behavior.
