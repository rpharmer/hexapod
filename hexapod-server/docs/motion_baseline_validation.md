# Motion Baseline Validation Checklist

Use this checklist after each investigation stage. Keep gait, command profile, terrain, and run duration fixed so results are comparable.

## Fixed test setup

- Runtime mode: `sim` or `physics-sim` (do not mix within one comparison set).
- Scenario: `scenarios/01_nominal_stand_walk.toml`.
- Duration: full scenario.
- Config profile: same file except for the stage toggles under `Runtime.Investigation.*`.

## Capture per run

- Process exit code and scenario completion status.
- Number of freshness gate rejects.
- Number of active safety fault transitions.
- Mean control loop dt and max control jitter from diagnostics logs.
- Stance support and static stability margin snapshots.

## Promotion criteria

- Move to next stage only if the target bug still reproduces.
- Do not promote if new faults appear (freshness or safety) that were absent in the prior stage.
- If regressions appear, roll back one stage and keep high-risk toggles disabled.

## Rollback

- Set all `Runtime.Investigation.Bypass*` flags to `false`.
- Keep baseline feature-disable flags explicit in config for reproducibility.
