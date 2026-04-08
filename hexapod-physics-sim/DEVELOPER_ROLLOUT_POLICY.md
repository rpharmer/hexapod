# Face-4 rollout policy (`useFace4PointNormalBlock`)

Default remains `false` until regression automation proves readiness on every CI run.

## Required scenes (hard gate)

`tests/regression_scene_suite.cpp` enforces that all of these scenes are present and passing:

- `face-contact slide-to-rest`
- `face-contact mild rocking`
- `face-contact stick-slip transition`
- `face4 ill-conditioned fallback case`

## Objective telemetry gates

For type-9-heavy face-contact scenes, the regression harness fails when any gate is violated:

- basis churn ratio (`block`) <= `0.90`
- mean stable type-9 contacts (`block`) >= `0.00`
- type-9 fallback rate (`block`) <= `0.95`
- face4 attempted count (`face4`) >= `1`
- face4 fallback-to-scalar rate (`face4FallbackToScalar / face4Attempted`) <= `0.25`
- face4 regression vs block baseline:
  - penetration regression <= `0.75`
  - jitter stddev regression <= `0.35`

For the ill-conditioned fallback scene, the harness also requires:

- face4 attempted count >= `1`
- at least one deterministic `face4 -> block2` fallback

## Default-setting criteria

Flip `ContactSolverConfig::useFace4PointNormalBlock` default to `true` **only after**:

1. The rollout policy gate passes in CI repeatedly (multiple runs, not a single run).
2. Required scenes remain in the regression suite with no local overrides.
3. No new fallback-thrash/topology-regression diagnostics are introduced by the change.

Until those conditions are met, keep the shipped default at `false` and rely on explicit opt-in for experiments.
