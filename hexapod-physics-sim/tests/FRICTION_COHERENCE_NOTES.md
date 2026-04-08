# Face-4 friction coherence readiness checklist

`useFace4PointNormalBlock` stays **off by default** until this checklist is consistently green in CI.

## Required scene set

Run `regression_scene_suite` and require all of the following scenes:

- `face-contact slide-to-rest`
- `face-contact mild rocking`
- `face-contact stick-slip transition`
- `face4 ill-conditioned fallback case`

## Numeric gates (must pass per run)

- [ ] **Basis churn ratio** (`block` run) ≤ **0.90** on type-9-heavy scenes.
- [ ] **Stable type-9 contact support** (`block` run mean stable type-9 contacts) ≥ **0.00**.
- [ ] **Type-9 fallback rate** (`block` run manifold-type[9] fallback rate) ≤ **0.95**.
- [ ] **Face4 attempts present** (`face4` run) attempted count > **0** when type-9-heavy scenes are eligible.
- [ ] **Face4 fallback-to-scalar budget** (`face4FallbackToScalar / face4Attempted`) ≤ **0.25**.
- [ ] **Face4 regression vs block baseline**:
  - penetration regression ≤ **0.75**
  - jitter stddev regression ≤ **0.35**
- [ ] **Ill-conditioned fallback route**: `face4 ill-conditioned fallback case` reports face4 attempt(s) and at least one face4→block2 fallback.

## Promotion rule

Only flip `ContactSolverConfig::useFace4PointNormalBlock` default to `true` after the checklist above is stable across multiple CI runs (not a single green run).
