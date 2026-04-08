# Face-4 friction coherence enablement note

`useFace4PointNormalBlock` remains **off by default**. Enable it by default only when CI regression baselines consistently satisfy the type-9 manifold gates encoded in `regression_scene_suite.cpp`:

- tangent basis churn ratio at or below the configured ceiling
- stable type-9 contact support at or above the configured minimum
- type-9 fallback rate at or below the configured ceiling

These gates are tied to `face4RequireFrictionCoherence` telemetry checks and are intended to prevent enabling face-4 normal blocking in scenes where tangent-basis reuse and manifold-friction continuity are still unstable.
