# traversability_analyzer Contract (v1)

## Responsibilities
- Convert world-model data into traversal risk/cost/confidence layers.
- Fuse occupancy, terrain gradient, obstacle-distance bands, slope bands, and confidence-zone statistics into a single risk/cost decision surface.

## Inputs
- `world_model` map slices
- Robot footprint/kinematics constraints

## Outputs
- Layer set with shared indexing:
  - `cost_layer`
  - `risk_layer`
  - `confidence_layer`
- Runtime decision outputs consumed by planners:
  - `traversable` gate (hard stop when false)
  - `reason` and `non_traversable_reasons` (deterministic ordered fault reasons)
- Metadata:
  - `timestamp_ms`
  - `frame_id`
  - generation parameters/version

## Runtime Tunables (config-driven)
- `Runtime.Autonomy.Traversability.OccupancyRiskWeight`
- `Runtime.Autonomy.Traversability.GradientRiskWeight`
- `Runtime.Autonomy.Traversability.ObstacleNearRiskWeight`
- `Runtime.Autonomy.Traversability.ObstacleMidRiskWeight`
- `Runtime.Autonomy.Traversability.ObstacleFarRiskWeight`
- `Runtime.Autonomy.Traversability.SlopeHighRiskWeight`
- `Runtime.Autonomy.Traversability.ConfidenceUnknownPenalty`
- `Runtime.Autonomy.Traversability.ConfidenceCostWeight`
- `Runtime.Autonomy.Traversability.RiskBlockThreshold`
- `Runtime.Autonomy.Traversability.ConfidenceBlockThreshold`

## Assumptions / Limits
- Planner integration assumes risk/cost are normalized to `[0, 1]` before policy gating.
- Unknown-confidence zones are penalized; high unknown fraction can block traversal even when occupancy alone looks nominal.

## Fault Behavior
- If analysis fails, emit explicit fault and last-valid age; do not silently reuse outdated layers beyond TTL.
