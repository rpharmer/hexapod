# traversability_analyzer Contract (v1)

## Responsibilities
- Convert world-model data into traversal risk/cost/confidence layers.

## Inputs
- `world_model` map slices
- Robot footprint/kinematics constraints

## Outputs
- Layer set with shared indexing:
  - `cost_layer`
  - `risk_layer`
  - `confidence_layer`
- Metadata:
  - `timestamp_ms`
  - `frame_id`
  - generation parameters/version

## Fault Behavior
- If analysis fails, emit explicit fault and last-valid age; do not silently reuse outdated layers beyond TTL.
