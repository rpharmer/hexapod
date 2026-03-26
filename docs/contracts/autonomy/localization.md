# localization Contract (v1)

## Responsibilities
- Provide robot pose/velocity estimate with covariance/quality metadata.
- Abstract underlying estimator implementation (including current `SimpleEstimator`).

## Output Schema
- `pose` in declared frame (`map` or `odom`)
- `twist`
- optional covariance blocks
- `quality` enum: `GOOD`, `DEGRADED`, `INVALID`
- `timestamp_ms`, `sample_id`

## Timing
- Publish target rate: >= 50 Hz for control consumers.
- Consumers must enforce freshness via `max_age_ms`.

## Fault Behavior
- Estimator invalidity must be surfaced as `quality=INVALID` plus typed fault.
- Localization dropout should not be silent.
