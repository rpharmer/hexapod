# Locomotion baseline comparison: 2026-05-06 (`913e8b3`) vs 2026-05-09 (`3dd1add`)

Canonical profile aggregates (`median` columns from each `baseline-*.json`).

| Case | Metric | 2026-05-06 median | 2026-05-09 median | Δ vs old |
| --- | --- | --- | --- | --- |
| aggressive_governor | path_length_m | 0.358238 | 0.206386 | -42.4% |
| aggressive_governor | net_displacement_m | 0.158945 | 0.03526 | -77.8% |
| aggressive_governor | stride_count | 2 | 0 | -100.0% |
| aggressive_governor | mean_horizontal_speed_mps | 0.055975 | 0.032248 | -42.4% |
| aggressive_governor | peak_horizontal_speed_mps | 0.374305 | 0.360405 | -3.7% |
| aggressive_governor | max_body_rate_radps | 2.82145 | 3.48635 | +23.6% |
| aggressive_governor | pass_rate | 1.0 | 0.0 | n/a |
| command_timeout_fallback | path_length_m | 0.073889 | 0.10721 | +45.1% |
| command_timeout_fallback | net_displacement_m | 0.012367 | 0.007384 | -40.3% |
| command_timeout_fallback | stride_count | 0 | 0 | 0 |
| command_timeout_fallback | mean_horizontal_speed_mps | 0.018472 | 0.026802 | +45.1% |
| command_timeout_fallback | peak_horizontal_speed_mps | 0.178483 | 0.232228 | +30.1% |
| command_timeout_fallback | max_body_rate_radps | 1.1988 | 1.6566 | +38.2% |
| gait_transition_stability | path_length_m | 0.687741 | 0.511595 | -25.6% |
| gait_transition_stability | net_displacement_m | 0.347046 | 0.154609 | -55.4% |
| gait_transition_stability | stride_count | 8 | 2 | -75.0% |
| gait_transition_stability | mean_horizontal_speed_mps | 0.036582 | 0.027212 | -25.6% |
| gait_transition_stability | peak_horizontal_speed_mps | 0.345931 | 0.369433 | +6.8% |
| gait_transition_stability | max_body_rate_radps | 1.60328 | 2.11602 | +32.0% |
| gait_transition_stability | pass_rate | 1.0 | 0.0 | n/a |
| low_support_walk | path_length_m | 0.369392 | 0.399467 | +8.1% |
| low_support_walk | net_displacement_m | 0.174173 | 0.185389 | +6.4% |
| low_support_walk | stride_count | 4 | 4 | +0.0% |
| low_support_walk | mean_horizontal_speed_mps | 0.027161 | 0.029373 | +8.1% |
| low_support_walk | peak_horizontal_speed_mps | 0.319789 | 0.309664 | -3.2% |
| low_support_walk | max_body_rate_radps | 1.63678 | 1.52277 | -7.0% |
| steady_forward_walk | path_length_m | 0.881289 | 0.555386 | -37.0% |
| steady_forward_walk | net_displacement_m | 0.440843 | 0.153095 | -65.3% |
| steady_forward_walk | stride_count | 8 | 1 | -87.5% |
| steady_forward_walk | mean_horizontal_speed_mps | 0.050073 | 0.031556 | -37.0% |
| steady_forward_walk | peak_horizontal_speed_mps | 0.415307 | 0.353878 | -14.8% |
| steady_forward_walk | max_body_rate_radps | 1.55167 | 2.11137 | +36.1% |
| steady_forward_walk | pass_rate | 1.0 | 0.0 | n/a |
| tilt_safety_trip | path_length_m | 0.401602 | 0.26619 | -33.7% |
| tilt_safety_trip | net_displacement_m | 0.105954 | 0.077302 | -27.0% |
| tilt_safety_trip | stride_count | 3 | 0 | -100.0% |
| tilt_safety_trip | mean_horizontal_speed_mps | 0.028686 | 0.019014 | -33.7% |
| tilt_safety_trip | peak_horizontal_speed_mps | 0.407915 | 0.341349 | -16.3% |
| tilt_safety_trip | max_body_rate_radps | 2.31081 | 2.69092 | +16.4% |
| turn_in_place | path_length_m | 0.224477 | 0.241728 | +7.7% |
| turn_in_place | net_displacement_m | 0.009426 | 0.024006 | +154.7% |
| turn_in_place | stride_count | 5 | 2 | -60.0% |
| turn_in_place | mean_horizontal_speed_mps | 0.013051 | 0.014054 | +7.7% |
| turn_in_place | peak_horizontal_speed_mps | 0.166781 | 0.164541 | -1.3% |
| turn_in_place | max_body_rate_radps | 0.932706 | 1.68842 | +81.0% |
| turn_in_place | pass_rate | 1.0 | 0.0 | n/a |

## Notes

- **2026-05-09** capture used the same procedure as [README.md](README.md): five repetitions of `test_locomotion_regression_suite` with `--emit-metrics-json`, `RelWithDebInfo` sim + server `build-tests` preset, `config.physics-sim-wsl.txt` (default limits manifest).
- At `3dd1add`, the canonical suite process **exited with code 1** on every repetition (`suite_exit_codes` in `baseline-2026-05-09-3dd1add.json`). Primary visible gate failure: **`steady_forward_walk` / `turn_in_place` stride_count** far below `stride_count_min` (e.g. `steady_forward_walk` reported `stride_count` 0–1 vs gate 5). Treat this snapshot as a **regression/diagnostics** record until locomotion passes again.
- **2026-05-06** (`913e8b3`) canonical runs had **all exit codes 0** and `pass_rate` 1.0 for every case in the stored aggregate.

## Stress profile (`long_walk_observability`)

| Metric | 2026-05-06 median | 2026-05-09 median | Δ |
| --- | --- | --- | --- |
| path_length_m | 0.996909 | 1.55849 | +56.3% |
| net_displacement_m | 0.236177 | 0.231562 | -2.0% |
| mean_horizontal_speed_mps | 0.016615 | 0.025975 | +56.3% |
| peak_horizontal_speed_mps | 0.538007 | 0.384375 | -28.6% |
| stride_count | 5 | 6 | +20.0% |
| pass_rate | 0.0 | 0.0 | — |
