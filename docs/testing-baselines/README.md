# Testing Baselines

This directory stores reproducible motion-performance baseline snapshots.

## Current baseline

- Summary metadata:
  - `baseline-2026-05-06-913e8b3.json`
- Raw per-run case metrics:
  - `baseline-2026-05-06-913e8b3.jsonl`
- Stress summary metadata:
  - `baseline-stress-2026-05-06-913e8b3.json`
- Stress raw per-run case metrics:
  - `baseline-stress-2026-05-06-913e8b3.jsonl`

## File conventions

- `baseline-<date>-<shortSHA>.json`
  - baseline metadata and aggregated per-case metric statistics
  - includes commit IDs, profile, run counts, and min/median/max values
- `baseline-<date>-<shortSHA>.jsonl`
  - one JSON object per case result per run (from `--emit-metrics-json`)
- `ab-runs/*.jsonl`
  - ad hoc A/B captures from `[scripts/run_motion_benchmark_ab.sh](../scripts/run_motion_benchmark_ab.sh)`, `[scripts/capture_physics_sim_metrics_jsonl.sh](../scripts/capture_physics_sim_metrics_jsonl.sh)`, and related helpers

### JSONL line shape

Each line should include at least `name`, `passed`, and `metrics`. Preferred fields (see `docs/TESTING_FUNCTIONALITY.md`, **Unified metrics JSONL schema**):

- `suite` — which binary family produced the line (e.g. `motion_performance`, `locomotion_regression`)
- `limits_applied` — numeric thresholds in effect for that run (for tightening and diffing)

Aggregate with `scripts/suggest_gate_limits.py` over one or more `.jsonl` files.

### P3 gate manifests (`gates/default-v1.json`)

Numeric pass/fail thresholds for several quantitative test binaries can be overridden via `[gates/default-v1.json](gates/default-v1.json)` (or another file passed with `--limits-manifest` / `HEXAPOD_TEST_LIMITS_MANIFEST`). See **P3: External limits manifests** in `[docs/TESTING_FUNCTIONALITY.md](../TESTING_FUNCTIONALITY.md)` for resolution order, schema notes, and `suggest_gate_limits.py --emit-manifest-fragment`.

## How this baseline was generated

- Suite: `hexapod-server/build-tests/test_locomotion_regression_suite`
- Profile: `canonical`
- Repetitions: `5`
- Sim binary: `hexapod-physics-sim/build/hexapod-physics-sim`
- Commits:
  - server: `913e8b3ea05cf4ed837ba693fc09c606e5a6a8e6`
  - sim: `913e8b3ea05cf4ed837ba693fc09c606e5a6a8e6`

To generate a new baseline, keep config/build type/environment fixed and follow the protocol in `docs/TESTING_FUNCTIONALITY.md` under `A/B Benchmark Protocol`.

Note: stress-profile baselines may record non-zero suite exit codes depending on current long-walk expectations; the JSONL still captures comparable numeric metrics for trend analysis.