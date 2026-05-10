# Testing Baselines

This directory stores reproducible motion-performance baseline snapshots.

## Latest capture (2026-05-09, `3dd1add`)

- Summary metadata:
  - [`baseline-2026-05-09-3dd1add.json`](baseline-2026-05-09-3dd1add.json)
- Raw per-run case metrics:
  - [`baseline-2026-05-09-3dd1add.jsonl`](baseline-2026-05-09-3dd1add.jsonl)
- Stress summary metadata:
  - [`baseline-stress-2026-05-09-3dd1add.json`](baseline-stress-2026-05-09-3dd1add.json)
- Stress raw per-run case metrics:
  - [`baseline-stress-2026-05-09-3dd1add.jsonl`](baseline-stress-2026-05-09-3dd1add.jsonl)

**Important:** at this commit the canonical `test_locomotion_regression_suite` process **returned exit code 1** on all five repetitions (see `suite_exit_codes` in the JSON). Gates such as `stride_count_min` fail for `steady_forward_walk` / `turn_in_place` (robot stops scaling commands: `min_command_scale` hits 0). Use this snapshot for **trend/diff analysis**, not as a “green” release reference.

Side-by-side numbers vs the 2026-05-06 reference: [`COMPARISON-2026-05-09.md`](COMPARISON-2026-05-09.md).

## Reference capture (2026-05-06, `913e8b3`)

- Summary metadata:
  - [`baseline-2026-05-06-913e8b3.json`](baseline-2026-05-06-913e8b3.json)
- Raw per-run case metrics:
  - [`baseline-2026-05-06-913e8b3.jsonl`](baseline-2026-05-06-913e8b3.jsonl)
- Stress summary metadata:
  - [`baseline-stress-2026-05-06-913e8b3.json`](baseline-stress-2026-05-06-913e8b3.json)
- Stress raw per-run case metrics:
  - [`baseline-stress-2026-05-06-913e8b3.jsonl`](baseline-stress-2026-05-06-913e8b3.jsonl)

Canonical runs: **exit code 0** every repetition; `pass_rate` **1.0** on all aggregated cases.

## File conventions

- `baseline-<date>-<shortSHA>.json`
  - baseline metadata and aggregated per-case metric statistics
  - includes commit IDs, profile, run counts, and min/median/max values
- `baseline-<date>-<shortSHA>.jsonl`
  - one JSON object per case result per run (from `--emit-metrics-json`)
- `ab-runs/*.jsonl`
  - ad hoc A/B captures from [`scripts/run_motion_benchmark_ab.sh`](../scripts/run_motion_benchmark_ab.sh), [`scripts/capture_physics_sim_metrics_jsonl.sh`](../scripts/capture_physics_sim_metrics_jsonl.sh), and related helpers

### JSONL line shape

Each line should include at least `name`, `passed`, and `metrics`. Preferred fields (see `docs/TESTING_FUNCTIONALITY.md`, **Unified metrics JSONL schema**):

- `suite` — which binary family produced the line (e.g. `motion_performance`, `locomotion_regression`)
- `limits_applied` — numeric thresholds in effect for that run (for tightening and diffing)

Aggregate with `scripts/suggest_gate_limits.py` over one or more `.jsonl` files.

### P3 gate manifests (`gates/default-v1.json`)

Numeric pass/fail thresholds for several quantitative test binaries can be overridden via [`gates/default-v1.json`](gates/default-v1.json) (or another file passed with `--limits-manifest` / `HEXAPOD_TEST_LIMITS_MANIFEST`). See **P3: External limits manifests** in [`docs/TESTING_FUNCTIONALITY.md`](../TESTING_FUNCTIONALITY.md) for resolution order, schema notes, and `suggest_gate_limits.py --emit-manifest-fragment`.

## How to regenerate a baseline

1. Fixed environment: same `CMAKE_BUILD_TYPE` (e.g. `RelWithDebInfo` for the sim), same `hexapod-server` `--preset tests` build, same `config.physics-sim-wsl.txt` / limits manifest as the run you want comparable to.
2. From repo root (Linux; suite is Linux-only):

```bash
cmake -S hexapod-physics-sim -B hexapod-physics-sim/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build hexapod-physics-sim/build -j"$(nproc)"

cmake --preset tests -S hexapod-server -B hexapod-server/build-tests
cmake --build hexapod-server/build-tests -j"$(nproc)"

SUITE=hexapod-server/build-tests/test_locomotion_regression_suite
SIM=hexapod-physics-sim/build/hexapod-physics-sim
OUT=docs/testing-baselines/baseline-$(date +%F)-$(git rev-parse --short=7 HEAD).jsonl

rm -f "$OUT" /tmp/loc_exit.txt
for _ in 1 2 3 4 5; do
  set +e
  "$SUITE" --sim "$SIM" --profile canonical --emit-metrics-json 2>/dev/null \
    | grep -E '^\{"(suite|name)"' >>"$OUT"
  echo $? >>/tmp/loc_exit.txt
  set -e
done
CODES=$(tr '\n' ',' </tmp/loc_exit.txt | sed 's/,$//')

python3 scripts/aggregate_locomotion_metrics_baseline.py "$OUT" \
  -o "${OUT%.jsonl}.json" \
  --baseline-name "canonical-local-$(date +%F)" \
  --server-commit "$(git rev-parse HEAD)" \
  --sim-commit "$(git rev-parse HEAD)" \
  --suite-profile canonical \
  --repetitions 5 \
  --exit-codes "$CODES"
```

3. Repeat with `--profile stress`, writing `baseline-stress-<date>-<sha>.jsonl` and passing `--suite-profile stress` to the aggregator.

The helper [`scripts/aggregate_locomotion_metrics_baseline.py`](../scripts/aggregate_locomotion_metrics_baseline.py) emits the same **aggregate metric key set** as the existing `baseline-*-*.json` files (min/median/max for motion scalars plus `pass_rate`).

Note: stress-profile baselines may record non-zero suite exit codes depending on long-walk expectations; the JSONL still captures comparable numeric metrics for trend analysis.
