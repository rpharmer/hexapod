#!/usr/bin/env bash
# Capture JSONL from test_motion_performance_suite for baseline vs candidate trees.
# Usage:
#   ./scripts/run_motion_benchmark_ab.sh BASELINE_EXE CANDIDATE_EXE [extra args...]
# Env:
#   HEXAPOD_PHYSICS_SIM_EXE — path to hexapod-physics-sim (required if not passed after --)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BASE_BIN="${1:?baseline test_motion_performance_suite binary}"
SHIFT=1
CAND_BIN=""
if [[ $# -ge 2 && "$2" != --* ]]; then
  CAND_BIN="$2"
  SHIFT=2
fi
shift "$SHIFT" || true

SIM="${HEXAPOD_PHYSICS_SIM_EXE:-${ROOT}/hexapod-physics-sim/build/hexapod-physics-sim}"
if [[ ! -x "$SIM" ]]; then
  echo "HEXAPOD_PHYSICS_SIM_EXE must point to built hexapod-physics-sim (got: $SIM)" >&2
  exit 1
fi

OUT_DIR="${ROOT}/docs/testing-baselines/ab-runs"
mkdir -p "$OUT_DIR"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASE_JSONL="$OUT_DIR/baseline-${STAMP}.jsonl"

(cd "${ROOT}/hexapod-server" && \
  HEXAPOD_PHYSICS_SIM_EXE="$SIM" "$BASE_BIN" --profile full --emit-metrics-json "$@" 2>/dev/null | grep -E '^\{"(suite|name)"' >"$BASE_JSONL" || true)

echo "Wrote $BASE_JSONL"
if [[ -n "$CAND_BIN" ]]; then
  CAND_JSONL="$OUT_DIR/candidate-${STAMP}.jsonl"
  (cd "${ROOT}/hexapod-server" && \
    HEXAPOD_PHYSICS_SIM_EXE="$SIM" "$CAND_BIN" --profile full --emit-metrics-json "$@" 2>/dev/null | grep -E '^\{"(suite|name)"' >"$CAND_JSONL" || true)
  echo "Wrote $CAND_JSONL"
  echo "Diff line counts: baseline=$(wc -l <"$BASE_JSONL") candidate=$(wc -l <"$CAND_JSONL")"
fi
