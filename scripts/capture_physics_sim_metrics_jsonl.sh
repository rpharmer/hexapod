#!/usr/bin/env bash
# Append one run of metrics JSON lines from a physics-sim integration test binary.
#
# Usage (from repo root):
#   export HEXAPOD_PHYSICS_SIM_EXE=/path/to/hexapod-physics-sim
#   ./scripts/capture_physics_sim_metrics_jsonl.sh docs/testing-baselines/ab-runs/walk-$(date -u +%Y%m%dT%H%M%SZ).jsonl \
#     ./hexapod-server/build-tests/test_physics_sim_walk_distance \
#     "$HEXAPOD_PHYSICS_SIM_EXE"
#
# The binary is invoked as: BIN --emit-metrics-json [remaining args...]
# stderr is discarded so human logs do not mix into the JSONL file.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:?output .jsonl path}"
BIN="${2:?path to test binary}"
shift 2

mkdir -p "$(dirname "$OUT")"
SIM="${HEXAPOD_PHYSICS_SIM_EXE:-}"
if [[ -n "$SIM" && ! -x "$SIM" ]]; then
  echo "HEXAPOD_PHYSICS_SIM_EXE is not executable: $SIM" >&2
  exit 1
fi

(cd "${ROOT}/hexapod-server" && \
  HEXAPOD_PHYSICS_SIM_EXE="${HEXAPOD_PHYSICS_SIM_EXE:-}" \
  "$BIN" --emit-metrics-json "$@" 2>/dev/null | grep -E '^\{"(suite|name)"' >>"$OUT")

echo "Appended JSON metrics lines to $OUT"
