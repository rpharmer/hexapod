#!/usr/bin/env bash
# Grid sweep for stand-quiescence gravity feedforward tuning (Linux + sim binary).
# Usage from repo root:
#   HEXAPOD_PHYSICS_SIM_EXE=/path/to/hexapod-physics-sim ./scripts/sweep_gravity_feedforward_stand.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT}/hexapod-server/build-tests/test_gravity_feedforward_stand_quiescence"
SIM="${HEXAPOD_PHYSICS_SIM_EXE:-${ROOT}/hexapod-physics-sim/build/hexapod-physics-sim}"
if [[ ! -x "$BIN" ]]; then
  echo "missing $BIN (cmake --preset tests && cmake --build --preset tests --target test_gravity_feedforward_stand_quiescence)" >&2
  exit 2
fi
if [[ ! -x "$SIM" ]]; then
  echo "set HEXAPOD_PHYSICS_SIM_EXE to hexapod-physics-sim executable" >&2
  exit 2
fi

export HEXAPOD_FF_CSV=1
export HEXAPOD_PHYSICS_SIM_EXE="$SIM"

echo "scale_femur,scale_tibia,stiff,lpf_tau,exit_code,note"
# lpf_tau column fixed at 0.08 to match harness defaults
for sf in 0.40 0.46 0.52 0.58; do
  for stiff in 0.50 0.62 0.74; do
    export HEXAPOD_FF_SCALE_FEMUR=$sf
    export HEXAPOD_FF_SCALE_TIBIA=$sf
    export HEXAPOD_FF_STIFFNESS_GAIN_SCALE=$stiff
    export HEXAPOD_FF_DELTA_LPF_TAU_S=0.08
    set +e
    log=$("$BIN" 2>&1)
    ec=$?
    set -e
    echo "$log" | grep '^[01],' || true
    echo "$sf,$sf,$stiff,0.08,$ec,see_rows_above"
  done
done
