#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
VIS_DIR="$ROOT_DIR/hexapod-visualiser"
VENV_DIR="$VIS_DIR/.venv"
INSTALL_DEPS=0
SIMULATE=0
SIM_HZ=30
SIM_AUTONOMY_SCENARIO="patrol"
SERVER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_visualiser.sh [options] [-- <server args>]

Starts hexapod-visualiser with optional virtualenv/dependency setup.

Options:
  --install-deps           Force dependency installation from requirements.txt.
  --simulate               Also run simulate_telemetry.py in the background.
  --simulate-hz <hz>       Telemetry simulator rate (default: 30).
  --simulate-scenario <id> Autonomy simulator scenario (run --help in simulate_telemetry.py for full list).
  -h, --help               Show this help text.

Examples:
  scripts/run_visualiser.sh
  scripts/run_visualiser.sh --install-deps -- --http-port 8081 --udp-port 9871
  scripts/run_visualiser.sh --simulate -- --http-port 8080 --udp-port 9870
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-deps)
      INSTALL_DEPS=1
      shift
      ;;
    --simulate)
      SIMULATE=1
      shift
      ;;
    --simulate-hz)
      SIM_HZ="$2"
      shift 2
      ;;
    --simulate-scenario)
      SIM_AUTONOMY_SCENARIO="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        SERVER_ARGS+=("$1")
        shift
      done
      ;;
    *)
      SERVER_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -d "$VENV_DIR" ]]; then
  run python3 -m venv "$VENV_DIR"
  INSTALL_DEPS=1
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

if [[ "$INSTALL_DEPS" -eq 1 ]]; then
  run python -m pip install --upgrade pip
  run python -m pip install -r "$VIS_DIR/requirements.txt"
fi

SIM_PID=""
cleanup() {
  if [[ -n "$SIM_PID" ]]; then
    kill "$SIM_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "$SIMULATE" -eq 1 ]]; then
  udp_port=9870
  for ((i = 0; i < ${#SERVER_ARGS[@]}; i++)); do
    if [[ "${SERVER_ARGS[$i]}" == "--udp-port" && $((i + 1)) -lt ${#SERVER_ARGS[@]} ]]; then
      udp_port="${SERVER_ARGS[$((i + 1))]}"
      break
    fi
  done

  run_in_dir "$VIS_DIR" python simulate_telemetry.py --host 127.0.0.1 --port "$udp_port" --hz "$SIM_HZ" --autonomy-scenario "$SIM_AUTONOMY_SCENARIO" &
  SIM_PID="$!"
fi

run_in_dir "$VIS_DIR" python server.py "${SERVER_ARGS[@]}"
