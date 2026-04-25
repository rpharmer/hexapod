#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SIM_DIR="$ROOT_DIR/hexapod-physics-sim"
RUNNER_BIN="$SERVER_DIR/build/hexapod-tripod-support-baseline"
SIM_BIN="$SIM_DIR/build/hexapod-physics-sim"
SKIP_BUILD=0
STAND_MS=3000
TRIPOD_MS=15000
INDEFINITE=0

usage() {
  cat <<'USAGE'
Usage: scripts/run_tripod_support_baseline.sh [options]

Launch the live physics-sim tripod-support baseline:
  1. Hold a nominal six-leg stand
  2. Raise legs 1/3/5 and hold tripod support

This is the manual baseline for "can the robot statically hold tripod support?"
It is not a TOML scenario because scenario TOMLs cannot directly command per-leg lifted poses.

Options:
  --stand-ms <ms>      Hold six-leg stand for this long before tripod support (default: 3000)
  --tripod-ms <ms>     Hold tripod support for this long (default: 15000)
  --indefinite         Hold tripod support until Ctrl-C
  --skip-build         Skip build steps
  -h, --help           Show this help text
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stand-ms)
      STAND_MS="$2"
      shift 2
      ;;
    --tripod-ms)
      TRIPOD_MS="$2"
      shift 2
      ;;
    --indefinite)
      INDEFINITE=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      msg_error "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  section "Build Physics Sim"
  run_in_dir "$SIM_DIR" cmake -S . -B build
  run_in_dir "$SIM_DIR" cmake --build build -j 2

  section "Build Baseline Runner"
  run_in_dir "$SERVER_DIR" cmake --preset default
  run_in_dir "$SERVER_DIR" cmake --build --preset default --target hexapod-tripod-support-baseline -j 2
fi

if [[ ! -x "$SIM_BIN" ]]; then
  msg_error "Physics sim binary not found at $SIM_BIN"
  exit 1
fi

if [[ ! -x "$RUNNER_BIN" ]]; then
  msg_error "Baseline runner not found at $RUNNER_BIN"
  exit 1
fi

section "Tripod Support Baseline"
CMD=("$RUNNER_BIN" "--sim-exe" "$SIM_BIN" "--stand-ms" "$STAND_MS")
if [[ "$INDEFINITE" -eq 1 ]]; then
  CMD+=("--indefinite")
else
  CMD+=("--tripod-ms" "$TRIPOD_MS")
fi
run "${CMD[@]}"
