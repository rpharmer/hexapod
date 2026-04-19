#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
VIS_SCRIPT="$ROOT_DIR/scripts/run_visualiser.sh"
SERVER_SCRIPT="$ROOT_DIR/scripts/run_server_with_telemetry.sh"
SERVER_DIR="$ROOT_DIR/hexapod-server"

UDP_PORT=9870
VIS_HOST="127.0.0.1"
SKIP_BUILD=0
SCENARIO=""
SERVER_MODE="sim"
SERVER_CONFIG=""
SERVER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_sim_stack.sh [options] [-- <extra server args>]

Launches hexapod-opengl-visualiser and hexapod-server together,
with server telemetry streamed to the visualiser over UDP.

Options:
  --udp-port <port>        Visualiser UDP ingest port (default: 9870).
  --visualiser-host <host> Telemetry destination host (default: 127.0.0.1).
  --server-mode <mode>     Server mode: sim or serial (default: sim).
  --server-config <path>   Server config override.
  --skip-build             Skip server configure/build.
  --scenario <path>        Optional scenario to run (relative to hexapod-server/ or absolute).
  -h, --help               Show this help text.

Examples:
  scripts/run_sim_stack.sh --scenario scenarios/01_nominal_stand_walk.toml
  scripts/run_sim_stack.sh --server-mode serial --visualiser-host 192.168.1.50
  scripts/run_sim_stack.sh -- --console-only
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --udp-port)
      UDP_PORT="$2"
      shift 2
      ;;
    --visualiser-host)
      VIS_HOST="$2"
      shift 2
      ;;
    --server-mode)
      SERVER_MODE="$2"
      shift 2
      ;;
    --server-config)
      SERVER_CONFIG="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --scenario)
      SCENARIO="$2"
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
      msg_error "Unknown argument: $1"
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "$VIS_SCRIPT" ]]; then
  msg_error "Visualiser launcher missing or not executable: $VIS_SCRIPT"
  exit 1
fi

if [[ ! -x "$SERVER_SCRIPT" ]]; then
  msg_error "Server launcher missing or not executable: $SERVER_SCRIPT"
  exit 1
fi

case "$SERVER_MODE" in
  sim|serial) ;;
  *)
    msg_error "Invalid --server-mode '$SERVER_MODE' (expected sim or serial)"
    exit 1
    ;;
esac

if [[ -n "$SCENARIO" ]]; then
  SCENARIO_PATH="$(resolve_server_path "$SCENARIO")"

  if [[ ! -f "$SCENARIO_PATH" ]]; then
    msg_error "Scenario file not found: $SCENARIO_PATH"
    exit 1
  fi
  SCENARIO_ARG=(--scenario "${SCENARIO_PATH#$SERVER_DIR/}")
else
  SCENARIO_ARG=()
fi

VIS_CMD=("$VIS_SCRIPT" -- --udp-port "$UDP_PORT")
"${VIS_CMD[@]}" &
VIS_PID="$!"

sleep 0.5
if ! kill -0 "$VIS_PID" >/dev/null 2>&1; then
  msg_error "Visualiser exited during startup. Check logs above (likely port conflict or dependency/runtime failure)."
  exit 1
fi

cleanup() {
  kill "$VIS_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Visualiser running on UDP port ${UDP_PORT}"
echo "Starting server mode=${SERVER_MODE} with telemetry -> ${VIS_HOST}:${UDP_PORT}"

SERVER_CMD=("$SERVER_SCRIPT" --mode "$SERVER_MODE" --telemetry-host "$VIS_HOST" --telemetry-port "$UDP_PORT")
if [[ "$SKIP_BUILD" -eq 1 ]]; then
  SERVER_CMD+=(--skip-build)
fi
if [[ -n "$SERVER_CONFIG" ]]; then
  SERVER_CMD+=(--config "$SERVER_CONFIG")
fi
if [[ ${#SCENARIO_ARG[@]} -gt 0 ]]; then
  SERVER_CMD+=("${SCENARIO_ARG[@]}")
fi
SERVER_CMD+=(-- "${SERVER_ARGS[@]}")

"${SERVER_CMD[@]}"
