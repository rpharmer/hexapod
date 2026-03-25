#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
VIS_SCRIPT="$ROOT_DIR/scripts/run_visualiser.sh"
SERVER_SCRIPT="$ROOT_DIR/scripts/run_server_with_telemetry.sh"
SERVER_DIR="$ROOT_DIR/hexapod-server"

HTTP_PORT=8080
UDP_PORT=9870
VIS_HOST="127.0.0.1"
INSTALL_DEPS=0
SKIP_BUILD=0
KILL_CONFLICTING=1
SCENARIO=""
SERVER_MODE="sim"
SERVER_CONFIG=""
SERVER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_sim_stack.sh [options] [-- <extra server args>]

Launches hexapod-visualiser and hexapod-server together,
with server telemetry streamed to the visualiser over UDP.

Options:
  --http-port <port>       Visualiser HTTP/WebSocket port (default: 8080).
  --udp-port <port>        Visualiser UDP ingest port (default: 9870).
  --visualiser-host <host> Telemetry destination host (default: 127.0.0.1).
  --server-mode <mode>     Server mode: sim or serial (default: sim).
  --server-config <path>   Server config override.
  --install-deps           Force visualiser Python dependency install.
  --no-kill-conflicting    Do not auto-kill processes already bound to selected HTTP/UDP ports.
  --skip-build             Skip server configure/build.
  --scenario <path>        Optional scenario to run (relative to hexapod-server/ or absolute).
  -h, --help               Show this help text.

Examples:
  scripts/run_sim_stack.sh --install-deps
  scripts/run_sim_stack.sh --scenario scenarios/01_nominal_stand_walk.toml
  scripts/run_sim_stack.sh --server-mode serial --visualiser-host 192.168.1.50
  scripts/run_sim_stack.sh -- --console-only
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --http-port)
      HTTP_PORT="$2"
      shift 2
      ;;
    --udp-port)
      UDP_PORT="$2"
      shift 2
      ;;
    --visualiser-host)
      VIS_HOST="$2"
      shift 2
      ;;
    --install-deps)
      INSTALL_DEPS=1
      shift
      ;;
    --no-kill-conflicting)
      KILL_CONFLICTING=0
      shift
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

kill_port_owners() {
  local port="$1"
  local proto="$2"
  local -a pids=()

  if command -v lsof >/dev/null 2>&1; then
    while IFS= read -r pid; do
      [[ -n "$pid" ]] && pids+=("$pid")
    done < <(lsof -t -i"${proto}:${port}" 2>/dev/null | sort -u)
  fi

  if [[ ${#pids[@]} -eq 0 ]] && command -v ss >/dev/null 2>&1; then
    local ss_flag="-lunp"
    if [[ "$proto" == "tcp" ]]; then
      ss_flag="-ltnp"
    fi
    while IFS= read -r pid; do
      [[ -n "$pid" ]] && pids+=("$pid")
    done < <(
      ss "$ss_flag" "sport = :${port}" 2>/dev/null \
        | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' \
        | sort -u
    )
  fi

  if [[ ${#pids[@]} -eq 0 ]]; then
    return 0
  fi

  msg_warn "Found ${#pids[@]} process(es) bound to ${proto^^} port ${port}; terminating: ${pids[*]}"
  kill "${pids[@]}" >/dev/null 2>&1 || true
  sleep 0.3
  kill -0 "${pids[@]}" >/dev/null 2>&1 && kill -9 "${pids[@]}" >/dev/null 2>&1 || true
}

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

if [[ "$KILL_CONFLICTING" -eq 1 ]]; then
  kill_port_owners "$HTTP_PORT" tcp
  kill_port_owners "$UDP_PORT" udp
fi

VIS_CMD=("$VIS_SCRIPT")
if [[ "$INSTALL_DEPS" -eq 1 ]]; then
  VIS_CMD+=(--install-deps)
fi
VIS_CMD+=(-- --http-port "$HTTP_PORT" --udp-port "$UDP_PORT")

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

echo "Visualiser running at http://${VIS_HOST}:${HTTP_PORT}"
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
