#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SERVER_BIN="$SERVER_DIR/build/hexapod-server"

MODE="serial"
CONFIG_PATH=""
TELEMETRY_HOST="127.0.0.1"
TELEMETRY_PORT=9870
COMMAND_ENABLE=0
COMMAND_HOST=""
COMMAND_PORT=9872
COMMAND_SCENARIOS_DIR="scenarios"
SKIP_BUILD=0
SCENARIO=""
SERVER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_server_with_telemetry.sh [options] [-- <extra server args>]

Runs hexapod-server with OpenGL visualiser telemetry explicitly enabled.
Supports both serial-connected hardware mode and simulator mode.

Options:
  --mode <serial|sim>      Runtime mode intent (default: serial).
  --config <path>          Config path override (default: config.txt for serial, config.sim.txt for sim).
  --telemetry-host <host>  Destination host/IP for UDP telemetry (default: 127.0.0.1).
  --telemetry-port <port>  Destination UDP port for telemetry (default: 9870).
  --command-enable         Enable visualiser command UDP listen (interactive mode).
  --command-disable        Disable command channel (default for this script).
  --command-host <host>    Bind host for command UDP (default: telemetry host).
  --command-port <port>    Command UDP port (default: 9872).
  --command-scenarios-dir <d>  Scenario dir relative to hexapod-server/ (default: scenarios).
  --scenario <path>        Optional scenario file (relative to hexapod-server/ or absolute).
  --skip-build             Skip CMake configure/build.
  -h, --help               Show this help text.

Examples:
  scripts/run_server_with_telemetry.sh --mode serial --telemetry-host 192.168.1.50
  scripts/run_server_with_telemetry.sh --mode sim --command-enable
  scripts/run_server_with_telemetry.sh --mode sim --scenario scenarios/01_nominal_stand_walk.toml
  scripts/run_server_with_telemetry.sh --config hexapod-server/config.txt -- --console-only
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --telemetry-host)
      TELEMETRY_HOST="$2"
      shift 2
      ;;
    --telemetry-port)
      TELEMETRY_PORT="$2"
      shift 2
      ;;
    --command-enable)
      COMMAND_ENABLE=1
      shift
      ;;
    --command-disable)
      COMMAND_ENABLE=0
      shift
      ;;
    --command-host)
      COMMAND_HOST="$2"
      shift 2
      ;;
    --command-port)
      COMMAND_PORT="$2"
      shift 2
      ;;
    --command-scenarios-dir)
      COMMAND_SCENARIOS_DIR="$2"
      shift 2
      ;;
    --scenario)
      SCENARIO="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
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

case "$MODE" in
  serial|sim) ;;
  *)
    msg_error "Invalid --mode '$MODE' (expected serial or sim)"
    exit 1
    ;;
esac

if [[ -z "$CONFIG_PATH" ]]; then
  if [[ "$MODE" == "sim" ]]; then
    CONFIG_PATH="$SERVER_DIR/config.sim.txt"
  else
    CONFIG_PATH="$SERVER_DIR/config.txt"
  fi
else
  CONFIG_PATH="$(resolve_from_root "$CONFIG_PATH")"
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  msg_error "Config file not found: $CONFIG_PATH"
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  run_in_dir "$SERVER_DIR" cmake --preset default
  run_in_dir "$SERVER_DIR" cmake --build --preset default -j
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  msg_error "Server binary not found or not executable: $SERVER_BIN"
  exit 1
fi

SCENARIO_ARGS=()
if [[ -n "$SCENARIO" ]]; then
  SCENARIO_PATH="$(resolve_server_path "$SCENARIO")"

  if [[ ! -f "$SCENARIO_PATH" ]]; then
    msg_error "Scenario file not found: $SCENARIO_PATH"
    exit 1
  fi

  SCENARIO_ARGS=(--scenario "${SCENARIO_PATH#$SERVER_DIR/}")
fi

echo "Running hexapod-server mode=$MODE telemetry=${TELEMETRY_HOST}:${TELEMETRY_PORT}"

a=()
a+=("$SERVER_BIN")
a+=(--config "$CONFIG_PATH")
a+=(--telemetry-enable --telemetry-host "$TELEMETRY_HOST" --telemetry-port "$TELEMETRY_PORT")
if [[ "$COMMAND_ENABLE" -eq 1 && ${#SCENARIO_ARGS[@]} -eq 0 ]]; then
  if [[ -z "$COMMAND_HOST" ]]; then
    COMMAND_HOST="$TELEMETRY_HOST"
  fi
  a+=(--command-enable --command-host "$COMMAND_HOST" --command-port "$COMMAND_PORT")
  a+=(--command-scenarios-dir "$COMMAND_SCENARIOS_DIR")
  echo "Command channel enabled on ${COMMAND_HOST}:${COMMAND_PORT}"
fi
a+=("${SCENARIO_ARGS[@]}")
a+=("${SERVER_ARGS[@]}")

run_in_dir "$SERVER_DIR" "${a[@]}"
