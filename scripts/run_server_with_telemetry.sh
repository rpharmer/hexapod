#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SERVER_BIN="$SERVER_DIR/build/hexapod-server"

MODE="serial"
CONFIG_PATH=""
TELEMETRY_HOST="127.0.0.1"
TELEMETRY_PORT=9870
SKIP_BUILD=0
SCENARIO=""
SERVER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_server_with_telemetry.sh [options] [-- <extra server args>]

Runs hexapod-server with visualiser telemetry explicitly enabled.
Supports both serial-connected hardware mode and simulator mode.

Options:
  --mode <serial|sim>      Runtime mode intent (default: serial).
  --config <path>          Config path override (default: config.txt for serial, config.sim.txt for sim).
  --telemetry-host <host>  Destination host/IP for UDP telemetry (default: 127.0.0.1).
  --telemetry-port <port>  Destination UDP port for telemetry (default: 9870).
  --scenario <path>        Optional scenario file (relative to hexapod-server/ or absolute).
  --skip-build             Skip CMake configure/build.
  -h, --help               Show this help text.

Examples:
  scripts/run_server_with_telemetry.sh --mode serial --telemetry-host 192.168.1.50
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
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

case "$MODE" in
  serial)
    :
    ;;
  sim)
    :
    ;;
  *)
    echo "Invalid --mode '$MODE' (expected serial or sim)" >&2
    exit 1
    ;;
esac

if [[ -z "$CONFIG_PATH" ]]; then
  if [[ "$MODE" == "sim" ]]; then
    CONFIG_PATH="$SERVER_DIR/config.sim.txt"
  else
    CONFIG_PATH="$SERVER_DIR/config.txt"
  fi
elif [[ "$CONFIG_PATH" != /* ]]; then
  CONFIG_PATH="$ROOT_DIR/$CONFIG_PATH"
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "Config file not found: $CONFIG_PATH" >&2
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  (cd "$SERVER_DIR" && cmake --preset default)
  (cd "$SERVER_DIR" && cmake --build --preset default -j)
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  echo "Server binary not found or not executable: $SERVER_BIN" >&2
  exit 1
fi

SCENARIO_ARGS=()
if [[ -n "$SCENARIO" ]]; then
  if [[ "$SCENARIO" = /* ]]; then
    SCENARIO_PATH="$SCENARIO"
  else
    SCENARIO_PATH="$SERVER_DIR/$SCENARIO"
  fi

  if [[ ! -f "$SCENARIO_PATH" ]]; then
    echo "Scenario file not found: $SCENARIO_PATH" >&2
    exit 1
  fi

  SCENARIO_ARGS=(--scenario "${SCENARIO_PATH#$SERVER_DIR/}")
fi

echo "Running hexapod-server mode=$MODE telemetry=${TELEMETRY_HOST}:${TELEMETRY_PORT}"

a=()
a+=("$SERVER_BIN")
a+=(--config "$CONFIG_PATH")
a+=(--telemetry-enable --telemetry-host "$TELEMETRY_HOST" --telemetry-port "$TELEMETRY_PORT")
a+=("${SCENARIO_ARGS[@]}")
a+=("${SERVER_ARGS[@]}")

(
  cd "$SERVER_DIR"
  "${a[@]}"
)
