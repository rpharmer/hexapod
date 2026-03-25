#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SERVER_BIN="$SERVER_DIR/build/hexapod-server"
SIM_CONFIG="$SERVER_DIR/config.sim.txt"
ACTIVE_CONFIG="$SERVER_DIR/config.txt"
SCENARIO_GLOB="$SERVER_DIR/scenarios/*.toml"
ONLY_SCENARIO=""
SKIP_BUILD=0

usage() {
  cat <<'USAGE'
Usage: scripts/run_server_scenarios.sh [options]

Builds and runs hexapod-server scenarios in simulator mode.

Options:
  --scenario <path>        Run a single scenario (relative to hexapod-server/ or absolute path).
  --skip-build             Assume build/hexapod-server already exists.
  -h, --help               Show this help text.

Examples:
  scripts/run_server_scenarios.sh
  scripts/run_server_scenarios.sh --scenario scenarios/02_command_timeout_fallback.toml
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      ONLY_SCENARIO="$2"
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
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "$SIM_CONFIG" ]]; then
  echo "Missing simulator config: $SIM_CONFIG" >&2
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  (cd "$SERVER_DIR" && cmake --preset default)
  (cd "$SERVER_DIR" && cmake --build --preset default -j)
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  echo "Server binary not found/executable: $SERVER_BIN" >&2
  exit 1
fi

restore_mode="none"
backup_file="$(mktemp)"
if [[ -f "$ACTIVE_CONFIG" ]]; then
  cp "$ACTIVE_CONFIG" "$backup_file"
  restore_mode="restore"
else
  restore_mode="remove"
fi

cleanup_config() {
  if [[ "$restore_mode" == "restore" ]]; then
    cp "$backup_file" "$ACTIVE_CONFIG"
  elif [[ "$restore_mode" == "remove" ]]; then
    rm -f "$ACTIVE_CONFIG"
  fi
  rm -f "$backup_file"
}
trap cleanup_config EXIT

cp "$SIM_CONFIG" "$ACTIVE_CONFIG"

run_one() {
  local scenario_path="$1"
  echo "=== Running ${scenario_path#$SERVER_DIR/} ==="
  (cd "$SERVER_DIR" && "$SERVER_BIN" --scenario "${scenario_path#$SERVER_DIR/}")
}

if [[ -n "$ONLY_SCENARIO" ]]; then
  if [[ "$ONLY_SCENARIO" = /* ]]; then
    target="$ONLY_SCENARIO"
  else
    target="$SERVER_DIR/$ONLY_SCENARIO"
  fi

  if [[ ! -f "$target" ]]; then
    echo "Scenario not found: $target" >&2
    exit 1
  fi

  run_one "$target"
else
  shopt -s nullglob
  scenarios=($SCENARIO_GLOB)
  shopt -u nullglob

  if [[ ${#scenarios[@]} -eq 0 ]]; then
    echo "No scenarios found matching $SCENARIO_GLOB" >&2
    exit 1
  fi

  for scenario in "${scenarios[@]}"; do
    run_one "$scenario"
  done
fi
