#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
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
      msg_error "Unknown argument: $1"
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "$SIM_CONFIG" ]]; then
  msg_error "Missing simulator config: $SIM_CONFIG"
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  run_in_dir "$SERVER_DIR" cmake --preset default
  run_in_dir "$SERVER_DIR" cmake --build --preset default -j
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  msg_error "Server binary not found/executable: $SERVER_BIN"
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
  run_in_dir "$SERVER_DIR" "$SERVER_BIN" --scenario "${scenario_path#$SERVER_DIR/}"
}

if [[ -n "$ONLY_SCENARIO" ]]; then
  target="$(resolve_server_path "$ONLY_SCENARIO")"

  if [[ ! -f "$target" ]]; then
    msg_error "Scenario not found: $target"
    exit 1
  fi

  run_one "$target"
else
  shopt -s nullglob
  scenarios=($SCENARIO_GLOB)
  shopt -u nullglob

  if [[ ${#scenarios[@]} -eq 0 ]]; then
    msg_error "No scenarios found matching $SCENARIO_GLOB"
    exit 1
  fi

  for scenario in "${scenarios[@]}"; do
    run_one "$scenario"
  done
fi
