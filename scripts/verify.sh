#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

section() {
  local title="$1"
  printf '\n========== %s =========='"\n" "$title"
}

run() {
  printf '+ %s\n' "$*"
  "$@"
}

run_in_dir() {
  local dir="$1"
  shift
  printf '+ (cd %s && %s)\n' "$dir" "$*"
  (
    cd "$dir"
    "$@"
  )
}

run_server_tests() {
  section "Server tests (hexapod-server / preset: tests)"
  run_in_dir "$ROOT_DIR/hexapod-server" cmake --preset tests
  run_in_dir "$ROOT_DIR/hexapod-server" cmake --build --preset tests -j
  run_in_dir "$ROOT_DIR/hexapod-server" ctest --preset tests --output-on-failure
}

run_client_host_tests() {
  section "Firmware host tests (hexapod-client / preset: host-tests)"
  run_in_dir "$ROOT_DIR/hexapod-client" cmake --preset host-tests
  run_in_dir "$ROOT_DIR/hexapod-client" cmake --build --preset host-tests -j
  run_in_dir "$ROOT_DIR/hexapod-client" ctest --preset host-tests --output-on-failure
}

run_server_smoke_scenario() {
  section "Scenario smoke (hexapod-server)"

  local server_dir="$ROOT_DIR/hexapod-server"
  local scenario="$server_dir/scenarios/01_nominal_stand_walk.toml"
  local sim_config="$server_dir/config.sim.txt"
  local active_config="$server_dir/config.txt"
  local server_bin_tests="$server_dir/build-tests/hexapod-server"

  if [[ ! -f "$scenario" || ! -f "$sim_config" ]]; then
    echo "Skipping scenario smoke: required files not found."
    return 0
  fi

  if [[ ! -x "$server_bin_tests" ]]; then
    run_in_dir "$server_dir" cmake --build --preset tests --target hexapod-server -j
  fi

  local restore_mode="none"
  local backup_file
  backup_file="$(mktemp)"

  if [[ -f "$active_config" ]]; then
    cp "$active_config" "$backup_file"
    restore_mode="restore"
  else
    restore_mode="remove"
  fi

  cleanup_config() {
    if [[ "$restore_mode" == "restore" ]]; then
      cp "$backup_file" "$active_config"
    elif [[ "$restore_mode" == "remove" ]]; then
      rm -f "$active_config"
    fi
    rm -f "$backup_file"
  }
  trap cleanup_config EXIT

  cp "$sim_config" "$active_config"
  run_in_dir "$server_dir" "$server_bin_tests" --scenario "scenarios/01_nominal_stand_walk.toml"

  trap - EXIT
  cleanup_config
}

main() {
  section "Root quality gates"
  echo "Repository root: $ROOT_DIR"

  run_server_tests
  run_client_host_tests
  run_server_smoke_scenario

  section "Verification complete"
  echo "All default quality gates passed."
}

main "$@"
