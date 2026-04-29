#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
BUILD_JOBS="${BUILD_JOBS:-2}"

run_server_tests() {
  section "Server tests (hexapod-server / preset: tests)"
  run_in_dir "$ROOT_DIR/hexapod-server" cmake --preset tests
  run_in_dir "$ROOT_DIR/hexapod-server" cmake --build --preset tests -j "$BUILD_JOBS"
  run_in_dir "$ROOT_DIR/hexapod-server" ctest --preset tests -LE locomotion-stress --output-on-failure
}

run_physics_sim_build() {
  section "Physics sim build (hexapod-physics-sim)"
  run_in_dir "$ROOT_DIR/hexapod-physics-sim" cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
  run_in_dir "$ROOT_DIR/hexapod-physics-sim" cmake --build build -j "$BUILD_JOBS"
}

run_client_host_tests() {
  section "Firmware host tests (hexapod-client / preset: host-tests)"
  run_in_dir "$ROOT_DIR/hexapod-client" cmake --preset host-tests
  run_in_dir "$ROOT_DIR/hexapod-client" cmake --build --preset host-tests -j "$BUILD_JOBS"
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
    msg_skip "Scenario smoke: required files not found."
    return 0
  fi

  if [[ ! -x "$server_bin_tests" ]]; then
    run_in_dir "$server_dir" cmake --build --preset tests --target hexapod-server -j "$BUILD_JOBS"
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

  run_physics_sim_build
  run_server_tests
  run_client_host_tests
  run_server_smoke_scenario

  section "Verification complete"
  echo "All default quality gates passed."
}

main "$@"
