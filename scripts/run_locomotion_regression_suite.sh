#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
BUILD_JOBS="${BUILD_JOBS:-2}"

section "Locomotion regression suite"

run_in_dir "$ROOT_DIR/hexapod-physics-sim" cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
run_in_dir "$ROOT_DIR/hexapod-physics-sim" cmake --build build -j "$BUILD_JOBS"

run_in_dir "$ROOT_DIR/hexapod-server" cmake --preset tests
run_in_dir "$ROOT_DIR/hexapod-server" cmake --build --preset tests -j "$BUILD_JOBS"
run_in_dir "$ROOT_DIR/hexapod-server" ctest --preset tests -R 'locomotion_regression_suite(_stress)?$' --output-on-failure
