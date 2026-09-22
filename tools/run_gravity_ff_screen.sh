#!/usr/bin/env bash
# Gravity-FF A/B screen. Clean HEXAPOD_* subshell per leftovers §8, then one
# walk-distance (or aggressive_governor) process. Candidate is opt-in via
# HEXAPOD_WALK_TEST_GRAVITY_FF.
#
# Usage: tools/run_gravity_ff_screen.sh <out-dir> <screen> <baseline|candidate>
#   screen: sequential | reverse_walk | turn_in_place | aggressive_governor
set -u -o pipefail

out_dir=$1
screen=$2
arm=$3

mkdir -p "$out_dir"
(
  for name in $(compgen -v HEXAPOD_); do unset "$name"; done
  cd /home/volly/pico/hexapod
  source scripts/lib/pinocchio_env.sh
  export HEXAPOD_PHYSICS_SIM_EXE=/home/volly/pico/hexapod/hexapod-physics-sim/build/hexapod-physics-sim
  export HEXAPOD_WALK_TEST_SOLVER_MODE=pinocchio-proximal
  export HEXAPOD_WALK_TEST_SOLVER_ITERATIONS=24
  export HEXAPOD_WALK_TEST_BODY_HEIGHT_M=0.14
  export HEXAPOD_WALK_TEST_CHILD_STDIO=1
  export HEXAPOD_PROXIMAL_TRACE_SPEED_LIMIT=1
  export HEXAPOD_PROXIMAL_TRACE_FAILURES=1
  if [ "$arm" = candidate ]; then
    export HEXAPOD_WALK_TEST_GRAVITY_FF=1
  fi
  # aggressive_governor is a locomotion-regression case, not a walk-distance one.
  if [ "$screen" = aggressive_governor ]; then
    cd hexapod-server
    exec ./build-tests/test_locomotion_regression_suite --case aggressive_governor
  fi
  case "$screen" in
    sequential) : ;;
    *) export HEXAPOD_WALK_TEST_CASE="$screen" ;;
  esac
  cd hexapod-server
  ./build-tests/test_physics_sim_walk_distance --emit-metrics-json
)
