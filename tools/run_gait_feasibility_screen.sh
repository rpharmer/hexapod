#!/usr/bin/env bash
# Gait-feasibility screen (leftover §3.15 command-side mechanisms). Clean
# HEXAPOD_* subshell per leftovers §8, then one walk-distance (or
# aggressive_governor) process.
#
# Usage: tools/run_gait_feasibility_screen.sh <out-dir> <screen> <arm>
#   screen: sequential | reverse_walk | turn_in_place | forward_walk | aggressive_governor
#         | slow_height | wave_height
#   arm:    baseline
#         | slew<fraction>  e.g. slew0.6 -> HEXAPOD_WALK_SLEW_FRACTION
#         | phase                        -> HEXAPOD_WALK_LOAD_PHASE=1
#         | snapoff                      -> HEXAPOD_NEAR_CAP_SNAP=0
#         | finalclamp                   -> HEXAPOD_WALK_FINAL_SLEW_CLAMP=1
#         | turnhold                     -> HEXAPOD_TURN_INPLACE_HOLD=1
#         | hh<scale>                    -> HEXAPOD_HEIGHT_HOLD_SCALE
#         | selfweight                   -> HEXAPOD_WALK_TEST_SELF_WEIGHT=1
#         | velocitylead                 -> HEXAPOD_WALK_TEST_VELOCITY_LEAD=1 (test bridge only)
#         | velocityleadfiltered         -> same, filtered with existing Kd/Kp timescale
#         | storedmotion                 -> test-only composed PD-error velocity request bound
#         | warmtransport                -> contact-frame warm-start transport experiment
#         | capture                      -> first speed trip + preceding accepted states, never overwritten
#         | retrydamping                 -> explicit speed-retry keeps nominal damping
#         | legacyrecovery               -> diagnostic old retry damping and turn hold both off
#         any '+'-joined combination, e.g. slew0.6+phase, snapoff+finalclamp
set -u -o pipefail

out_dir=${1:?output directory required}
screen=${2:?screen required}
arm=${3:?arm required}
case "$screen" in
  sequential|reverse_walk|turn_in_place|forward_walk|aggressive_governor|slow_height|wave_height) : ;;
  *) echo "unknown screen: $screen" >&2; exit 2 ;;
esac

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

  IFS='+' read -r -a arm_parts <<< "$arm"
  for part in "${arm_parts[@]}"; do
    case "$part" in
      baseline) : ;;
      selfweight) export HEXAPOD_WALK_TEST_SELF_WEIGHT=1 ;;
      velocitylead) export HEXAPOD_WALK_TEST_VELOCITY_LEAD=1 ;;
      velocityleadfiltered) export HEXAPOD_WALK_TEST_VELOCITY_LEAD=filtered ;;
      storedmotion) export HEXAPOD_WALK_TEST_STORED_MOTION=1 ;;
      warmtransport) export HEXAPOD_PINOCCHIO_TRANSPORT_WARM_START=1 ;;
      retrydamping) export HEXAPOD_PINOCCHIO_RETRY_KEEP_DAMPING=1 ;;
      legacyrecovery) export HEXAPOD_PINOCCHIO_RETRY_KEEP_DAMPING=0 HEXAPOD_TURN_INPLACE_HOLD=0 ;;
      capture)
        export HEXAPOD_PINOCCHIO_SPEED_LIMIT_SNAPSHOT_PATH="$out_dir/$screen-$arm-first-speed-trip.json"
        export HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH="$out_dir/$screen-$arm-first-failure-history.json"
        ;;
      phase) export HEXAPOD_WALK_LOAD_PHASE=1 ;;
      snapoff) export HEXAPOD_NEAR_CAP_SNAP=0 ;;
      turnhold) export HEXAPOD_TURN_INPLACE_HOLD=1 ;;
      hh*) export HEXAPOD_HEIGHT_HOLD_SCALE="${part#hh}" ;;
      finalclamp) export HEXAPOD_WALK_FINAL_SLEW_CLAMP=1 ;;
      slew*) export HEXAPOD_WALK_SLEW_FRACTION="${part#slew}" ;;
      *) echo "unknown arm part: $part" >&2; exit 2 ;;
    esac
  done

  # aggressive_governor is a locomotion-regression case, not a walk-distance one.
  if [ "$screen" = aggressive_governor ]; then
    cd hexapod-server
    exec ./build-tests/test_locomotion_regression_suite --case aggressive_governor
  fi
  if [ "$screen" = slow_height ] || [ "$screen" = wave_height ]; then
    cd hexapod-server
    if [ "$screen" = slow_height ]; then
      exec ./build-tests/test_physics_sim_slow_fwd_walk_foot_clearance --emit-metrics-json
    fi
    exec ./build-tests/test_physics_sim_wave_slow_walk_foot_clearance --emit-metrics-json
  fi
  case "$screen" in
    sequential) : ;;
    *) export HEXAPOD_WALK_TEST_CASE="$screen" ;;
  esac
  cd hexapod-server
  ./build-tests/test_physics_sim_walk_distance --emit-metrics-json
)
