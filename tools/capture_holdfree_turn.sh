#!/usr/bin/env bash
# Capture scored-turn donors on the hold-free command plant (leftover §3.17
# slew cap). One sequential process per invocation; dumps are never-overwrite,
# so re-run until both the fail and the pass donor freeze.
#
# Usage: tools/capture_holdfree_turn.sh <log-dir> <fail|pass> [slew-fraction]
set -u -o pipefail

log_dir=$1
side=$2
fraction=${3:-0.6}
snap=/home/volly/pico/hexapod/docs/contact-snapshots

mkdir -p "$log_dir"
(
  for name in $(compgen -v HEXAPOD_); do unset "$name"; done
  cd /home/volly/pico/hexapod
  source scripts/lib/pinocchio_env.sh
  export HEXAPOD_PHYSICS_SIM_EXE=/home/volly/pico/hexapod/hexapod-physics-sim/build/hexapod-physics-sim
  export HEXAPOD_WALK_TEST_SOLVER_MODE=pinocchio-proximal
  export HEXAPOD_WALK_TEST_SOLVER_ITERATIONS=24
  export HEXAPOD_WALK_TEST_BODY_HEIGHT_M=0.14
  export HEXAPOD_WALK_TEST_CHILD_STDIO=1
  export HEXAPOD_WALK_SLEW_FRACTION="$fraction"

  export HEXAPOD_TURN_ENTRY_DUMP_PATH="$snap/holdfree-turn-entry-$side-v1.json"
  export HEXAPOD_TURN_TRAJ_DUMP_PATH="$snap/holdfree-turn-traj-$side-v1.json"
  if [ "$side" = fail ]; then
    # Scored fail is net above the 0.21 m gate.
    export HEXAPOD_TURN_ENTRY_DUMP_MIN_NET_M=0.21
    export HEXAPOD_TURN_TRAJ_DUMP_MIN_NET_M=0.21
  else
    export HEXAPOD_TURN_ENTRY_DUMP_MAX_NET_M=0.21
    export HEXAPOD_TURN_TRAJ_DUMP_MAX_NET_M=0.21
  fi

  cd hexapod-server
  ./build-tests/test_physics_sim_walk_distance --emit-metrics-json
)
