#!/usr/bin/env bash
B=/home/volly/pico/hexapod/hexapod-physics-sim/build-articulation-hardening
TESTS=(
  test_world_step_determinism
  test_zero_gravity_energy_conservation
  test_servo_classical_pd_invariant
  test_single_leg_pd_oscillation_sweep
  test_single_leg_pd_response
  test_hexapod_stand_quiescence
  test_hexapod_substep_convergence
)
for t in "${TESTS[@]}"; do
  echo "=== $t ==="
  "$B/$t" >/tmp/o 2>/tmp/e
  rc=$?
  echo "rc=$rc"
  cat /tmp/e
  echo
done
