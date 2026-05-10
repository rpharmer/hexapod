#!/usr/bin/env bash
B=/home/volly/pico/hexapod/hexapod-physics-sim/build-articulation-hardening
TESTS=(
  test_servo_chain_stability
  test_servo_stability_regression
  test_servo_torque_saturation_matches_inertia
  test_servo_stall_under_overload
  test_servo_chain_vertical_lift_under_gravity
  test_hexapod_zero_g
  test_hexapod_freefall_no_support
  test_servo_joint_target
  test_servo_regression_matrix
  test_hexapod_geometry_alignment
  test_hexapod_planted_foot_drift
  test_hexapod_live
  test_hexapod_live_pose_hold
  test_hexapod_pose_hold_with_payload
  test_constraint_solver_extreme_mass_ratio_stress
  test_articulation_impulse_response_decay
)
PASS_COUNT=0
FAIL_COUNT=0
FAILURES=()
for t in "${TESTS[@]}"; do
  printf "%-60s " "$t"
  if "$B/$t" >/tmp/o 2>/tmp/e; then
    echo PASS
    PASS_COUNT=$((PASS_COUNT+1))
  else
    echo FAIL
    head -3 /tmp/e
    FAIL_COUNT=$((FAIL_COUNT+1))
    FAILURES+=("$t")
  fi
done
echo
echo "RESULTS: $PASS_COUNT passed, $FAIL_COUNT failed"
if [ ${#FAILURES[@]} -gt 0 ]; then
  echo "Failed tests: ${FAILURES[*]}"
fi
