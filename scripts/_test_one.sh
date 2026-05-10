#!/usr/bin/env bash
B=/home/volly/pico/hexapod/hexapod-physics-sim/build-articulation-hardening
TEST="$B/test_servo_stall_under_overload"
echo "Binary: $TEST"
ls -la "$TEST"
echo "--- run ---"
"$TEST"
rc=$?
echo "rc=$rc"
echo "--- via if/then ---"
if "$TEST" >/tmp/o 2>/tmp/e; then
  echo "if-branch: PASS"
else
  echo "if-branch: FAIL with rc=$?"
fi
echo "stderr was:"
cat /tmp/e
