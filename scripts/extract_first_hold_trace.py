#!/usr/bin/env python3
"""Keep the first Pinocchio held step and its immediately preceding speed trace.

Drains all child output so a failed live regression can finish and keep its
normal replay bundle, without flooding the terminal during a hold cascade.
"""

from collections import deque
import sys


history = deque(maxlen=16)
captured = False
remaining_contact_lines = 0

for line in sys.stdin:
    line = line.rstrip("\n")
    if line.startswith("[proximal-held]") and not captured:
        print("first_hold_context:")
        for previous in history:
            print(previous)
        print(line)
        captured = True
        remaining_contact_lines = 3
        continue
    if remaining_contact_lines and line.startswith(
        ("[proximal-held-winner-joints]", "[proximal-held-contacts]")
    ):
        print(line)
        remaining_contact_lines -= 1
        continue
    if not captured and line.startswith(
        (
            "[proximal-speed-limit]",
            "[proximal-speed-limit-joints]",
            "[proximal-integrated-speed-limit]",
            "[proximal-first-failure]",
        )
    ):
        history.append(line)
    if line.startswith(
        (
            "long_walk_aggressive_diagnostic passed=",
            "long_walk_aggressive_diagnostic recover_census",
            "long_walk_observability passed=",
            "long_walk_observability recover_census",
        )
    ) or line.startswith('{"suite":"locomotion_regression"'):
        print(line)
