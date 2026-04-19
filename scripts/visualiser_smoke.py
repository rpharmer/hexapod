#!/usr/bin/env python3
"""Deprecated smoke helper for the retired browser visualiser.

The browser/WebSocket visualiser has been replaced by `hexapod-opengl-visualiser`.
Use `scripts/run_visualiser.sh` or `scripts/run_sim_stack.sh` instead.
"""

from __future__ import annotations

import sys


def main() -> int:
    print(
        "ERROR: scripts/visualiser_smoke.py is retired. "
        "Use scripts/run_visualiser.sh or scripts/run_sim_stack.sh with "
        "hexapod-opengl-visualiser.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
