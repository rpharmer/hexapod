#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec python3 "$ROOT_DIR/scripts/scenario_regression_harness.py" \
  --scenario scenarios/05_long_walk_observability.toml \
  "$@"
