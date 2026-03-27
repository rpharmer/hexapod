#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
SERVER_DIR="$ROOT_DIR/hexapod-server"
PROFILE="all"

usage() {
  cat <<'USAGE'
Usage: scripts/run_acceptance_gates.sh [--profile fast-sim|extended-hil|all]

Profiles:
  fast-sim      Runs quick simulation-oriented autonomy checks.
  extended-hil  Runs hardware-targeted acceptance checks and emits HIL artifacts.
  all           Runs both profiles (default).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      msg_error "Unknown argument: $1"
      usage >&2
      exit 1
      ;;
  esac
done

if [[ "$PROFILE" != "fast-sim" && "$PROFILE" != "extended-hil" && "$PROFILE" != "all" ]]; then
  msg_error "Invalid profile: $PROFILE"
  usage >&2
  exit 1
fi

run_in_dir "$SERVER_DIR" cmake --preset tests
run_in_dir "$SERVER_DIR" cmake --build --preset tests -j

if [[ "$PROFILE" == "fast-sim" || "$PROFILE" == "all" ]]; then
  section "Fast simulation checks"
  run_in_dir "$SERVER_DIR" ctest --preset tests --output-on-failure -L fast_sim
fi

if [[ "$PROFILE" == "extended-hil" || "$PROFILE" == "all" ]]; then
  section "Extended HIL acceptance"
  local_artifact_dir="$SERVER_DIR/build-tests/hil-artifacts"
  mkdir -p "$local_artifact_dir"
  HEXAPOD_HIL_ARTIFACT_DIR="$local_artifact_dir" \
    run_in_dir "$SERVER_DIR" ctest --preset tests --output-on-failure -L extended_hil
  echo "HIL artifacts written under: $local_artifact_dir"
fi
