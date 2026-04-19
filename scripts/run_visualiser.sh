#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
VIS_DIR="$ROOT_DIR/hexapod-opengl-visualiser"
VIS_BUILD_DIR="$VIS_DIR/build"
VIS_BIN="$VIS_BUILD_DIR/hexapod-opengl-visualiser"
FORCE_BUILD=0
SKIP_BUILD=0
VISUALISER_ARGS=()

usage() {
  cat <<'USAGE'
Usage: scripts/run_visualiser.sh [options] [-- <visualiser args>]

Starts hexapod-opengl-visualiser, building it on demand when needed.

Options:
  --build                  Force a CMake configure/build before launch.
  --install-deps           Alias for --build (legacy compatibility).
  --skip-build             Skip build even if the binary is missing; fail instead.
  -h, --help               Show this help text.

Examples:
  scripts/run_visualiser.sh -- --udp-port 9870
  scripts/run_visualiser.sh --build -- --udp-port 9871
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build|--install-deps)
      FORCE_BUILD=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      while [[ $# -gt 0 ]]; do
        VISUALISER_ARGS+=("$1")
        shift
      done
      ;;
    *)
      VISUALISER_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -d "$VIS_DIR" ]]; then
  msg_error "OpenGL visualiser directory not found: $VIS_DIR"
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 && ( "$FORCE_BUILD" -eq 1 || ! -x "$VIS_BIN" ) ]]; then
  run_in_dir "$VIS_DIR" cmake -S . -B build
  run_in_dir "$VIS_DIR" cmake --build build -j --target hexapod-opengl-visualiser
fi

if [[ ! -x "$VIS_BIN" ]]; then
  msg_error "OpenGL visualiser binary not found or not executable: $VIS_BIN"
  exit 1
fi

echo "Launching hexapod-opengl-visualiser"
run_in_dir "$VIS_DIR" "$VIS_BIN" "${VISUALISER_ARGS[@]}"
