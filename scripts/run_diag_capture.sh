#!/usr/bin/env bash
# Build both binaries with diag logging, run a timed walk scenario, and analyze.
# Run this inside WSL from the repo root:
#
#   bash scripts/run_diag_capture.sh [duration_seconds]
#
# Pass --skip-build to skip the cmake build step if binaries are already fresh.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DURATION="${1:-30}"
SKIP_BUILD=0
[[ "${2:-}" == "--skip-build" ]] && SKIP_BUILD=1

LOG="${TMPDIR:-/tmp}/hexapod_diag_$(date -u +%Y%m%dT%H%M%SZ).log"
SIM_BIN="$ROOT/hexapod-physics-sim/build/hexapod-physics-sim"
SERVER_BIN="$ROOT/hexapod-server/build/hexapod-server"
BUILD_JOBS="${BUILD_JOBS:-2}"

if [[ "$SKIP_BUILD" -eq 0 ]]; then
    echo "=== Building hexapod-physics-sim ==="
    cmake -S "$ROOT/hexapod-physics-sim" -B "$ROOT/hexapod-physics-sim/build" \
          -DCMAKE_BUILD_TYPE=RelWithDebInfo --log-level=WARNING
    cmake --build "$ROOT/hexapod-physics-sim/build" \
          -j "$BUILD_JOBS" --target hexapod-physics-sim 2>&1 | tail -8

    echo "=== Building hexapod-server ==="
    (cd "$ROOT/hexapod-server" && cmake --preset default 2>&1 | tail -3)
    (cd "$ROOT/hexapod-server" && cmake --build --preset default -j "$BUILD_JOBS" 2>&1 | tail -8)
fi

if [[ ! -x "$SIM_BIN" ]]; then
    echo "ERROR: sim binary not found: $SIM_BIN" >&2; exit 1
fi
if [[ ! -x "$SERVER_BIN" ]]; then
    echo "ERROR: server binary not found: $SERVER_BIN" >&2; exit 1
fi

# Use the WSL physics-sim config (already has Runtime.Mode = "physics-sim" and Port = 9871)
if [[ -f "$ROOT/hexapod-server/config.physics-sim-wsl.txt" ]]; then
    SERVER_CFG="$ROOT/hexapod-server/config.physics-sim-wsl.txt"
else
    SERVER_CFG="$ROOT/hexapod-server/config.physics-sim.txt"
fi
PHYSICS_PORT=9871

SIM_PID=""
SERVER_PID=""
cleanup() {
    [[ -n "$SIM_PID" ]] && kill "$SIM_PID" 2>/dev/null || true
    [[ -n "$SERVER_PID" ]] && kill "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "=== Starting sim (diag logging ON) => $LOG ==="
MINPHYS_DIAG_LOG=1 \
    "$SIM_BIN" --serve \
               --serve-port "$PHYSICS_PORT" \
               --sink dummy \
    2>&1 | grep --line-buffered '\[DIAG_\|ERROR\|WARN' >> "$LOG" &
SIM_PID=$!

sleep 1
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "ERROR: sim exited during startup — check $LOG" >&2; exit 1
fi

echo "=== Starting server (diag logging ON) ==="
HEXAPOD_DIAG_LOG=1 \
    "$SERVER_BIN" \
        --config "$SERVER_CFG" \
        --scenario "$ROOT/hexapod-server/scenarios/01_nominal_stand_walk.toml" \
    2>&1 | grep --line-buffered '\[DIAG_\|ERROR\|WARN' >> "$LOG" &
SERVER_PID=$!

echo "=== Running for ${DURATION}s (Ctrl-C to stop early) ... ==="
sleep "$DURATION"

echo "=== Stopping processes ==="
kill "$SERVER_PID" 2>/dev/null || true
kill "$SIM_PID" 2>/dev/null || true
wait "$SERVER_PID" 2>/dev/null || true
wait "$SIM_PID" 2>/dev/null || true
SIM_PID=""
SERVER_PID=""

echo ""
echo "=== Log: $LOG  ($(wc -l < "$LOG") lines) ==="
echo ""
echo "=== Analysis ==="
python3 "$ROOT/scripts/analyze_diag_log.py" "$LOG"
