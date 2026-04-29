#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SIM_DIR="$ROOT_DIR/hexapod-physics-sim"
SERVER_BIN="$SERVER_DIR/build/hexapod-server"
SIM_BIN="$SIM_DIR/build/hexapod-physics-sim"

BUILD_JOBS=2
SKIP_BUILD=0
DURATION_SEC=12
OUT_DIR=""

usage() {
  cat <<'USAGE'
Usage: scripts/trace_control_loop_stability.sh [options]

Capture control-loop traces for both startup paths and compare them:
  - offline `config.sim.txt`
  - interactive `physics-sim` using `config.physics-sim-wsl.txt` when available, otherwise `config.physics-sim.txt`

Options:
  --duration <sec>   Capture duration for each run (default: 12).
  --out-dir <path>   Directory for trace artifacts (default: temp dir).
  --skip-build       Skip CMake configure/build.
  --build-jobs <n>   Max concurrent compile jobs (default: 2).
  -h, --help         Show this help text.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --build-jobs)
      BUILD_JOBS="$2"
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

if [[ ! "$BUILD_JOBS" =~ ^[0-9]+$ ]] || (( BUILD_JOBS < 1 )); then
  msg_error "--build-jobs must be a positive integer (got: ${BUILD_JOBS})"
  exit 1
fi

if [[ ! "$DURATION_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk "BEGIN { exit !($DURATION_SEC > 0) }"; then
  msg_error "--duration must be a positive number (got: ${DURATION_SEC})"
  exit 1
fi

if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/hexapod-control-loop-trace.XXXXXX")"
else
  mkdir -p "$OUT_DIR"
fi

SIM_CONFIG="$SERVER_DIR/config.sim.txt"
PHYSICS_CONFIG="$SERVER_DIR/config.physics-sim.txt"
if [[ -f "$SERVER_DIR/config.physics-sim-wsl.txt" ]]; then
  PHYSICS_CONFIG="$SERVER_DIR/config.physics-sim-wsl.txt"
fi

SIM_LOG="$OUT_DIR/sim-server.log"
PHYSICS_SIM_LOG="$OUT_DIR/physics-sim-serve.log"
PHYSICS_SERVER_LOG="$OUT_DIR/physics-sim-server.log"
SIM_CONTROL_TRACE="$OUT_DIR/sim-control-trace.txt"
PHYSICS_CONTROL_TRACE="$OUT_DIR/physics-control-trace.txt"
SIM_FEEDBACK_TRACE="$OUT_DIR/sim-feedback-trace.txt"
PHYSICS_FEEDBACK_TRACE="$OUT_DIR/physics-feedback-trace.txt"

build_binaries() {
  section "Build"
  run_in_dir "$SERVER_DIR" cmake --preset default
  run_in_dir "$SERVER_DIR" cmake --build --preset default -j "$BUILD_JOBS" --target hexapod-server
  run_in_dir "$SIM_DIR" cmake --preset default
  run_in_dir "$SIM_DIR" cmake --build --preset default -j "$BUILD_JOBS" --target hexapod-physics-sim
}

normalize_trace() {
  sed -E 's/^.*(control_loop_trace|physics_sim_feedback_trace)/\1/; s/ \(.*\)$//'
}

run_with_timeout() {
  local log_file="$1"
  shift
  local status=0
  timeout "${DURATION_SEC}s" "$@" >"$log_file" 2>&1 || status=$?
  if [[ "$status" -ne 124 ]]; then
    msg_error "command failed or exited early (status=${status}): $*"
    return 1
  fi
  return 0
}

run_sim_path() {
  section "Offline sim trace"
  run_with_timeout "$SIM_LOG" \
    "$SERVER_BIN" \
    --config "$SIM_CONFIG" \
    --trace-control-loop
  sed -n '/control_loop_trace/p' "$SIM_LOG" | normalize_trace >"$SIM_CONTROL_TRACE"
  sed -n '/physics_sim_feedback_trace/p' "$SIM_LOG" | normalize_trace >"$SIM_FEEDBACK_TRACE"
}

run_physics_path() {
  section "Physics-sim trace"
  "$SIM_BIN" --serve --serve-port 9871 >"$PHYSICS_SIM_LOG" 2>&1 &
  local sim_pid=$!
  cleanup() {
    kill "$sim_pid" >/dev/null 2>&1 || true
    wait "$sim_pid" >/dev/null 2>&1 || true
  }
  trap cleanup RETURN

  sleep 0.25
  run_with_timeout "$PHYSICS_SERVER_LOG" \
    "$SERVER_BIN" \
    --config "$PHYSICS_CONFIG" \
    --trace-control-loop

  trap - RETURN
  cleanup
  sed -n '/control_loop_trace/p' "$PHYSICS_SERVER_LOG" | normalize_trace >"$PHYSICS_CONTROL_TRACE"
  sed -n '/physics_sim_feedback_trace/p' "$PHYSICS_SERVER_LOG" | normalize_trace >"$PHYSICS_FEEDBACK_TRACE"
}

show_summary() {
  local sim_control_count physics_control_count sim_feedback_count physics_feedback_count
  sim_control_count="$(wc -l <"$SIM_CONTROL_TRACE")"
  physics_control_count="$(wc -l <"$PHYSICS_CONTROL_TRACE")"
  sim_feedback_count="$(wc -l <"$SIM_FEEDBACK_TRACE")"
  physics_feedback_count="$(wc -l <"$PHYSICS_FEEDBACK_TRACE")"

  echo "Artifacts: $OUT_DIR"
  echo "Trace lines: sim_control=$sim_control_count physics_control=$physics_control_count sim_feedback=$sim_feedback_count physics_feedback=$physics_feedback_count"

  if [[ "$sim_control_count" -eq 0 || "$physics_control_count" -eq 0 ]]; then
    msg_error "Missing control trace lines; inspect logs in $OUT_DIR"
    exit 1
  fi

  local sim_walk_line physics_walk_line physics_tip_line sim_tip_line
  local physics_estimator_fault_line sim_estimator_fault_line
  local sim_final_line physics_final_line
  sim_walk_line="$(grep -n 'requested_mode=WALK' "$SIM_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  physics_walk_line="$(grep -n 'requested_mode=WALK' "$PHYSICS_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  sim_tip_line="$(grep -n 'safety_fault=TIP_OVER' "$SIM_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  physics_tip_line="$(grep -n 'safety_fault=TIP_OVER' "$PHYSICS_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  sim_estimator_fault_line="$(grep -n 'safety_fault=ESTIMATOR_INVALID' "$SIM_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  physics_estimator_fault_line="$(grep -n 'safety_fault=ESTIMATOR_INVALID' "$PHYSICS_CONTROL_TRACE" | head -n 1 | cut -d: -f1 || true)"
  sim_final_line="$(tail -n 1 "$SIM_CONTROL_TRACE")"
  physics_final_line="$(tail -n 1 "$PHYSICS_CONTROL_TRACE")"

  if [[ -n "$sim_walk_line" || -n "$physics_walk_line" ]]; then
    msg_error "No-controller startup should remain in SAFE_IDLE; unexpected WALK detected (sim=${sim_walk_line:-none}, physics=${physics_walk_line:-none})"
    exit 1
  fi

  if [[ -n "$sim_tip_line" ]]; then
    msg_error "Offline sim should not tip over in the captured window (first TIP_OVER trace line: ${sim_tip_line})"
    exit 1
  fi

  if [[ -n "$physics_tip_line" ]]; then
    msg_error "Physics-sim should not tip over after the no-controller SAFE_IDLE fix (first TIP_OVER trace line: ${physics_tip_line})"
    exit 1
  fi

  if [[ -n "$sim_estimator_fault_line" ]]; then
    msg_error "Offline sim should not latch ESTIMATOR_INVALID in the startup window (first fault trace line: ${sim_estimator_fault_line})"
    exit 1
  fi

  if [[ -n "$physics_estimator_fault_line" ]]; then
    msg_error "Physics-sim should not latch ESTIMATOR_INVALID in the startup window (first fault trace line: ${physics_estimator_fault_line})"
    exit 1
  fi

  if [[ "$sim_final_line" != *"requested_mode=SAFE_IDLE"* || "$sim_final_line" != *"status_mode=SAFE_IDLE"* || "$sim_final_line" != *"safety_fault=NONE"* ]]; then
    msg_error "Offline sim should settle cleanly into SAFE_IDLE by the end of the trace (final line: ${sim_final_line})"
    exit 1
  fi

  if [[ "$physics_final_line" != *"requested_mode=SAFE_IDLE"* || "$physics_final_line" != *"status_mode=SAFE_IDLE"* || "$physics_final_line" != *"safety_fault=NONE"* ]]; then
    msg_error "Physics-sim should settle cleanly into SAFE_IDLE by the end of the trace (final line: ${physics_final_line})"
    exit 1
  fi

  section "First divergence"
  diff -u <(head -n 40 "$SIM_CONTROL_TRACE") <(head -n 40 "$PHYSICS_CONTROL_TRACE") || true

  if [[ "$sim_feedback_count" -gt 0 || "$physics_feedback_count" -gt 0 ]]; then
    section "Feedback trail"
    diff -u <(head -n 20 "$SIM_FEEDBACK_TRACE") <(head -n 20 "$PHYSICS_FEEDBACK_TRACE") || true
  fi
}

main() {
  if [[ "$SKIP_BUILD" -eq 0 ]]; then
    build_binaries
  fi

  if [[ ! -x "$SERVER_BIN" ]]; then
    msg_error "Server binary not found or not executable: $SERVER_BIN"
    exit 1
  fi
  if [[ ! -x "$SIM_BIN" ]]; then
    msg_error "Physics sim binary not found or not executable: $SIM_BIN"
    exit 1
  fi

  run_sim_path
  run_physics_path
  show_summary
}

main "$@"
