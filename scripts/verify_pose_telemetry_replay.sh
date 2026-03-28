#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

PORT=9871
HOST="127.0.0.1"
MODE="sim"
SCENARIO="scenarios/05_long_walk_observability.toml"
OUTPUT_PATH=""
METRICS_OUTPUT_PATH=""
ANALYZE_STRIDE_METRICS=1
SKIP_BUILD=0
KEEP_CAPTURE=0

usage() {
  cat <<'USAGE'
Usage: scripts/verify_pose_telemetry_replay.sh [options]

Runs a scenario replay with UDP telemetry capture and validates pose telemetry invariants.

Checks:
  1) Counts frames containing autonomy_debug.localization.current_pose.
  2) Ensures pose and commanded speed values are finite.
  3) Requires at least one non-zero movement interval when commanded_speed_mps is non-zero.

Options:
  --host <host>            UDP capture bind host (default: 127.0.0.1).
  --port <port>            UDP capture port + server telemetry port (default: 9871).
  --scenario <path>        Scenario path relative to hexapod-server/ (default: scenarios/05_long_walk_observability.toml).
  --output <path>          NDJSON output path (default: temp file under .tmp/).
  --metrics-output <path>  JSON metrics output path (default: sibling .metrics.json).
  --no-metrics             Skip stride/limiter/freshness metric extraction.
  --skip-build             Pass through to run_server_with_telemetry.sh --skip-build.
  --keep-capture           Keep capture file (default temp file is removed on success).
  -h, --help               Show this help text.

Runtime/ports:
  - Typical runtime is ~30-90 seconds when the server is already built.
  - Default capture + telemetry UDP port is 9871 on loopback.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --port)
      PORT="$2"
      shift 2
      ;;
    --scenario)
      SCENARIO="$2"
      shift 2
      ;;
    --output)
      OUTPUT_PATH="$2"
      shift 2
      ;;
    --metrics-output)
      METRICS_OUTPUT_PATH="$2"
      shift 2
      ;;
    --no-metrics)
      ANALYZE_STRIDE_METRICS=0
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --keep-capture)
      KEEP_CAPTURE=1
      shift
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

if [[ -z "$OUTPUT_PATH" ]]; then
  mkdir -p "$HEXAPOD_ROOT_DIR/.tmp"
  OUTPUT_PATH="$HEXAPOD_ROOT_DIR/.tmp/pose_telemetry_capture_$(date +%Y%m%d_%H%M%S).ndjson"
fi

OUTPUT_PATH="$(resolve_from_root "$OUTPUT_PATH")"
mkdir -p "$(dirname "$OUTPUT_PATH")"

CAPTURE_PID=""
cleanup() {
  if [[ -n "$CAPTURE_PID" ]] && kill -0 "$CAPTURE_PID" 2>/dev/null; then
    kill "$CAPTURE_PID" 2>/dev/null || true
    wait "$CAPTURE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

section "Start telemetry capture"
run python3 "$HEXAPOD_ROOT_DIR/scripts/capture_udp_telemetry.py" \
  --host "$HOST" \
  --port "$PORT" \
  --output "$OUTPUT_PATH" &
CAPTURE_PID=$!

sleep 0.5
if ! kill -0 "$CAPTURE_PID" 2>/dev/null; then
  msg_error "Capture process failed to start"
  exit 1
fi

section "Run scenario with telemetry"
SERVER_CMD=(
  "$HEXAPOD_ROOT_DIR/scripts/run_server_with_telemetry.sh"
  --mode "$MODE"
  --telemetry-host "$HOST"
  --telemetry-port "$PORT"
  --scenario "$SCENARIO"
)
if [[ "$SKIP_BUILD" -eq 1 ]]; then
  SERVER_CMD+=(--skip-build)
fi

if ! run "${SERVER_CMD[@]}"; then
  msg_error "Scenario run failed"
  exit 1
fi

section "Stop telemetry capture"
cleanup
CAPTURE_PID=""

section "Validate telemetry invariants"
run python3 - "$OUTPUT_PATH" <<'PY'
import json
import math
import sys
from pathlib import Path

path = Path(sys.argv[1])
if not path.exists() or path.stat().st_size == 0:
    print(f"ERROR: capture file is missing or empty: {path}", file=sys.stderr)
    raise SystemExit(2)

pose_count = 0
finite_ok = True
movement_detected = False
checked_intervals = 0

prev_pose = None

SPEED_EPS = 1e-4
MOVE_EPS = 1e-4


def parse_pose(datagram):
    try:
        pose = datagram["autonomy_debug"]["localization"]["current_pose"]
    except (TypeError, KeyError):
        return None

    if not isinstance(pose, dict):
        return None

    keys = ("x_m", "y_m", "yaw_rad")
    values = []
    for key in keys:
        value = pose.get(key)
        if not isinstance(value, (int, float)):
            return None
        values.append(float(value))
    return tuple(values)


def parse_commanded_speed(datagram):
    try:
        value = datagram["dynamic_gait"]["commanded_speed_mps"]
    except (TypeError, KeyError):
        return None
    if not isinstance(value, (int, float)):
        return None
    return float(value)


with path.open("r", encoding="utf-8") as handle:
    for line_no, line in enumerate(handle, start=1):
        text = line.strip()
        if not text:
            continue

        try:
            row = json.loads(text)
        except json.JSONDecodeError as exc:
            print(f"ERROR: invalid JSON at line {line_no}: {exc}", file=sys.stderr)
            raise SystemExit(3)

        datagram = row.get("datagram")
        if not isinstance(datagram, dict):
            continue

        pose = parse_pose(datagram)
        cmd_speed = parse_commanded_speed(datagram)

        if pose is not None:
            pose_count += 1
            if not all(math.isfinite(v) for v in pose):
                finite_ok = False
                print(f"ERROR: non-finite pose at line {line_no}: {pose}", file=sys.stderr)

        if cmd_speed is not None and not math.isfinite(cmd_speed):
            finite_ok = False
            print(f"ERROR: non-finite commanded speed at line {line_no}: {cmd_speed}", file=sys.stderr)

        if pose is None or cmd_speed is None:
            continue

        if prev_pose is not None and abs(cmd_speed) > SPEED_EPS:
            dx = pose[0] - prev_pose[0]
            dy = pose[1] - prev_pose[1]
            travel = math.hypot(dx, dy)
            checked_intervals += 1
            if travel > MOVE_EPS:
                movement_detected = True
        prev_pose = pose

if pose_count == 0:
    print("ERROR: no frames with autonomy_debug.localization.current_pose", file=sys.stderr)
    raise SystemExit(4)

if not finite_ok:
    raise SystemExit(5)

if checked_intervals == 0:
    print("ERROR: no intervals with commanded_speed_mps != 0 were found", file=sys.stderr)
    raise SystemExit(6)

if not movement_detected:
    print("ERROR: commanded non-zero speed was never accompanied by measurable pose movement", file=sys.stderr)
    raise SystemExit(7)

print(
    f"PASS: pose_frames={pose_count} intervals_checked={checked_intervals} "
    f"movement_detected={movement_detected} capture={path}"
)
PY

if [[ "$ANALYZE_STRIDE_METRICS" -eq 1 ]]; then
  section "Compute stride/limiter/freshness metrics"
  if [[ -z "$METRICS_OUTPUT_PATH" ]]; then
    METRICS_OUTPUT_PATH="${OUTPUT_PATH%.ndjson}.metrics.json"
  fi
  METRICS_OUTPUT_PATH="$(resolve_from_root "$METRICS_OUTPUT_PATH")"
  run python3 "$HEXAPOD_ROOT_DIR/scripts/metrics_stride_analysis.py" \
    --input "$OUTPUT_PATH" \
    --scenario "$(resolve_server_path "$SCENARIO")" \
    --json-output "$METRICS_OUTPUT_PATH" \
    --pretty
  echo "Metrics JSON written to: $METRICS_OUTPUT_PATH"
fi

if [[ "$KEEP_CAPTURE" -eq 0 ]]; then
  rm -f "$OUTPUT_PATH"
else
  echo "Capture retained at: $OUTPUT_PATH"
fi

echo "Pose telemetry replay verification passed."
