#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib/common.sh"

ROOT_DIR="$HEXAPOD_ROOT_DIR"
VIS_SCRIPT="$ROOT_DIR/scripts/run_visualiser.sh"
VIS_DIR="$ROOT_DIR/hexapod-opengl-visualiser"
VIS_BIN="$VIS_DIR/build/hexapod-opengl-visualiser"
SIM_DIR="$ROOT_DIR/hexapod-physics-sim"
SIM_BIN="$SIM_DIR/build/hexapod-physics-sim"
SERVER_DIR="$ROOT_DIR/hexapod-server"
SERVER_BIN="$SERVER_DIR/build/hexapod-server"

VIS_HOST="127.0.0.1"
UDP_PORT=9870
PHYSICS_HOST="127.0.0.1"
PHYSICS_PORT=9871
SERVER_CONFIG=""
SCENE_FILE=""
CONTROLLER_DEVICE=""
CONTROLLER_OPTIONAL=0
SKIP_BUILD=0
BUILD_JOBS=2
SERVER_ARGS=()

VIS_PID=""
SIM_PID=""
TMP_SERVER_CONFIG=""

usage() {
  cat <<'USAGE'
Usage: scripts/run_physics_stack.sh [options] [-- <extra server args>]

Launches the full controller-driven physics stack:
  1. hexapod-opengl-visualiser
  2. hexapod-physics-sim --serve
  3. hexapod-server in Runtime.Mode = "physics-sim"

Defaults:
  - physics serve mode runs the built-in hexapod rig
  - server config defaults to config.physics-sim.txt (or config.physics-sim-wsl.txt on WSL)
  - Xbox controller device is auto-detected from /dev/input/by-id
  - visualiser + telemetry UDP port = 9870
  - physics serve UDP port = 9871

Options:
  --udp-port <port>           Visualiser ingest and server telemetry UDP port (default: 9870).
  --visualiser-host <host>    Host/IP that receives visualiser UDP packets (default: 127.0.0.1).
  --physics-host <host>       Host/IP that hexapod-server uses for physics IPC (default: 127.0.0.1).
  --physics-port <port>       UDP port for hexapod-physics-sim --serve (default: 9871).
  --server-config <path>      Base server config to use before physics host/port overrides.
  --scene-file <path>         Optional minphys JSON scene appended into serve mode.
  --controller-device <path>  Explicit evdev device path for the Xbox controller.
  --controller-optional       Continue without a controller if auto-detect fails.
  --skip-build                Skip all configure/build steps and launch existing binaries.
  --build-jobs <n>            Max concurrent compile jobs for configure/build (default: 2).
  -h, --help                  Show this help text.

Examples:
  scripts/run_physics_stack.sh
  scripts/run_physics_stack.sh --controller-device /dev/input/event12
  scripts/run_physics_stack.sh --scene-file hexapod-physics-sim/assets/scenes/examples/stack_minimal.json
  scripts/run_physics_stack.sh -- --console-only
USAGE
}

default_server_config() {
  if grep -qi microsoft /proc/version 2>/dev/null && [[ -f "$SERVER_DIR/config.physics-sim-wsl.txt" ]]; then
    printf '%s\n' "$SERVER_DIR/config.physics-sim-wsl.txt"
    return 0
  fi
  printf '%s\n' "$SERVER_DIR/config.physics-sim.txt"
}

detect_controller_device() {
  local candidate=""

  if [[ -d /dev/input/by-id ]]; then
    while IFS= read -r candidate; do
      if [[ -n "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
    done < <(
      find -L /dev/input/by-id -maxdepth 1 -type l \
        \( -iname '*xbox*event-joystick' -o \
           -iname '*xbox*event-*' -o \
           -iname '*microsoft*event-joystick' -o \
           -iname '*x-input*event-joystick' \) \
        2>/dev/null | sort
    )
  fi

  return 1
}

validate_port() {
  local label="$1"
  local value="$2"

  if [[ ! "$value" =~ ^[0-9]+$ ]] || (( value < 1 || value > 65535 )); then
    msg_error "${label} must be an integer in the range 1..65535 (got: ${value})"
    exit 1
  fi
}

validate_build_jobs() {
  if [[ ! "$BUILD_JOBS" =~ ^[0-9]+$ ]] || (( BUILD_JOBS < 1 )); then
    msg_error "--build-jobs must be a positive integer (got: ${BUILD_JOBS})"
    exit 1
  fi
}

render_server_config() {
  local source_config="$1"
  local output_config="$2"

  awk \
    -v physics_host="$PHYSICS_HOST" \
    -v physics_port="$PHYSICS_PORT" \
    '
      BEGIN {
        mode_done = 0;
        host_done = 0;
        port_done = 0;
      }
      $0 ~ /^Runtime\.Mode[[:space:]]*=/ {
        print "Runtime.Mode = \"physics-sim\"";
        mode_done = 1;
        next;
      }
      $0 ~ /^Runtime\.PhysicsSim\.Host[[:space:]]*=/ {
        printf "Runtime.PhysicsSim.Host = \"%s\"\n", physics_host;
        host_done = 1;
        next;
      }
      $0 ~ /^Runtime\.PhysicsSim\.Port[[:space:]]*=/ {
        printf "Runtime.PhysicsSim.Port = %s\n", physics_port;
        port_done = 1;
        next;
      }
      {
        print;
      }
      END {
        if (!mode_done) {
          print "Runtime.Mode = \"physics-sim\"";
        }
        if (!host_done) {
          printf "Runtime.PhysicsSim.Host = \"%s\"\n", physics_host;
        }
        if (!port_done) {
          printf "Runtime.PhysicsSim.Port = %s\n", physics_port;
        }
      }
    ' "$source_config" > "$output_config"
}

launch_in_dir() {
  local __pid_var="$1"
  local dir="$2"
  shift 2

  printf '+ (cd %s &&' "$dir"
  printf ' %q' "$@"
  printf ')\n'

  (
    cd "$dir"
    "$@"
  ) &

  printf -v "$__pid_var" '%s' "$!"
}

cleanup() {
  if [[ -n "$SIM_PID" ]]; then
    kill "$SIM_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "$VIS_PID" ]]; then
    kill "$VIS_PID" >/dev/null 2>&1 || true
  fi
  if [[ -n "$TMP_SERVER_CONFIG" && -f "$TMP_SERVER_CONFIG" ]]; then
    rm -f "$TMP_SERVER_CONFIG"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --udp-port)
      UDP_PORT="$2"
      shift 2
      ;;
    --visualiser-host)
      VIS_HOST="$2"
      shift 2
      ;;
    --physics-host)
      PHYSICS_HOST="$2"
      shift 2
      ;;
    --physics-port)
      PHYSICS_PORT="$2"
      shift 2
      ;;
    --server-config)
      SERVER_CONFIG="$2"
      shift 2
      ;;
    --scene-file)
      SCENE_FILE="$2"
      shift 2
      ;;
    --controller-device|--xbox-device)
      CONTROLLER_DEVICE="$2"
      shift 2
      ;;
    --controller-optional)
      CONTROLLER_OPTIONAL=1
      shift
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
    --)
      shift
      while [[ $# -gt 0 ]]; do
        SERVER_ARGS+=("$1")
        shift
      done
      ;;
    *)
      msg_error "Unknown argument: $1"
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "$VIS_SCRIPT" ]]; then
  msg_error "Visualiser launcher missing or not executable: $VIS_SCRIPT"
  exit 1
fi

if [[ -z "$SERVER_CONFIG" ]]; then
  SERVER_CONFIG_PATH="$(default_server_config)"
else
  SERVER_CONFIG_PATH="$(resolve_from_root "$SERVER_CONFIG")"
fi

if [[ ! -f "$SERVER_CONFIG_PATH" ]]; then
  msg_error "Server config file not found: $SERVER_CONFIG_PATH"
  exit 1
fi

validate_port "--udp-port" "$UDP_PORT"
validate_port "--physics-port" "$PHYSICS_PORT"
validate_build_jobs

if [[ -n "$SCENE_FILE" ]]; then
  SCENE_FILE_PATH="$(resolve_from_root "$SCENE_FILE")"
  if [[ ! -f "$SCENE_FILE_PATH" ]]; then
    msg_error "Scene file not found: $SCENE_FILE_PATH"
    exit 1
  fi
else
  SCENE_FILE_PATH=""
fi

if [[ -z "$CONTROLLER_DEVICE" ]]; then
  CONTROLLER_DEVICE="$(detect_controller_device || true)"
fi

if [[ -n "$CONTROLLER_DEVICE" && ! -e "$CONTROLLER_DEVICE" ]]; then
  msg_error "Controller device path does not exist: $CONTROLLER_DEVICE"
  exit 1
fi

if [[ -z "$CONTROLLER_DEVICE" && "$CONTROLLER_OPTIONAL" -eq 0 ]]; then
  msg_error "No Xbox controller device detected under /dev/input/by-id"
  msg_error "Use --controller-device /dev/input/eventX or pass --controller-optional to run without one"
  exit 1
fi

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  run_in_dir "$VIS_DIR" cmake -S . -B build
  run_in_dir "$VIS_DIR" cmake --build build -j "$BUILD_JOBS" --target hexapod-opengl-visualiser
  run_in_dir "$SIM_DIR" cmake -S . -B build
  run_in_dir "$SIM_DIR" cmake --build build -j "$BUILD_JOBS" --target hexapod-physics-sim
  run_in_dir "$SERVER_DIR" cmake --preset default
  run_in_dir "$SERVER_DIR" cmake --build --preset default -j "$BUILD_JOBS"
fi

if [[ ! -x "$VIS_BIN" ]]; then
  msg_error "OpenGL visualiser binary not found or not executable: $VIS_BIN"
  exit 1
fi

if [[ ! -x "$SIM_BIN" ]]; then
  msg_error "Physics sim binary not found or not executable: $SIM_BIN"
  exit 1
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  msg_error "Server binary not found or not executable: $SERVER_BIN"
  exit 1
fi

TMP_SERVER_CONFIG="$(mktemp "${TMPDIR:-/tmp}/hexapod-physics-stack-config.XXXXXX.toml")"
render_server_config "$SERVER_CONFIG_PATH" "$TMP_SERVER_CONFIG"

trap cleanup EXIT INT TERM

VIS_CMD=("$VIS_SCRIPT" --skip-build -- --udp-port "$UDP_PORT")
launch_in_dir VIS_PID "$ROOT_DIR" "${VIS_CMD[@]}"

sleep 0.5
if ! kill -0 "$VIS_PID" >/dev/null 2>&1; then
  msg_error "Visualiser exited during startup. Check logs above."
  exit 1
fi

SIM_CMD=("$SIM_BIN" --serve --sink udp --udp-host "$VIS_HOST" --udp-port "$UDP_PORT" --serve-port "$PHYSICS_PORT")
if [[ -n "$SCENE_FILE_PATH" ]]; then
  SIM_CMD+=(--scene-file "$SCENE_FILE_PATH")
fi
launch_in_dir SIM_PID "$SIM_DIR" "${SIM_CMD[@]}"

sleep 0.5
if ! kill -0 "$SIM_PID" >/dev/null 2>&1; then
  msg_error "Physics sim exited during startup. Check logs above."
  exit 1
fi

echo "Visualiser running on UDP ${VIS_HOST}:${UDP_PORT}"
echo "Physics sim serving on ${PHYSICS_HOST}:${PHYSICS_PORT} with built-in hexapod rig"
if [[ -n "$SCENE_FILE_PATH" ]]; then
  echo "Physics scene extras loaded from ${SCENE_FILE_PATH}"
fi
if [[ -n "$CONTROLLER_DEVICE" ]]; then
  echo "Controller enabled from ${CONTROLLER_DEVICE}"
else
  echo "Controller not detected; proceeding without controller input"
fi

SERVER_CMD=(
  "$SERVER_BIN"
  --config "$TMP_SERVER_CONFIG"
  --telemetry-enable
  --telemetry-host "$VIS_HOST"
  --telemetry-port "$UDP_PORT"
)
if [[ -n "$CONTROLLER_DEVICE" ]]; then
  SERVER_CMD+=(--controller-device "$CONTROLLER_DEVICE")
fi
SERVER_CMD+=("${SERVER_ARGS[@]}")

echo "Starting hexapod-server in physics-sim mode"
run_in_dir "$SERVER_DIR" "${SERVER_CMD[@]}"
