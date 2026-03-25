#!/usr/bin/env bash

# Shared bash helpers for scripts/ launchers and verification entry points.

HEXAPOD_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEXAPOD_ROOT_DIR="$(cd "${HEXAPOD_SCRIPTS_DIR}/.." && pwd)"

section() {
  local title="$1"
  printf '\n========== %s =========='"\n" "$title"
}

run() {
  printf '+ %s\n' "$*"
  "$@"
}

run_in_dir() {
  local dir="$1"
  shift
  printf '+ (cd %s && %s)\n' "$dir" "$*"
  (
    cd "$dir"
    "$@"
  )
}

msg_error() {
  echo "ERROR: $*" >&2
}

msg_warn() {
  echo "WARN: $*" >&2
}

msg_skip() {
  echo "SKIP: $*"
}

resolve_from_root() {
  local path="$1"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s/%s\n' "$HEXAPOD_ROOT_DIR" "$path"
  fi
}

resolve_server_path() {
  local path="$1"
  local server_dir="${HEXAPOD_ROOT_DIR}/hexapod-server"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s/%s\n' "$server_dir" "$path"
  fi
}
