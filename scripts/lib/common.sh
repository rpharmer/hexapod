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

# Prefer X11/XWayland under WSLg so GLFW can request activation (Wayland blocks it).
prefer_wsl_x11_backend() {
  if grep -qi microsoft /proc/version 2>/dev/null; then
    if [[ -n "${DISPLAY:-}" ]]; then
      unset WAYLAND_DISPLAY
      export GDK_BACKEND=x11
      export QT_QPA_PLATFORM=xcb
    fi
  fi
}

# Detect broken WSLg VAIL shared-memory (invisible OpenGL windows titled [WARN:COPY MODE]).
wslg_copy_mode_active() {
  if [[ ! -f /mnt/wslg/weston.log ]]; then
    return 1
  fi
  grep -q 'enable_copy_warning_title = 1' /mnt/wslg/weston.log 2>/dev/null
}

warn_if_wslg_copy_mode() {
  if ! grep -qi microsoft /proc/version 2>/dev/null; then
    return 0
  fi
  if ! wslg_copy_mode_active; then
    return 0
  fi
  msg_error "WSLg is in COPY MODE (shared-memory VAIL failed)."
  msg_error "OpenGL windows can appear in Alt+Tab/taskbar but stay invisible/unfocusable."
  msg_error "Confirm:  wsl --system -- ls /mnt/shared_memory"
  msg_error "If that prints 'Function not implemented', focus helpers cannot fix it."
  msg_error "See: ./scripts/check_wslg_gui.sh  and hexapod-opengl-visualiser/README.md"
  return 0
}

# Raise a WSLg-hosted window by title substring via Win32 SetForegroundWindow.
# AppActivate alone fails when WSLg prefixes titles with "[WARN:COPY MODE]".
raise_wslg_window() {
  local title_substr="$1"
  local attempts="${2:-80}"
  local delay_ms="${3:-250}"
  local hold_seconds="${4:-0}"
  local helper="${HEXAPOD_ROOT_DIR}/scripts/raise_wslg_window.ps1"
  local log_file="${HEXAPOD_WSLG_RAISE_LOG:-/tmp/hexapod-wslg-raise.log}"

  if ! grep -qi microsoft /proc/version 2>/dev/null; then
    return 0
  fi
  if [[ ! -f "$helper" ]]; then
    msg_warn "WSLg raise helper missing: $helper"
    return 0
  fi
  if ! command -v powershell.exe >/dev/null 2>&1; then
    return 0
  fi

  local win_helper win_log
  win_helper="$(wslpath -w "$helper" 2>/dev/null || true)"
  win_log="$(wslpath -w "$log_file" 2>/dev/null || true)"
  if [[ -z "$win_helper" ]]; then
    win_helper="$helper"
  fi

  : >"$log_file" 2>/dev/null || true
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$win_helper" \
    -TitleSubstring "$title_substr" \
    -Attempts "$attempts" \
    -DelayMs "$delay_ms" \
    -HoldSeconds "$hold_seconds" \
    -LogFile "$win_log" \
    >/dev/null 2>&1 || true
}
