#!/usr/bin/env bash
set -uo pipefail

log_step() {
  printf '\n==> %s\n' "$1"
}

log_warn() {
  printf 'WARNING: %s\n' "$1" >&2
}

run_step() {
  local description="$1"
  shift

  log_step "$description"
  if "$@"; then
    return 0
  fi

  log_warn "$description failed (continuing)."
  return 1
}

failures=0

# Bootstrap third-party host dependencies used by hexapod-server.
run_step "Prepare toml11 repository" bash -c '
  cd /workspace
  if [ ! -d toml11/.git ]; then
    git clone https://github.com/ToruNiina/toml11
  fi
' || failures=$((failures + 1))

run_step "Build and install toml11" bash -c '
  cd /workspace/toml11
  git submodule update --init --recursive &&
  cmake -B ./build/ &&
  cmake --build ./build/ &&
  cmake --install ./build/
' || failures=$((failures + 1))

run_step "Prepare CppLinuxSerial repository" bash -c '
  cd /workspace
  if [ ! -d CppLinuxSerial/.git ]; then
    git clone https://github.com/gbmhunter/CppLinuxSerial
  fi
' || failures=$((failures + 1))

run_step "Build and install CppLinuxSerial" bash -c '
  cd /workspace/CppLinuxSerial
  cmake -S . -B build -DBUILD_TESTS=FALSE &&
  cmake --build build -j"$(nproc)" &&
  cmake --install build
' || failures=$((failures + 1))

# Install build toolchain + Python runtime used by visualiser.
run_step "Install required apt packages" bash -c '
  sudo apt update &&
  sudo apt install -y cmake gcc-arm-none-eabi build-essential python3-venv python3-pip
' || failures=$((failures + 1))

# Bootstrap Pico SDKs used by hexapod-client firmware build.
run_step "Prepare pico-sdk repository" bash -c '
  cd /workspace
  mkdir -p pico
  cd pico
  if [ ! -d pico-sdk/.git ]; then
    git clone https://github.com/raspberrypi/pico-sdk
  fi
' || failures=$((failures + 1))

run_step "Update pico-sdk submodules" bash -c '
  cd /workspace/pico/pico-sdk
  git submodule update --init
' || failures=$((failures + 1))

run_step "Prepare pimoroni-pico repository" bash -c '
  cd /workspace/pico
  if [ ! -d pimoroni-pico/.git ]; then
    git clone https://github.com/pimoroni/pimoroni-pico.git
  fi
' || failures=$((failures + 1))

run_step "Update pimoroni-pico submodules" bash -c '
  cd /workspace/pico/pimoroni-pico
  git submodule update --init
' || failures=$((failures + 1))

# Build all project targets so setup cache contains every object/binary flavor.
run_step "Build hexapod-server default preset" bash -c '
  cd /workspace/hexapod/hexapod-server
  cmake --preset default &&
  cmake --build --preset default -j"$(nproc)"
' || failures=$((failures + 1))

run_step "Build hexapod-server tests preset" bash -c '
  cd /workspace/hexapod/hexapod-server
  cmake --preset tests &&
  cmake --build --preset tests -j"$(nproc)"
' || failures=$((failures + 1))

run_step "Prepare and build hexapod-client SDK artifacts" bash -c '
  cd /workspace/hexapod/hexapod-client
  cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON &&
  cmake --build build --target setup-sdks -j"$(nproc)"
' || failures=$((failures + 1))

run_step "Build hexapod-client default preset" bash -c '
  cd /workspace/hexapod/hexapod-client
  cmake --preset default &&
  cmake --build --preset default -j"$(nproc)"
' || failures=$((failures + 1))

run_step "Build hexapod-client host-tests preset" bash -c '
  cd /workspace/hexapod/hexapod-client
  cmake --preset host-tests &&
  cmake --build --preset host-tests -j"$(nproc)"
' || failures=$((failures + 1))

# Install visualiser Python dependencies in a local virtual environment.
run_step "Set up hexapod-visualiser virtual environment" bash -c '
  cd /workspace/hexapod/hexapod-visualiser
  python3 -m venv .venv &&
  source .venv/bin/activate &&
  python -m pip install --upgrade pip &&
  python -m pip install -r requirements.txt
' || failures=$((failures + 1))

if [ "$failures" -gt 0 ]; then
  log_warn "Setup completed with $failures failed step(s). Check warnings above for details."
else
  log_step "Setup completed successfully."
fi
