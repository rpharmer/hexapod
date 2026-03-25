#!/usr/bin/env bash
set -euo pipefail

# Bootstrap third-party host dependencies used by hexapod-server.
cd ..
git clone https://github.com/ToruNiina/toml11
cd toml11
git submodule update --init --recursive
cmake -B ./build/
cmake --build ./build/
cmake --install ./build/

cd ..
git clone https://github.com/gbmhunter/CppLinuxSerial
cd CppLinuxSerial
cmake -S . -B build -DBUILD_TESTS=FALSE
cmake --build build -j"$(nproc)"
cmake --install build

# Install build toolchain + Python runtime used by visualiser.
sudo apt update
sudo apt install -y cmake gcc-arm-none-eabi build-essential python3-venv python3-pip

# Bootstrap Pico SDKs used by hexapod-client firmware build.
cd /workspace/
mkdir -p pico
cd pico
git clone https://github.com/raspberrypi/pico-sdk
cd pico-sdk
git submodule update --init

cd ..
git clone https://github.com/pimoroni/pimoroni-pico.git
cd pimoroni-pico
git submodule update --init

# Build all project targets so setup cache contains every object/binary flavor.
cd /workspace/hexapod/hexapod-server
cmake --preset default
cmake --build --preset default -j"$(nproc)"

#cmake --preset tests
#cmake --build --preset tests -j"$(nproc)"

cd /workspace/hexapod/hexapod-client
# Prebuild SDK/tooling artifacts that are otherwise fetched lazily.
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks -j"$(nproc)"

# Build firmware artifacts (.elf/.bin/.hex/.uf2).
cmake --preset default
cmake --build --preset default -j"$(nproc)"

# Build native host test binaries.
cmake --preset host-tests
cmake --build --preset host-tests -j"$(nproc)"

# Install visualiser Python dependencies in a local virtual environment.
cd /workspace/hexapod/hexapod-visualiser
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
