# Codebase Review: Hexapod

## Review scope

This review focused on build health and a light static code/documentation pass across:

- `hexapod-server/` (host application and build configuration)
- `hexapod-client/` (firmware and Pico/Pimoroni build configuration)
- `hexapod-common/` (shared framing/protocol helpers)
- repository documentation in `README.md` and `docs/`

## Build verification

The following builds were executed successfully in this environment:

1. Server configure + build
   - `cmake -S hexapod-server -B hexapod-server/build`
   - `cmake --build hexapod-server/build -j4`
2. Client SDK dependency prebuild
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON`
   - `cmake --build hexapod-client/build --target setup-sdks -j4`
3. Client full firmware build
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF`
   - `cmake --build hexapod-client/build --target hexapod-client -j4`

## Review findings

### 1) Build and integration status: healthy

- Both top-level deliverables (`hexapod-server` and `hexapod-client`) configured and compiled successfully.
- Shared framing code in `hexapod-common/framing.cpp` compiles into both targets cleanly, indicating protocol helper reuse is wired correctly.

### 2) Documentation quality has improved, but can still be tightened

- The top-level `README.md` now reflects the CMake-based build flow and current repository layout.
- A minor follow-up would be to keep command duplication low between `README.md` and `docs/FIRMWARE.md` by designating one canonical source for firmware build/flash details.

### 3) TOML header ambiguity: resolved

- `hexapod-server` uses system/package `toml11` (`find_package(toml11 REQUIRED)` and `#include <toml.hpp>`).
- `hexapod-server/include/toml.hpp.tmp` is not present, eliminating the earlier ambiguity around a potential vendored snapshot.

**Recommendation:** keep this state and document external `toml11` as the sole TOML source of truth.

## Suggested next actions

1. Keep the current build verification commands in CI (or script them) to preserve the reproducibility confirmed in this review.
2. Keep external `toml11` as the sole TOML dependency path and avoid reintroducing vendored temporary headers.
3. Continue consolidating firmware documentation to one canonical protocol/build reference.
