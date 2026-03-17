# Codebase Review: Hexapod

## Review scope

This review re-examined the current repository state across:

- `hexapod-server/` (host runtime, config parsing, control loop orchestration)
- `hexapod-client/` (Servo 2040 firmware and command handlers)
- `hexapod-common/` (wire framing/protocol helpers)
- top-level documentation (`README.md`, `docs/`)

## Build verification

The following builds were run in this environment and completed successfully:

1. Server configure + build
   - `cmake -S hexapod-server -B hexapod-server/build`
   - `cmake --build hexapod-server/build -j4`
2. Client SDK setup target
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON`
   - `cmake --build hexapod-client/build --target setup-sdks -j4`
3. Client full firmware build
   - `cmake -S hexapod-client -B hexapod-client/build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF`
   - `cmake --build hexapod-client/build --target hexapod-client -j4`

## Findings

### 1) Build health: good

- Server and firmware builds are green, and shared framing code compiles into both targets.
- Existing CMake flow is reproducible from a clean configure/build sequence.

### 2) Runtime architecture status: functional skeleton with intentional placeholders

- `RobotControl` currently acts as the central orchestrator for lifecycle, periodic loop execution, command integration, and diagnostic/status publishing.
- `BodyController::update()` is still a placeholder returning default leg targets.
- `GaitScheduler::update()` still uses a fallback speed magnitude constant rather than command-derived speed.

These are reasonable for an incremental bring-up stage, but they remain the most important gaps for motion behavior quality.

### 3) Configuration and protocol handling quality: improved and mostly consistent

- Server TOML parsing validates required keys, calibration key format, key uniqueness, expected cardinality, and pulse bounds before startup.
- Framing/transport RX buffer caps are aligned (`1024` bytes for both generic and transport limits).

### 4) Documentation drift still present

- Top-level `README.md` references `hexapod-common/protocol.md`, which does not exist in the repository.
- `README.md` contains two overlapping “quick verification flow” blocks for client firmware builds.

### 5) Firmware lifecycle cleanup issue

- `hexapod-client` has an infinite command loop (`while(1)`) followed by cleanup code that is never reached.
- The unreachable cleanup section currently calls `enable_all()` in the “Disable servos” comment block, indicating a likely intended `disable_all()` path once shutdown handling is introduced.

## Recommended near-term actions

1. Keep the current build commands as baseline CI checks.
2. Implement a graceful firmware shutdown/exit path (or remove dead cleanup code until supported).
3. Resolve top-level documentation drift (missing protocol file reference and duplicated quick-verify block).
4. Prioritize replacing placeholder body-control and fallback gait speed behavior to improve motion fidelity.
