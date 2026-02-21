# Codebase Structure and Future Steps

This document summarizes how the repository is organized today and outlines practical next improvements.

## Codebase structure

### 1) `hexapod-common/` (shared protocol contract)

- Defines command IDs used by both host and firmware (`SET_ANGLE_CALIBRATIONS`, `SET_TARGET_ANGLE`, etc.).
- Defines byte packing/unpacking helpers (`MSB0..MSB3`, `GETINT`) for serial payloads.
- Declares the `SerialComms` abstract interface so host/client transport details can vary while API stays stable.
- Built with a Makefile to produce both static (`.a`) and shared (`.so`) libraries.

Why it matters: this directory is effectively the protocol contract and shared serialization boundary.

### 2) `hexapod-server/` (Linux host app)

- Linux process using `CppLinuxSerial` and `toml11`.
- Parses `config.txt` (TOML), validates title, loads + sorts `MotorCalibrations` entries.
- Opens `/dev/ttyACM0` at 115200 baud and writes calibration values over serial.
- Contains `SerialCommsServer`, a concrete implementation of the shared `SerialComms` interface.

Why it matters: this is currently the orchestration point where configuration is translated into wire-level bytes.

### 3) `hexapod-client/` (Pico / Servo 2040 firmware)

- Firmware entrypoint initializes ADC/mux, LED bar, servos, and default calibration array.
- Receives command bytes over USB serial and dispatches handlers.
- Handler functions support calibration write/read, set angle, relay control, current/voltage/sensor reads.
- CMake-based build uses Pico SDK + Pimoroni libraries.

Why it matters: this is real-time device control and telemetry surface.

### 4) Root-level docs and wiring context

- Root `README.md` describes three-part architecture and command protocol.
- Build instructions are split between root README and `hexapod-client/README.md` for Pico environment setup.

## Recommended next steps

### Near-term (stability + maintainability)

1. **Remove dead/unreachable code in firmware main loop.**
   There is an infinite serial echo loop before command dispatch, so dispatcher logic will never execute unless the first loop is refactored.

2. **Unify protocol typing in one place.**
   Replace macro constants with strongly typed enums/constexpr values and shared message structs in `hexapod-common`.

3. **Fix host receive stubs.**
   `recv_u32`, `recv_i16`, `recv_i32`, and `recv_f32` currently return zero placeholders in server transport class; implement actual reads or remove until needed.

4. **Make include/link paths portable.**
   `hexapod-client/CMakeLists.txt` currently hardcodes a machine-specific include path for `hexapod-common`.

### Mid-term (quality + testing)

5. **Add protocol round-trip tests.**
   Unit tests around packing/unpacking and command framing in `hexapod-common`.

6. **Add host integration test with virtual serial pair.**
   Validate config parsing + packet emission without hardware.

7. **Add firmware-side command validation.**
   Check ranges for servo index, sensor index, and calibration bounds before applying values.

8. **Version and document protocol messages.**
   Add a simple protocol version handshake and message table with payload schema.

### Longer-term (productization)

9. **Introduce a bidirectional acknowledgment/error model.**
   Standard response frames for success/error/status to make host orchestration robust.

10. **Separate configuration, transport, and behavior layers in server.**
    Improves CLI/API extension and future GUI/controller integration.

11. **Add telemetry polling mode + structured logs.**
    Useful for runtime diagnosis and field calibration.

12. **Automate build + lint in CI.**
    Enforce formatting, compile checks, and tests for common/server paths at minimum.
