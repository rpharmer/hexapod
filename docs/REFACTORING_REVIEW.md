# Refactoring Plan (March 23, 2026)

## Goal
Reduce maintenance cost and regression risk in command transport, runtime control gating, and firmware dispatch.

## Workstreams

## 1) Server command execution abstraction

- Add a typed helper for request/response execution inside `SimpleHardwareBridge`.
- Standardize error logging fields: `command`, `outcome`, `nack_code`, `payload_size`.
- Migrate scalar/sensor/calibration/servo-enabled command paths first.

## 2) Runtime control-stage decomposition

- Split `RobotRuntime::controlStep` into decision helpers.
- Make freshness gate decision testable without hardware bridge dependencies.
- Preserve externally visible behavior (SAFE_IDLE fallback and status fault codes).

## 3) Firmware command route simplification

- Eliminate thin forwarding wrappers by using direct bindings/adapters.
- Keep command route table declarative and compile-time.
- Preserve payload policy enforcement (`ExactBytes`).

## 4) Shared protocol command metadata

- Introduce command metadata table in `hexapod-common`.
- Generate or derive command-name lookups for host telemetry.
- Reuse metadata in firmware route definitions where practical.

## 5) Documentation and developer ergonomics

- Keep `docs/CODEBASE_REVIEW.md`, `docs/REFACTORING_REVIEW.md`, and `docs/NEXT_STEPS.md` date-stamped.
- Add “when adding a new command” checklist to root README.

## Exit criteria

- Host tests pass (`ctest --preset tests`).
- No behavior changes in protocol semantics.
- Reduced duplication in targeted files (`hardware_bridge.cpp`, `robot_runtime.cpp`, `command_dispatch.cpp`).
