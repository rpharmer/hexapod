# Next Steps, Fixes, and Improvements

This roadmap reflects the current repository state after a fresh build and codebase review.

## 1) Immediate fixes (high impact, low effort)

1. **Fix top-level documentation drift**
   - Update `README.md` to remove/replace the non-existent `hexapod-common/protocol.md` reference.
   - Keep one canonical “quick verification flow” block for client firmware build commands.

2. **Define firmware shutdown behavior**
   - Add an explicit exit condition for the firmware command loop.
   - Make post-loop cleanup reachable and correct (`disable` vs `enable` behavior should match comments and intended safety policy).

3. **Document bring-up + recovery runbook**
   - Add a short operator checklist for serial connection, handshake validation, servo enable sequencing, and safe stop.

## 2) Reliability and safety hardening

1. **Protocol framing regression tests**
   - Add tests for truncated packets, invalid CRC, malformed length fields, and unknown command handling.

2. **Handshake/link-health scenario tests**
   - Add tests for retry behavior, timeout handling, and out-of-sequence responses.

3. **Safety policy expansion**
   - Add explicit handling and tests for stale heartbeat windows, repeated serial timeouts, and estimator/controller divergence thresholds.

## 3) Motion/control quality improvements

1. **Replace body-controller placeholder output**
   - Implement stance/swing placement policy and validate output ranges against kinematic constraints.

2. **Replace fallback gait speed estimate**
   - Drive stride-rate from command/state inputs rather than static fallback constants.

3. **Tighten control interfaces with stronger typing**
   - Introduce aliases/wrappers for timestamps and angular units at module boundaries.

## 4) Architecture and maintainability

1. **Decompose `hexapod-client.cpp`**
   - Split hardware initialization, command dispatch, and command handlers into focused files/modules.

2. **Decompose `RobotControl` orchestration**
   - Separate loop scheduling, state coordination, and reporting/diagnostics responsibilities.

3. **Externalize geometry/calibration configuration**
   - Move hardcoded geometry defaults into configuration files with schema/version checks.

## 5) Suggested 30/60/90 day plan

- **Next 30 days**
  - Resolve README drift and firmware shutdown semantics.
  - Add basic framing error-path tests.

- **Next 60 days**
  - Land body-controller and gait-speed improvements.
  - Introduce CI checks for server build, client build, and framing tests.

- **Next 90 days**
  - Complete client/robot-control decomposition.
  - Finalize config schema versioning for geometry/calibration data.

## Definition of done for roadmap items

- Each completed item includes:
  - reproducible build/test verification,
  - relevant documentation updates,
  - explicit safety impact note (what risk is reduced, added, or unchanged).
