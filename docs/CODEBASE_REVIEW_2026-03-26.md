# Hexapod Codebase Review (2026-03-26) — Monorepo Update

## Executive summary

This update broadens the previous review from mostly server internals to a **monorepo-level assessment** covering host runtime, firmware, protocol-common code, and the visualiser stack.

Reviewed components:

- `hexapod-server/` (runtime orchestration, control pipeline, simulation/scenario harness)
- `hexapod-client/` (Servo 2040 firmware command and lifecycle handling)
- `hexapod-common/` (protocol constants, framing, shared codec helpers)
- `hexapod-visualiser/` and `scripts/` (telemetry transport, smoke tooling)
- root-level docs and verification workflow (`README.md`, `scripts/verify.sh`)

---

## What changed from the prior review

1. **Scope expansion**: prior review emphasized server source inventory; this update adds cross-component integration contracts.
2. **Risk framing update**: findings now prioritize interface drift between server/firmware/common/visualiser.
3. **Documentation refresh**: added explicit docs index guidance so operators and contributors can find canonical docs faster.

---

## Current architecture assessment

## Strengths

1. **Clear component boundaries** with dedicated directories and focused READMEs.
2. **Strong protocol-centralization pattern** through `hexapod-common`, reducing accidental wire-format divergence.
3. **Good simulation and scenario posture** in `hexapod-server`, enabling deterministic safety/control iteration.
4. **Operational tooling exists** (`verify.sh`, scenario scripts, visualiser smoke) to validate multi-component workflows.

## Key risks

1. **Contract drift risk across repos/components**
   - Feature changes can land in server or firmware without synchronized protocol/schema updates.
   - Telemetry shape changes may break visualiser assumptions if not versioned and validated.

2. **Runtime policy duplication risk**
   - Some behavior/policy details are split between docs and code; updates may miss one side.

3. **Observability fragmentation risk**
   - Diagnostics are strong in server internals, but end-to-end health signals (transport, protocol version, visualiser compatibility) are not yet surfaced as a single compatibility report.

---

## Prioritized recommendations

### P0 (Immediate): interface safety

1. **Introduce a protocol/telemetry compatibility checklist in CI**
   - Validate that server + firmware + common constants agree on command IDs and payload sizes.
   - Validate telemetry JSON fields against a documented schema snapshot consumed by visualiser tests.

2. **Publish a single startup compatibility banner**
   - Print runtime mode, protocol version, telemetry schema version, and hardware/sim backend at server start.

### P1 (Near-term): maintainability

3. **Formalize subsystem contracts in docs**
   - Define invariants and failure behavior for control, transport, telemetry, and scenario execution.

4. **Reduce test registration boilerplate**
   - Standardize CMake helper usage for server/client tests to lower maintenance overhead.

### P2 (Medium-term): velocity and regression prevention

5. **Add golden replay harness for telemetry + command traces**
   - Replay archived packets/intents and compare resulting telemetry signatures.

6. **Track trend metrics instead of single-point health checks**
   - Add p95/p99 loop timing, queue backpressure, and reconnection frequency trend outputs.

---

## 90-day execution plan (refined)

## Phase 1 (Weeks 1-3)

- Add compatibility checks (protocol IDs/payloads + telemetry shape) to CI smoke paths.
- Add startup compatibility banner and document expected fields.

## Phase 2 (Weeks 4-8)

- Expand subsystem contract docs and link each section to corresponding tests.
- Refactor high-complexity server modules without behavior changes.

## Phase 3 (Weeks 9-12)

- Introduce replay-driven regression harness.
- Add longitudinal diagnostics trend reporting and alert thresholds.

---

## Success metrics

- Fewer cross-component regressions per release (server/firmware/visualiser mismatch incidents).
- Faster triage time for protocol/telemetry failures.
- Reduced PR review churn on runtime orchestration and transport paths.
- Improved scenario-to-hardware confidence before live robot runs.
