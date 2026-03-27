# Hexapod Codebase Review (2026-03-27)

This review supersedes older dated inventories/reviews and focuses on the current monorepo major units.

## Executive summary

The repository is organized around a clear host/firmware/shared-protocol split, with a strong simulation and regression-testing posture:

- `hexapod-server/` provides the largest runtime/control surface and now includes autonomy contracts, supervision semantics, scenario runs, and broad unit/integration coverage.
- `hexapod-client/` firmware keeps command handling and lifecycle logic modular and testable via native host tests.
- `hexapod-common/` remains the shared protocol and framing source-of-truth between host and firmware.
- `hexapod-visualiser/` and `scripts/` provide practical observability and smoke-test paths.

The highest-value next engineering focus is not raw feature breadth, but production-hardening: true process split for autonomy, hardware-in-the-loop validation, and longer-duration reliability evidence.

---

## Major-unit review

## 1) `hexapod-server/` (host runtime + control + autonomy)

### Current strengths
- Clear subsystem partitioning under `src/` and `include/` (app/config/control/hardware/input/kinematics/scenario/autonomy).
- Strong test density across control, runtime, autonomy, transport, and scenario flows.
- Simulation/scenario workflows are first-class and scriptable.
- Autonomy stack includes:
  - mission/navigation/recovery wiring,
  - contract envelope validation,
  - supervision metadata and degraded-mode behavior.

### Risks / gaps
- Runtime complexity is concentrated in server and will continue growing as process split work lands.
- Process-group semantics are implemented and tested in-process, but true multi-process separation is still a follow-on milestone.
- HIL and endurance evidence should continue expanding to match simulation confidence.

### Recommended near-term actions
1. Complete true process split with concrete IPC transport and external supervisor lifecycle orchestration.
2. Expand HIL acceptance around mission/recovery/degraded behavior.
3. Add reliability campaigns for restart churn and sustained degraded operation.

---

## 2) `hexapod-client/` (Servo 2040 firmware)

### Current strengths
- Responsibilities are clear: command dispatch, motion/power/sensing domains, firmware lifecycle, and serial transport.
- Protocol handling aligns to shared constants in `hexapod-common`.
- Host-native tests provide fast regression checks without requiring target hardware.

### Risks / gaps
- Firmware/host protocol evolution still requires strict synchronization discipline.
- Hardware-only behavior (timing/load effects) remains less observable than host-side simulation paths.

### Recommended near-term actions
1. Keep compatibility checks tightly coupled to `hexapod-common` protocol changes.
2. Continue adding host-native tests for command edge cases.
3. Increase hardware-run diagnostics capture for triage parity with server-side observability.

---

## 3) `hexapod-common/` (shared protocol/framing)

### Current strengths
- Maintains centralized protocol IDs/layout/framing and supports both host and firmware consumers.
- Supports the repository’s contract-first approach and reduces accidental wire-format drift.

### Risks / gaps
- Any uncoordinated changes can break both host and firmware simultaneously.

### Recommended near-term actions
1. Preserve single-source-of-truth discipline for command IDs/payload expectations.
2. Ensure compatibility gates stay in default verification paths.

---

## 4) `hexapod-visualiser/` (telemetry UI path)

### Current strengths
- Lightweight architecture (UDP ingest + WebSocket fanout) keeps visualisation decoupled from control loops.
- Includes diagnostics counters/health endpoint and parser/smoke tests.
- Capture/replay tooling improves reproducibility for telemetry regressions.

### Risks / gaps
- Telemetry schema changes must stay synchronized with producer evolution in server.
- As autonomy observability grows, dashboard/schema governance becomes more important.

### Recommended near-term actions
1. Keep telemetry schema/version checks synchronized with server changes.
2. Expand replay fixtures for autonomy-specific degraded/recovery telemetry events.

---

## 5) `scripts/` + top-level workflows

### Current strengths
- `scripts/verify.sh` provides one fail-fast quality gate entrypoint.
- Scenario and visualiser scripts reduce friction for routine regression and demos.

### Risks / gaps
- As coverage grows, execution time and environment variability can increase CI/local divergence risk.

### Recommended near-term actions
1. Keep verify defaults stable and deterministic.
2. Continue separating “fast default gates” from “extended reliability campaigns”.

---

## Overall prioritized recommendations

1. **P0:** true multi-process autonomy runtime split using existing logical contracts.
2. **P0:** HIL mission/recovery/degraded acceptance expansion.
3. **P1:** endurance/reliability campaigns with trend metrics (restart counts, degraded-time share, mission success rate).
4. **P1:** telemetry/schema governance across server/visualiser for autonomy observability growth.
