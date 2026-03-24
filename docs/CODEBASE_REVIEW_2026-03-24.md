# Codebase Review (March 24, 2026)

## Scope

This review covers host control (`hexapod-server`), firmware dispatch (`hexapod-client`), shared protocol metadata (`hexapod-common`), build/test wiring, and maintainability hotspots.

Reviewed references include:

- Root architecture docs and execution plan.
- Core runtime/control orchestration.
- Hardware bridge command transport + decode pathways.
- Firmware command-dispatch table and host-liveness handling.
- Build-system test registration for host and firmware.
- Existing integration/unit tests around command flow and handler validation.

## High-level assessment

The codebase is in a healthy intermediate state: protocol metadata centralization and freshness-gate decomposition work are present and align with the in-repo execution plan. The main remaining challenges are now concentrated around (1) reducing orchestration coupling in large control/hardware files, and (2) closing end-to-end test gaps between firmware dispatch and host transport behavior.

## Strengths

1. **Clear modular boundaries in core runtime path**
   - `RobotRuntime` cleanly separates bus, estimator, safety, control, and diagnostics phases and uses dedicated collaborators (`freshness_policy_`, `ControlPipeline`, `SafetySupervisor`).

2. **Shared metadata-driven firmware routing**
   - Firmware route payload policies are derived from shared command metadata and validated via `static_assert`, preventing drift between route table and metadata contracts.

3. **Good safety defaults and stale-data fallback posture**
   - Freshness-gate failures route control to `SAFE_IDLE`, zero joint targets, and publish explicit fault mode (`ESTIMATOR_INVALID` vs `COMMAND_TIMEOUT`).

4. **Meaningful integration-style transport tests on server**
   - `test_command_flow_integration.cpp` validates realistic host request/ACK/NACK semantics and malformed payload paths over a fake endpoint.

## Refactoring opportunities (prioritized)

### 1) Split `SimpleHardwareBridge` into focused collaborators (High impact)

**Current issue**
- `SimpleHardwareBridge` currently owns serial init, transport/handshake lifecycle, command RPC wrappers, decode handling, and software-feedback synthesis in one class. This centralization increases change-risk and makes targeted tests more expensive.

**Evidence**
- Large implementation file size and mixed responsibilities in `hardware_bridge.cpp`.
- Runtime-specific behavior (`synthesizeJointFeedback`) is embedded beside transport concerns.

**Proposed refactor**
- Introduce three internal components:
  1. `BridgeLinkManager` (serial endpoint, heartbeat, re-establish).
  2. `BridgeCommandApi` (typed request/ack/decode wrappers).
  3. `JointFeedbackEstimator` (software fallback dynamics).
- Keep `SimpleHardwareBridge` as facade/composition root.

**Benefits**
- Smaller units with narrower interfaces.
- Easier deterministic unit tests for each concern.
- Lower blast radius for future protocol updates.

### 2) Reduce orchestration complexity in `RobotRuntime::controlStep`/`safetyStep` (High impact)

**Current issue**
- Freshness evaluation and rejection policy are mostly extracted, but control flow still combines telemetry bookkeeping, freshness handling, fault shaping, and pipeline invocation in a single function.

**Evidence**
- `controlStep()` performs multiple side effects plus decision logic.
- `safetyStep()` recomputes freshness with different policy flags and updates shared freshness state, which can become subtle as policy grows.

**Proposed refactor**
- Extract a pure `ControlDecision` function returning `{allow_pipeline, status, joint_targets}`.
- Extract freshness snapshot update into one helper used by both safety/control with explicit mode enum (`StrictControl`, `SafetyLenient`).
- Move stale counters/metrics writes into a single `recordFreshnessMetrics(...)` helper.

**Benefits**
- Easier proof and tests for mode/fault precedence.
- Lower chance of divergence between safety and control freshness semantics.

### 3) Decompose `calibration_probe.cpp` into algorithm modules (Medium-high impact)

**Current issue**
- Calibration fitting, contact filtering, base-plane computations, and servo dynamics fitting live in a single large source file.

**Evidence**
- Large file footprint and many static utility functions spanning unrelated calibration phases.

**Proposed refactor**
- Split into:
  - `touch_residuals.*`
  - `plane_estimation.*`
  - `servo_dynamics_fit.*`
  - `probe_contact_logic.*`
- Keep a small orchestration file for end-to-end probe workflow.

**Benefits**
- Improved readability for future Phase-4 probe workflow.
- Better unit-test granularity and lower compile churn.

### 4) Unify logger ownership and eliminate global default lookups in critical paths (Medium impact)

**Current issue**
- Parts of the host runtime use injected logger pointers, but hardware/config components still frequently call `GetDefaultLogger()` global state.

**Evidence**
- Global default logger singleton in logger implementation.
- Hardware bridge uses `GetDefaultLogger()` for command/link failures.

**Proposed refactor**
- Pass logger handles explicitly into bridge/config parser constructors.
- Keep global fallback only for bootstrapping or legacy compatibility.

**Benefits**
- Better test determinism and lower hidden coupling.
- Easier log routing for multi-instance/runtime embeddings.

### 5) Tighten test matrix alignment between roadmap intent and build defaults (Medium impact)

**Current issue**
- Plan highlights that malformed payload and broader command-path coverage need stronger CI/default execution. Current CMake files register many server tests and a separate firmware host-test option, but cross-project full-matrix execution still requires manual orchestration.

**Evidence**
- `plan.md` explicitly marks missing automation depth.
- Server and firmware tests are gated by separate options/presets and not unified at repository root.

**Proposed refactor**
- Add a top-level test driver (script or CMake super-target) that runs:
  - server preset tests,
  - firmware host tests,
  - scenario sweep smoke test in sim mode.
- Fail-fast and produce one consolidated summary artifact.

**Benefits**
- Better confidence for protocol and behavior compatibility.
- Lower friction for contributors and CI onboarding.

## Tactical next steps (30/60/90-day)

### Next 30 days

1. **Bridge decomposition spike (no behavior change)**
   - Extract command API + link manager interfaces behind existing `SimpleHardwareBridge` facade.
2. **RobotRuntime decision object extraction**
   - Introduce `ControlDecision` helper and move freshness counter updates behind one API.
3. **Root-level verification command**
   - Add a single script (`scripts/verify.sh`) invoking server tests + firmware host tests.

### 60 days

1. **Calibration probe modularization (file split only first)**
   - Move pure math and fitting helpers into dedicated translation units with unchanged APIs.
2. **Logging dependency injection pass**
   - Replace direct `GetDefaultLogger()` calls in bridge/parser layers with explicit constructor dependencies.
3. **Coverage expansion for command categories**
   - Add end-to-end command path checks for all metadata-declared firmware-dispatch commands.

### 90 days

1. **Phase-4 guided probe scaffolding**
   - Build orchestration shell and telemetry schema extension points.
2. **Scenario-driven regression pack**
   - Add deterministic scenario checks for abort interlocks and stale/fault transitions.
3. **Performance and timing guardrails**
   - Add benchmark-like checks for control-loop jitter ceilings in sim mode.

## Suggested ownership map

- **Control/runtime refactors:** host-control maintainers.
- **Bridge/transport decomposition:** host + protocol maintainers.
- **Dispatch + metadata coverage:** firmware + common-protocol maintainers.
- **Probe modularization:** controls/kinematics maintainers.

## Definition of done for this review cycle

1. No protocol-ID or payload contract behavior changes.
2. Existing tests remain green.
3. New refactors land with characterization tests before structural moves.
4. Roadmap (`plan.md`) updated when each phase checkpoint closes.
