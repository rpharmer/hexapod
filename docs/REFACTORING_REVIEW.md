# Refactoring Opportunities (Prioritized)

## Priority 0 — Safety and protocol correctness guardrails (do first)

### R0.1: Extract host transport stack into dedicated units

**Current state:** `hardware_bridge.cpp` embeds transport session, command client, handshake, codec, and bridge behavior in one file.

**Refactor:**
- Introduce modules:
  - `transport_session.*`
  - `command_client.*`
  - `handshake_client.*`
  - `hardware_state_codec.*`
  - keep `simple_hardware_bridge.*` as orchestrator
- Keep interfaces unchanged at first (`IHardwareBridge`, wire payloads).

**Acceptance criteria:**
- No behavior change in existing transport tests.
- `test_hardware_bridge_transport` remains green.
- Per-module tests exist for handshake and retry edge cases.

### R0.2: Centralize stream freshness policy

**Current state:** freshness checks in `RobotRuntime::controlStep`; timeout/fault policy partly in `SafetySupervisor`.

**Refactor:**
- Introduce a `FreshnessPolicy` component that evaluates both estimator and intent validity.
- Have runtime consume policy result; keep supervisor focused on safety latching/priority semantics.
- Preserve existing fault mapping (`ESTIMATOR_INVALID` vs `COMMAND_TIMEOUT`) as current contract.

**Acceptance criteria:**
- Existing runtime/safety tests pass unchanged.
- Add focused tests for monotonic sample-id, missing timestamp, and stale age boundaries.

## Priority 1 — Configuration and startup composition

### R1.1: Split TOML parser by section

**Current state:** one large parser file with repeated fallback lambdas and section-specific concerns.

**Refactor:**
- Move section parsers into dedicated internal helpers/files:
  - schema/header
  - runtime/transport
  - calibrations
  - tuning
  - geometry
- Consolidate typed range-check helpers in one utility header.

**Acceptance criteria:**
- Same parse results for existing config fixtures.
- New tests for malformed ranges and missing keys by section.

### R1.2: Introduce `AppModeRunner` for main orchestration

**Current state:** `main` contains scenario mode, controller mode, and fallback loop behavior.

**Refactor:**
- Build mode-specific runners (`ScenarioRunner`, `InteractiveRunner`) from parsed CLI options.
- Keep `main` as composition root only (init logger, config, runtime, delegate run mode).

**Acceptance criteria:**
- CLI behavior unchanged.
- Add tests for argument parsing and mode selection.

## Priority 2 — Firmware maintainability and testability

### R2.1: Remove implicit global firmware dependency from handlers

**Current state:** handlers take `FirmwareContext&` but global accessor remains central and pervasive.

**Refactor:**
- Keep `firmware()` only at top-level boot/loop wiring.
- Ensure all command paths are reachable through explicit context injection in tests.

**Acceptance criteria:**
- New tests cover motion/sensing/power handlers with fake serial endpoint.
- No direct `firmware()` usage in command handler implementation files.

### R2.2: Unify payload validation patterns in firmware commands

**Current state:** repeated "size check + decode + NACK" boilerplate across handlers.

**Refactor:**
- Add shared payload decode helpers (`expect_payload<T>` / command-specific wrappers).
- Standardize error responses and logging format.

**Acceptance criteria:**
- Smaller handler functions.
- Equivalent wire responses for invalid payload paths.

## Priority 3 — Developer ergonomics and architectural hygiene

### R3.1: Enforce architecture boundaries with lightweight rules

**Refactor:**
- Add a simple boundary document and optional CI script to flag disallowed include dependencies (e.g., control pipeline must not depend on transport internals).

### R3.2: Expand docs-as-contract for protocol and operating modes

**Refactor:**
- Keep docs in sync with real file structure and current schema keys.
- Add "when to edit" ownership notes per document.

## Suggested sequencing

1. **R0.1 + R0.2** (safety/protocol boundaries)
2. **R1.1 + R1.2** (config/startup readability)
3. **R2.1 + R2.2** (firmware testability)
4. **R3.1 + R3.2** (long-term maintainability)

This sequence minimizes risk while steadily reducing complexity and improving confidence.
