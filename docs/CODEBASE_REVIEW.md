# Hexapod Codebase Review (March 24, 2026)

This review covers repository structure, maintainability, test strategy, and roadmap opportunities across `hexapod-server`, `hexapod-client`, and `hexapod-common`.

## Executive summary

### What is working well
- Clear separation of concerns between host runtime, firmware runtime, and shared protocol primitives.
- Strong test suite coverage in `hexapod-server` for control, transport, runtime, and parser behavior.
- Robust end-to-end verification script (`scripts/verify.sh`) that chains server tests, firmware host tests, and simulator smoke.

### Highest-value refactoring opportunities
1. **Reduce command boilerplate in `SimpleHardwareBridge`** with typed request helper templates.
2. **Split oversized bridge implementation** into command-focused translation units.
3. **Tighten build hygiene** by ensuring generated test/build outputs are always ignored.
4. **Rationalize CMake source enumeration** (choose either explicit lists or `GLOB_RECURSE`, not both).
5. **Unify config parse/validation defaults** into declarative schemas to reduce duplicated fallback code.

---

## In-depth findings and refactoring opportunities

## 1) `SimpleHardwareBridge` is doing too much in one file
- `hexapod-server/src/hardware/hardware_bridge.cpp` is one of the largest implementation files and contains initialization/link management, payload assembly, request/response decoding, command-specific logging, and calibration upload behavior in one class.
- The command methods follow repeated structural patterns:
  - `requireReady(...)`
  - `withCommandApi(...)`
  - `complete_command(...)`
  - decode or map output

### Refactor direction
- Introduce reusable command helpers (for example `runAckCommand`, `runDecodedCommand<T>`, `runScalarFloatCommand`) to reduce repeated error plumbing and lambda boilerplate.
- Split by behavior into files such as:
  - `hardware_bridge_lifecycle.cpp`
  - `hardware_bridge_power_sensor.cpp`
  - `hardware_bridge_servo.cpp`
  - `hardware_bridge_led.cpp`
  - `hardware_bridge_calibration.cpp`

### Expected payoff
- Lower cognitive load and easier review of protocol changes.
- Smaller PR blast radius for future command additions.
- Fewer accidental inconsistencies in command error handling.

## 2) CMake source management has redundancy risk
- `hexapod-server/CMakeLists.txt` currently uses `file(GLOB_RECURSE ... src/*.cpp)` and also manually appends specific sources before deduplicating.

### Refactor direction
- Pick one strategy:
  - **Preferred for reliability:** explicit source lists grouped by module.
  - **Alternative:** pure glob with strict code-owner discipline and CI checks.
- If using explicit lists, add module comments so adding files is intentional and reviewable.

### Expected payoff
- Fewer accidental source-linking surprises.
- Clearer ownership by subsystem.

## 3) Config parsing/validation can become schema-driven
- `src/config/config_validation.cpp` has several fallback parser variants (`int`, `u64`, `double`, `bool`, vector values, `Vec3` lists) with similar range checks and logging semantics.
- `TomlParser::parse(...)` orchestrates many section-specific parse functions with repetitive boolean chaining patterns.

### Refactor direction
- Introduce a compact validation descriptor model (`key`, type, bounds, default, required/optional, section tag).
- Provide common conversion/validation primitives and reduce custom one-off parser logic.
- Return structured diagnostics (code + key + section + message), not only log side effects.

### Expected payoff
- Easier schema evolution and migration to future `SchemaVersion` values.
- Better UX for CLI tools that may want machine-readable config diagnostics.

## 4) Error semantics should distinguish capability mismatch vs transport failure
- In `withCommandApi(...)`, capability negotiation fallback and command transport failure are sometimes folded into broad error categories.

### Refactor direction
- Introduce richer bridge result metadata (`error`, `command_code`, `phase`, `retryable`, `peer_capability_state`).
- Add optional metrics counters per command outcome.

### Expected payoff
- Faster diagnosis for field issues (USB transport instability vs protocol/capability mismatch).
- Better observability for reliability tuning.

## 5) Test-build artifacts should be consistently ignored
- Server and client include dedicated out-of-source test build directories (`build-tests`, `build-host-tests`) that can appear as untracked clutter if not ignored.

### Refactor direction
- Expand per-project `.gitignore` entries to include these generated test build directories.

### Expected payoff
- Cleaner `git status` and reduced accidental add/commit risk.

---

## Recommended next steps (execution plan)

## Next 2 weeks (stabilization)
1. Implement `.gitignore` hardening for generated test build directories.
2. Add one reusable bridge helper (`runDecodedCommand`) and migrate 2-3 commands as a pilot.
3. Add a small benchmark test around control-loop jitter in simulated runtime.

## Next 1-2 months (maintainability)
1. Split `hardware_bridge.cpp` by domain while preserving API compatibility.
2. Replace parser fallback duplication with schema descriptors and shared validation utilities.
3. Add CI linting checks (format + static analysis profile) to catch complexity regressions.

## Next quarter (product maturity)
1. Add telemetry/event stream output for runtime health and command outcomes.
2. Build richer scenario coverage (fault injection matrix + endurance/soak simulation).
3. Add compatibility tests for protocol version negotiation between server/client revisions.

---

## Feature ideas / future directions

## A) Runtime introspection & developer tooling
- Live diagnostics endpoint (CLI + JSON output mode) exposing:
  - loop timing percentiles,
  - freshness gate state,
  - safety supervisor state transitions,
  - active gait/mode and command rates.
- Benefit: dramatically easier tuning and support in both sim and hardware sessions.

## B) “Protocol evolution” guardrails
- Add protocol metadata hash exchange and negotiated capability map with strict feature flags.
- Add compatibility test matrix in CI:
  - latest server vs N-1 firmware,
  - latest firmware vs N-1 server.
- Benefit: safer incremental protocol changes.

## C) Calibration UX improvements
- Guided calibration wizard mode that logs each stage and validates ranges before apply.
- Persist calibration confidence/quality score per joint based on probe stability and repeatability.
- Benefit: fewer bad calibration deployments and easier operator workflows.

## D) More realistic simulation
- Add optional noise models and transient communication faults (bursty packet loss, delayed ACKs, intermittent sensor stalls).
- Add scripted terrain/contact scenarios for estimator robustness.
- Benefit: better pre-hardware validation of resilience.

## E) Motion quality and safety enhancements
- Add gait transition smoothing and jerk-limited body commands.
- Add safety envelope checks for body pose requests before IK submission.
- Benefit: reduced instability risk and smoother operator feel.

---

## Suggested review metrics to track going forward
- Mean / p95 loop execution time by loop type (bus/estimator/safety/control).
- Command success rate by command code.
- Fault class frequency over runtime hours.
- Calibration rejection rate (and reasons).
- Scenario suite pass rate + runtime duration trends.

These metrics can act as objective acceptance criteria for refactors and feature rollouts.
