# Hexapod Monorepo Codebase Review

**Date:** 2026-03-26  
**Reviewer:** Codex (GPT-5.3-Codex)  
**Repository:** `/workspace/hexapod`

---

## 1. Scope and Methodology

This review covered architecture, code organization, protocol contracts, runtime behavior, test/CI workflows, and maintainability hotspots across:

- `hexapod-server/` (host control runtime)
- `hexapod-client/` (firmware + host tests)
- `hexapod-common/` (shared protocol layer)
- `hexapod-visualiser/` (UDP→WebSocket bridge + frontend contract)
- `.github/workflows/` and `scripts/`

### Commands executed

```bash
./scripts/verify.sh
git status --short
```

### Verification result summary

`./scripts/verify.sh` passed all default gates:

1. `hexapod-server` tests (preset `tests`)
2. `hexapod-client` host tests (preset `host-tests`)
3. simulator scenario smoke (`scenarios/01_nominal_stand_walk.toml`)

No tracked source files were modified during analysis; pre-existing untracked `hexapod-visualiser/.venv/` content remains.

---

## 2. Executive Summary

Overall, the repository is in **good operational shape** with strong test coverage and a practical end-to-end verification pipeline. The codebase demonstrates solid engineering fundamentals:

- clear subsystem boundaries,
- shared protocol metadata with compile-time/runtime checks,
- robust scenario-based simulation flow,
- explicit, reproducible quality gates.

The main opportunities are **maintainability and scale-readiness** rather than correctness:

- decompose a few large “orchestrator” source files,
- reduce CMake boilerplate repetition,
- align telemetry schema docs with actual compatibility behavior,
- improve observability of transient logger backpressure.

---

## 3. Current Strengths

### 3.1 Shared protocol contract is centralized and safer than average

`hexapod-common/include/protocol_ids.hpp` defines command IDs, payload sizes, metadata tables, typed parse helpers, and completeness assertions in one place. This significantly reduces host/firmware drift risk.

Notable strengths:

- typed enums + raw byte aliases for compatibility,
- command metadata table (`kCommandMetadata`) with payload contracts,
- compile-time completeness assertion (`command_metadata_is_complete`),
- convenience lookups (`find_command_metadata`, `try_parse_command_code`).

This is also backed by dedicated tests validating metadata correctness and dispatch flags (`test_protocol_command_metadata`).

### 3.2 Quality gate design is practical and high-value

`scripts/verify.sh` is excellent as a “single command of truth” for local/CI checks:

- server configure/build/test,
- client host configure/build/test,
- scenario smoke with config backup/restore safety.

This is high leverage and should remain the default contributor pathway.

### 3.3 Runtime diagnostics are already embedded where they matter

`RobotRuntime` reports loop/freshness/telemetry diagnostics, while bridge command metadata tracks domain/phase/retryability. Combined with controlled teardown logging, this gives meaningful production visibility.

### 3.4 Firmware dispatch has contract-aware routing checks

`hexapod-client/command_dispatch.cpp` uses route tables with payload policies and a compile-time alignment assertion with shared command metadata. This is exactly the kind of guardrail that prevents subtle protocol regressions.

---

## 4. Key Refactoring Opportunities (Prioritized)

## Priority 1 — Decompose high-responsibility runtime orchestration

### Problem

`hexapod-server/src/control/runtime/robot_runtime.cpp` currently manages:

- init/reset state,
- bus/read/write step,
- estimator step,
- control step + freshness gate decisions,
- telemetry cadence + publish behavior,
- safety step,
- diagnostics reporting,
- test hooks / sim fault toggles.

This creates a broad change surface where behavior-level edits are harder to isolate and review.

### Recommendation

Extract collaborators while preserving behavior:

- `ControlAdmission` (freshness + allow/reject decisions),
- `TelemetryLoop` (publish scheduling + geometry refresh),
- `DiagnosticsLoop` (report assembly and counters).

Keep `RobotRuntime` as explicit coordinator.

### Expected payoff

- faster focused testing,
- simpler code reviews,
- lower regression risk from multi-domain edits.

---

## Priority 2 — Split bootstrap concerns in server `main`

### Problem

`hexapod-server/src/app/hexapod-server.cpp` combines process concerns (signals, lifecycle), runtime wiring, config normalization, and logger fallback setup.

### Recommendation

Refactor into small units:

- `config/bootstrap_runtime_config.*` (parse, normalize, clamp),
- `app/runtime_factory.*` (construct bridge/estimator/control),
- keep `main()` mostly as sequencing + exit code policy.

### Expected payoff

- easier feature additions (new CLI flags / modes),
- cleaner test seams for config policy logic.

---

## Priority 3 — Reduce repetitive CMake test registration boilerplate

### Problem

`hexapod-server/CMakeLists.txt` contains many repetitive blocks:

- `add_executable(...)`
- `target_link_libraries(... hexapod_server_core)`
- `add_test(...)`

The explicitness is good, but repetitive patterns increase maintenance friction.

### Recommendation

Introduce helper function/macro, e.g.:

```cmake
function(register_server_test test_name source)
  add_executable(${test_name} ${source})
  target_link_libraries(${test_name} PRIVATE hexapod_server_core)
  add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()
```

Use explicit source lists but reduce redundant glue code.

### Expected payoff

- smaller diffs for new tests,
- fewer copy/paste mistakes,
- improved CMake readability.

---

## Priority 4 — Align visualiser schema documentation with implementation reality

### Problem

`docs/VISUALISER_TELEMETRY.md` says `schema_version` is required on every payload and missing schema should be ignored.

However, `hexapod-visualiser/telemetry_protocol.py` intentionally accepts recognized legacy payloads without `schema_version`, and tests enforce that behavior (`test_udp_protocol_accepts_legacy_packets_without_schema_version`).

### Recommendation

Choose and enforce one policy:

- **Option A (recommended):** Keep backward compatibility and update docs to explicitly describe legacy acceptance behavior and deprecation window.
- **Option B:** Enforce strict schema gate everywhere and remove legacy acceptance + adjust tests.

### Expected payoff

- avoids operator confusion,
- clarifies upgrade constraints between producer/consumer versions.

---

## Priority 5 — Improve logger backpressure observability during runtime

### Problem

`AsyncLogger` drops messages when queue capacity is exceeded. While dropped counts are visible at teardown, transient bursts during long runs may go unnoticed.

### Recommendation

- expose dropped-message counters in periodic runtime diagnostics,
- optionally break down by severity (warn/error/debug),
- add threshold-triggered warning emission.

### Expected payoff

- faster diagnosis of contention/perf issues,
- reduced blind spots during stress scenarios.

---

## 5. Additional Design/Code Health Notes

### 5.1 Keep protocol metadata as source of truth and expand generated artifacts

Given the mature metadata structure in `hexapod-common`, consider generating:

- protocol documentation tables,
- parser validation stubs,
- golden fixtures for host/firmware integration tests.

This would reduce manual drift between C++, Python parser logic, and docs.

### 5.2 Visualiser runtime organization is currently closure-centric

`hexapod-visualiser/server.py` is concise and works well now, but feature growth (auth, multi-stream playback, richer health endpoints) will be easier with a small runtime class encapsulating state and client/session management.

### 5.3 CMake hygiene tweak: remove unused includes

`hexapod-server/CMakeLists.txt` includes `FetchContent` but currently does not use it directly in this file. Removing unused CMake includes helps clarity.

---

## 6. Test and CI Assessment

### What is strong

- Server tests appear broad and layered (runtime, safety, transport, telemetry serialization, CLI mode selection, calibration fit, integration flow).
- Firmware host tests validate routing/handlers/dispatch without hardware dependencies.
- Scenario smoke verifies control loop behavior in simulator mode.

### What could be incrementally added

1. **Contract tests across components:** generate/verify golden packets for protocol and visualiser parser.
2. **Perf regression checks:** lightweight timing budget assertions in CI for hot loops.
3. **Negative integration tests:** stale telemetry, link drop/recovery, malformed repeated datagrams at scale.

---

## 7. Suggested Roadmap

## Phase 1 (1–2 weeks)

1. Schema policy alignment (docs + tests + parser behavior as decided).
2. CMake helper for server test registration.
3. Runtime diagnostics extension for logger drop counters.

## Phase 2 (2–6 weeks)

1. Extract `RobotRuntime` collaborators (telemetry/freshness/diagnostics units).
2. Split server bootstrap into config normalization + runtime factory.

## Phase 3 (6+ weeks)

1. Contract-first tooling for protocol/docs/fixtures generation.
2. Extended scenario fault injection matrix and resilience tests.

---

## 8. High-Impact “Next Actions” Checklist

- [ ] Decide visualiser schema compatibility policy and codify it.
- [ ] Create CMake test-registration helper in `hexapod-server`.
- [ ] Add periodic logger drop metrics to runtime diagnostics reporter.
- [ ] Open refactor design doc for `RobotRuntime` decomposition.
- [ ] Introduce bootstrap factory/config modules for `hexapod-server` app entrypoint.

---

## 9. Conclusion

The codebase is already in a **healthy, test-backed state** and appears production-minded for robotics iteration workflows. Recommended next work is mostly about reducing complexity concentration and tightening contract clarity so future feature work remains fast and safe.

The highest-leverage improvements are:

1. runtime decomposition,
2. telemetry schema policy clarity,
3. build/test maintainability cleanup.


---

## 5. Test Sweep Update (2026-03-26)

Ran the repository-wide verification entrypoint to satisfy the request to run all tests and check for obvious bugs:

```bash
./scripts/verify.sh
```

### Results

- `hexapod-server` preset `tests`: **34 / 34 passed**.
- `hexapod-client` preset `host-tests`: **3 / 3 passed**.
- Scenario smoke (`scenarios/01_nominal_stand_walk.toml`): **passed** with normal runtime diagnostics and clean shutdown summary.

### Bug check outcome

- No failing test, crash, assertion failure, or scenario abort was observed in this sweep.
- No obvious regressions were detected by default quality gates at this time.

