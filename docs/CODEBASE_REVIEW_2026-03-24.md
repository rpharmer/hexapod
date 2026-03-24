# Hexapod Codebase Review (2026-03-24)

## Executive summary

This codebase is in a healthy state for ongoing robotics iteration:

- architecture is separated by deployment boundary (`hexapod-server`, `hexapod-client`, `hexapod-common`),
- protocol contracts are centralized and increasingly metadata-driven,
- both server and firmware host test suites are passing in this environment.

The most valuable next refactors are **structural** rather than correctness fixes:

1. Decompose large orchestration files into smaller units with explicit responsibilities.
2. Isolate mutable runtime geometry and calibration side effects behind a dedicated service boundary.
3. Make loop execution/timing more cancellation-aware and deterministic under shutdown/load.
4. Improve parser maintainability by splitting section-level parsing and table validation.
5. Expand integration coverage around interactive runtime paths that currently mix IO + behavior logic.

### Delta since the 2026-03-23 review

The repo has already closed part of the prior review backlog:

- interactive CLI parsing and calibration behavior are no longer concentrated in `mode_runners.cpp`; they now live in dedicated units (`cli_options.cpp`, `interactive_input_mapper.cpp`, `interactive_calibration_actions.cpp`),
- server test coverage increased from 17 to 19 tests in this environment,
- firmware host tests are now discoverable in CTest (2/2 passing in this environment).

Given those improvements, the remaining highest-leverage work is now concentrated in hardware bridge transaction ergonomics, parser decomposition, and loop cancellation determinism.

---

## Scope and review method

### What was reviewed

- Top-level repository organization and project docs.
- Server runtime control, safety, hardware bridge, timing, and config parsing components.
- Firmware command dispatch/route metadata alignment.
- Existing tests and test build presets.

### Commands run

```bash
# repository scan
rg --files

# structural size sweep (excluding build artifacts)
python - <<'PY'
# counted .cpp/.hpp/.h/.c/.cc line counts and printed top files
PY

# server tests
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests --output-on-failure

# firmware host tests
cd ../hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

### Current verification status in this environment

- `hexapod-server`: 19/19 tests passed.
- `hexapod-client` host tests: 2/2 tests passed.

---

## Strengths observed

### 1) Clear layering and protocol cohesion

The repo boundary between host runtime, firmware, and shared protocol remains clean and practical for coordinated changes.

### 2) Metadata-driven command contract checks are a major reliability win

Firmware dispatch validates route coverage against shared command metadata via compile-time checks, reducing host/firmware drift risk when adding commands.

### 3) Safety logic already uses declarative rule composition

`SafetySupervisor` has moved toward rule-table style fault evaluation, which is easier to reason about than deeply nested conditionals.

### 4) Test coverage breadth is solid for core runtime behavior

Server tests cover runtime loop behavior, safety transitions, parser sections, transport behavior, scenario validation, and metadata alignment.

---

## Refactoring opportunities (prioritized)

## P0 — High-value maintainability changes

### A) Split oversized orchestration units by concern

#### Why

Several implementation units are still broad enough to increase merge conflict risk and make targeted changes expensive:

- `hexapod-server/src/hardware/hardware_bridge.cpp` (~406 LOC)
- `hexapod-server/src/config/toml_parser.cpp` (~362 LOC)
- `hexapod-server/src/control/mode_runners.cpp` (~356 LOC)
- `hexapod-server/src/control/calibration_probe.cpp` (~448 LOC)

#### Refactor shape

- `hardware_bridge.cpp`
  - extract `hardware_link_lifecycle.cpp` (handshake/heartbeat/link health)
  - extract `hardware_commands.cpp` (typed request/response wrappers)
  - extract `software_feedback_estimator.cpp` (synthesized joint feedback path)
- `mode_runners.cpp`
  - extract `cli_options.cpp`
  - extract `interactive_input_mapper.cpp`
  - extract `interactive_calibration_actions.cpp`
- `toml_parser.cpp`
  - extract section parsers (`runtime`, `transport`, `calibrations`, `tuning`, `geometry`)
  - isolate motor calibration table validation/sorting into one unit

#### Expected impact

- lower review surface area per change,
- less accidental coupling between interaction logic and calibration behavior,
- easier unit-level tests for section-specific parser behavior.

### B) Remove global mutable geometry writes from interactive mode

#### Why

Interactive calibration currently mutates active geometry dynamics directly in-place (`geometry_config::kHexapodGeometry`), which increases hidden state coupling and can create session-to-session reproducibility surprises.

#### Refactor shape

- Add `GeometryProfileService` (or equivalent) that owns active geometry mutations.
- Perform updates through explicit transactions (`preview`, `apply`, `rollback`, `persist`).
- Keep `activeHexapodGeometry()` read-only to most runtime modules.

#### Expected impact

- clearer ownership boundaries,
- safer calibration workflow integration,
- easier future persistence/versioning of tuned dynamics.

### C) Improve loop cancellation semantics and deterministic shutdown behavior

#### Why

`LoopExecutor` already has timing dependency injection, but the stop path still depends on external flag orchestration and thread join timing. It can be hardened for cancellation and test determinism.

#### Refactor shape

- Introduce a small cancellation token/source object (or C++20 stop-token equivalent facade).
- Ensure `sleep_until` can respond promptly to stop signals.
- Add deterministic tests around stop latency and period overrun behavior.

#### Expected impact

- fewer latent race/shutdown issues,
- more robust long-run operation when control loop load increases.

---

## P1 — Medium-term improvements

### D) Further isolate command transaction boilerplate in bridge API

The bridge still contains repeated command wrappers around request/decode/output mapping. Introduce typed command descriptors (command id + codec) to reduce repetitive wrappers and enforce consistent error logging semantics.

### E) Split parser validation policy from data extraction

`TomlParser` currently combines schema checks, fallback/range policy, data extraction, and calibration ordering in one flow. Move policy checks to small pure validators to improve readability and failure diagnostics.

### F) Separate controller event handling from robot command synthesis

`InteractiveRunner::run` currently interleaves controller polling, mode switching, calibration actions, state updates, and robot intent publication in one loop. Introduce a pure “event reducer” for state transitions and keep side effects in one place.

---

## P2 — Test and observability next steps

### G) Add focused tests for interactive calibration action routing

Current testing is strongest in core runtime and transport. Add tests that validate button-action mapping in calibration mode and guard against unintended behavior drift when modifying controller mappings.

### H) Add invariants around geometry mutation paths

If mutable geometry remains, add tests that assert invariants after any calibration update (positive taus, non-negative vmax, finite values, expected per-leg propagation behavior).

### I) Add loop executor stress tests

Expand timing tests to include:

- intentionally slow task steps,
- period overrun accounting,
- prompt cancellation under blocked/near-deadline conditions.

---

## Suggested implementation roadmap

### Sprint 1 (1 week)

1. Add tests for controller-event-to-action mapping and interactive calibration action routing.
2. Keep behavior identical with golden log/test assertions.
3. Begin extracting an event reducer so controller event interpretation remains side-effect free.

### Sprint 2 (1–2 weeks)

1. Introduce geometry profile service boundary.
2. Route all calibration writes through service methods.
3. Add apply/rollback/persist scaffold API (persistence can be stubbed initially).

### Sprint 3 (1–2 weeks)

1. Decompose `toml_parser.cpp` by section parser.
2. Add parser diagnostics improvements (per-section error context).
3. Preserve current config schema behavior.

### Sprint 4 (1 week)

1. Harden loop cancellation and add stress/cancellation tests.
2. Add telemetry around loop jitter and stop latency (optional but recommended).

---

## Success metrics

- Reduce top-4 runtime `.cpp` files by ~30% LOC without behavior changes.
- Keep server test suite green at 19/19 and firmware host suite green at 2/2 throughout refactor.
- Increase interactive mode coverage with dedicated tests for reducer/action mapping.
- Eliminate direct write sites to global geometry state outside one owning service.
