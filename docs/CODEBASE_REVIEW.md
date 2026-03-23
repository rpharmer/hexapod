# Codebase Review (March 23, 2026)

## Scope and method

This review covered repository structure, runtime architecture, serial protocol surfaces, build/test setup, and maintainability hotspots across:

- `hexapod-server` (host runtime, control loops, transport)
- `hexapod-client` (firmware command handling)
- `hexapod-common` (protocol/framing)

Validation baseline used the existing server test suite (`cmake --preset tests`, `ctest --preset tests`) to ensure recommendations align with current behavior.

## High-level strengths

1. **Clear layered runtime model on host side.**
   The server separates hardware, estimator, control, and safety loops with clear data exchange boundaries through runtime buffers and dedicated step methods. This structure is visible in `RobotRuntime::{busStep, estimatorStep, controlStep, safetyStep}` and the control pipeline handoff. It is a strong foundation for real-time evolution.  

2. **Good protocol centralization.**
   Protocol constants and payload-size invariants are centralized in `hexapod-common/include/hexapod-common.hpp`, reducing mismatch risk across host and firmware.  

3. **Command routing on firmware is trending in the right direction.**
   The route table in `hexapod-client/command_dispatch.cpp` reduces command spaghetti and enforces payload policy at dispatch boundaries.  

4. **Server-side automated tests are meaningful and fast.**
   The server includes unit/integration-like tests spanning runtime, safety transitions, hardware transport mocks, scenario validation, and kinematics consistency. This is a healthy baseline.

## Primary risks and maintainability hotspots

### 1) `SimpleHardwareBridge` is a broad "god object"

`hexapod-server/src/hardware_bridge.cpp` currently combines:
- transport session management,
- retry/telemetry policy,
- handshake protocol,
- command catalog semantics,
- raw-state codec translation,
- bridge lifecycle.

This concentration in one large source file increases cognitive load and makes targeted changes risky (e.g., changing retry policy now touches command semantics and handshake flow in the same unit).

**Impact:** higher regression probability, slower onboarding, difficult focused testing.

### 2) Freshness/safety logic is split across layers with overlapping concerns

`RobotRuntime::controlStep` applies stream freshness gating (timestamp/sample-id/age checks), while `SafetySupervisor` separately handles command staleness and latching semantics. They are valid individually, but policy interpretation is currently distributed, with different clocks and decision points.

**Impact:** future policy changes (timeouts, monotonicity, state transitions) may drift or produce unintuitive interactions.

### 3) Config parsing is robust but monolithic

`hexapod-server/src/toml_parser.cpp` handles schema validation, runtime mode, transport, calibrations, tuning, and geometry parsing in one large translation unit with repeated fallback/parser lambdas.

**Impact:** difficult to extend safely; repeated validation patterns invite inconsistent behavior over time.

### 4) `main` is carrying operational orchestration details

`hexapod-server/src/hexapod-server.cpp` handles logger bootstrapping, config parse, bridge selection, scenario mode, controller mode, command loop policy, and process shutdown in one function.

**Impact:** hard to test startup/runtime modes in isolation; feature additions are likely to increase complexity further.

### 5) Firmware global singleton context limits testability and determinism

`hexapod-client/firmware_context.cpp` exposes a global `FirmwareContext`, which simplifies bring-up but couples command handlers to global mutable state.

**Impact:** multi-instance simulation, deterministic tests, and dependency injection are harder than necessary.

### 6) Test coverage asymmetry between host and firmware

Server has nine tests; firmware currently has a single router test. Motion/power/sensing command handlers are lightly protected by tests.

**Impact:** protocol and payload handling regressions are more likely on firmware changes.

### 7) Documentation drift already visible

Root `README.md` references `docs/CODEBASE_REVIEW.md`, `docs/REFACTORING_REVIEW.md`, and `docs/NEXT_STEPS.md`, but those files were missing before this review.

**Impact:** discoverability and contributor onboarding degrade quickly when docs point to absent artifacts.

## Recommended strategic direction

- Preserve current behavior and performance characteristics.
- Refactor **by seams** (transport, policy, parsing, app modes) rather than broad rewrites.
- Grow coverage around protocol and safety boundaries first, then iterate on architecture.
- Use simulation/scenario mode as the main regression gate for behavior-preserving refactors.

## Target outcomes (next 1–2 milestones)

1. Reduce single-file complexity in host transport/config/startup path.
2. Make freshness and safety policy explicit and centrally testable.
3. Increase firmware command-handler test depth.
4. Keep docs synchronized with implemented architecture and operating workflows.
