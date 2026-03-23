# Codebase Review (March 23, 2026)

## Executive summary

The codebase is in good operational health: boundaries between host runtime (`hexapod-server`), firmware (`hexapod-client`), and protocol (`hexapod-common`) are clear, and the host test suite is broad and currently passing (14/14).

The highest-value improvements are maintainability refactors that preserve behavior while reducing duplicated logic and decision-surface complexity:

1. **Decompose `RobotRuntime::controlStep`** into pure decisions plus side-effect orchestration.
2. **Unify host command transaction/decode patterns** in `SimpleHardwareBridge`.
3. **Consolidate command metadata** (names + payload contracts) into one protocol-owned source.
4. **Reduce firmware dispatch wrappers and centralize payload/NACK helpers**.
5. **Add explicit resilience tests around retry policy and stale-stream transitions**.

---

## Review method

- Read architecture and workflow docs at root and component level.
- Inspected key runtime and transport files in server/firmware paths.
- Verified baseline by running server test preset and full `ctest` suite.

Validation commands run:

- `cd hexapod-server && cmake --preset tests`
- `cd hexapod-server && cmake --build --preset tests -j`
- `cd hexapod-server && ctest --preset tests --output-on-failure`

Result: **14/14 tests passing**.

---

## Strengths worth preserving

### 1) Architectural layering is explicit and practical

- Runtime stages (`busStep`, `estimatorStep`, `controlStep`, `safetyStep`, `diagnosticsStep`) are separated and easy to reason about at a high level.
- Command transport responsibilities are separated between `TransportSession`, `CommandClient`, handshake, and bridge APIs.
- Shared protocol constants are centralized in `hexapod-common`, reducing host/firmware drift risk.

### 2) Freshness gating is safety-oriented

- Control gates outputs when estimator or intent freshness fails.
- Runtime logs both gate rejects and periodic metrics (stale counts, jitter, diagnostics counters).

### 3) Firmware route table is declarative

- Firmware command dispatch uses compile-time route declarations with payload policy contracts.
- Dispatch behavior is easier to inspect than ad-hoc command branching.

---

## Refactoring opportunities (ranked)

## P0 — Split `RobotRuntime::controlStep` into testable decision units

### Evidence

`controlStep` currently blends:

- timing metrics updates,
- state reads,
- freshness evaluation,
- stale counters,
- status construction for stale paths,
- pipeline invocation,
- write-back side effects.

### Why this matters

- One function currently carries multiple reasons to change (timing policy, freshness policy, status semantics, command emission).
- Edge-case tests need broad fixture setup instead of directly testing pure decision logic.

### Refactor direction

Extract helpers such as:

- `evaluate_control_gate(now, est, intent)` -> pure gate decision + diagnostics reason.
- `build_stale_status(...)` -> pure status synthesis.
- `emit_nominal_or_safe_idle(...)` -> side-effects only.

### Success criteria

- `controlStep` becomes an orchestration shell with minimal branching.
- New unit tests cover stale estimator / stale intent / both stale matrix at decision-helper level.
- No behavior regressions in `test_robot_runtime_freshness_gate_matrix`.

---

## P0 — Introduce typed command helpers in `SimpleHardwareBridge`

### Evidence

Bridge methods like `get_current`, `get_voltage`, `get_sensor`, `get_angle_calibrations`, and servo-state commands repeat the same request/decode/fail pattern with minor variations.

### Why this matters

- Repetition invites inconsistency in error handling, labels, and future retry-policy evolution.
- Adding new commands remains more expensive than necessary.

### Refactor direction

Create a thin templated helper API:

- `request_ack(cmd, payload, label)` (already present; keep)
- `request_decoded<T>(cmd, payload, decoder, out, label)` (expand usage + shared decode adapters)
- Optional `CommandTraits<Cmd>` for command id, label, payload schema, decoder type.

### Success criteria

- Most scalar/query command methods collapse to one-liners.
- Uniform logging fields for all failures (`command`, `outcome`, `nack_code`, `payload_size`).
- Existing hardware bridge tests continue to pass.

---

## P1 — Single source for command metadata across host + firmware

### Evidence

- Host `CommandClient::command_name` has a switch table.
- Firmware command route table separately defines payload expectations.
- Protocol definitions exist in `hexapod-common`, but metadata is not fully centralized.

### Why this matters

- New command introduction requires multi-file synchronized edits.
- Logging labels/payload contracts can drift from protocol ID definitions.

### Refactor direction

Adopt a shared metadata table (e.g., X-macro) in `hexapod-common` with, at minimum:

- command id
- canonical command name
- default payload policy (if applicable)

Use it to derive host name lookup and firmware payload policy declarations where feasible.

### Success criteria

- One authoritative command-name source.
- Consistency test fails if metadata and IDs drift.

---

## P1 — Simplify firmware dispatch adapters and payload-error paths

### Evidence

`command_dispatch.cpp` uses wrapper templates (`routeNoPayload`, `routeWithPayload`) and a local invalid-payload responder while each domain handler still performs additional decode/error branching.

### Why this matters

- Still some boilerplate overhead for adding/adjusting commands.
- NACK policy consistency can degrade over time if decode checks diverge.

### Refactor direction

- Keep route table shape, but standardize decode+NACK helper utilities shared by motion/sensing/power handlers.
- Consider a small handler adapter that returns typed decode results with explicit error codes.

### Success criteria

- Fewer hand-written payload-length/parse branches in handlers.
- NACK codes for decode failures are uniform and test-covered.

---

## P2 — Improve resilience and observability tests

### Evidence

Current tests are strong for control and transport basics, but there is room for targeted resilience cases:

- retry-exhausted behavior under repeated transport timeouts,
- heartbeat/link re-establish edge cases,
- stale-stream counters/log cadence checks.

### Refactor direction

Add focused tests around:

- `CommandClient::transact` retry transitions,
- `SimpleHardwareBridge::ensure_link` timeout/heartbeat paths,
- diagnostics counter monotonicity under synthetic stale inputs.

### Success criteria

- Explicit tests for timeout -> retry_exhausted mapping.
- Link timeout recovery behavior covered by deterministic mocks.

---

## Suggested implementation roadmap

### Week 1 (highest impact)

1. `RobotRuntime` control-step decomposition + tests.
2. `SimpleHardwareBridge` typed decode/helper consolidation.

### Week 2

1. Shared command metadata table in `hexapod-common`.
2. Host `command_name` migration to shared source.
3. Firmware route metadata alignment where practical.

### Week 3

1. Firmware decode/NACK helper consolidation.
2. Resilience/telemetry test additions.
3. Documentation refresh with “adding a command” checklist.

---

## Risks and mitigations

- **Risk:** behavioral regressions in safety gating while decomposing control logic.
  - **Mitigation:** lock behavior with existing freshness matrix tests before refactor and expand matrix coverage first.

- **Risk:** template-heavy command helper abstractions hurt readability.
  - **Mitigation:** prefer a small traits table + straightforward wrappers; avoid deep metaprogramming.

- **Risk:** metadata unification touches host + firmware simultaneously.
  - **Mitigation:** migrate in two phases: generate host name map first, then route payload contracts.

---

## Immediate next steps (actionable)

1. Create a short design note for `controlStep` decomposition boundaries and invariants.
2. Add two new failing tests first:
   - retry exhausted after N timeouts,
   - stale estimator + stale intent status precedence.
3. Refactor one vertical slice (`get_current`, `get_voltage`, `get_sensor`) to prove bridge helper design before bulk migration.
4. Add a metadata prototype in `hexapod-common` that only drives host `command_name` to validate approach with minimal risk.

