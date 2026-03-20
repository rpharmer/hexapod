# Next Steps Roadmap

Roadmap for executing the review recommendations in manageable phases.

## Phase 0 (Immediate: 1–2 days)

1. **Lock in documentation baseline**
   - Keep review, refactor plan, and roadmap versioned in-repo.
2. **Add lightweight code-quality guardrails**
   - Enable `-Wall -Wextra -Wconversion` where practical (especially host build).
   - Add a formatting/lint command to contributor workflow.
3. **Fix low-risk hygiene issues**
   - Normalize log terminology and typo cleanup (`received`, `initialized`, etc.).

## Phase 1 (Short term: 1–2 weeks)

1. **Safety regression harness**
   - Table-driven tests for fault ordering, latching, and recovery hold behavior.
2. **Protocol call-site deduplication**
   - Continue migrating bridge command handlers to shared request/decode helpers.
3. **Loop diagnostics MVP**
   - Capture loop execution timing and overrun counters.

## Phase 2 (Medium term: 2–4 weeks)

1. **Joint/leg type model refactor**
   - Unify raw vs processed containers and reduce duplicate conversion code.
2. **Config parser hardening**
   - Define schema descriptors and test fixtures for valid/invalid configs.
3. **Control/gait TODO burn-down**
   - Implement and validate stance/swing policy and conservative fallback behavior.

## Phase 3 (Longer term: 1–2 months)

1. **Simulation-first validation flow**
   - Expand simulated hardware coverage for control/safety regression testing.
2. **Protocol compatibility strategy**
   - Add explicit feature/capability negotiation patterns in handshake.
3. **Operational readiness improvements**
   - Emit structured status snapshots (JSON or compact binary) for post-run analysis.

## Suggested ownership split

- **Runtime/Safety owner**
  - Safety tests, loop metrics, timeout policy.
- **Protocol/Comms owner**
  - ACK/NACK consistency, command metadata, bridge helper APIs.
- **Config/DevEx owner**
  - Parser schema evolution, tooling, and docs synchronization.
