# Next Steps Roadmap

## Phase 0 (Immediate: 1–2 days)

1. **Establish review baseline in version control**
   - Keep this review and roadmap in-repo.
2. **Add lightweight code-quality guardrails**
   - Enable `-Wall -Wextra -Wconversion` for host build where feasible.
   - Add formatting/lint command to contributor workflow.
3. **Fix low-risk hygiene issues**
   - Correct logging typos and normalize terminology (`received`, `initialized`).

## Phase 1 (Short term: 1–2 weeks)

1. **Safety regression test harness**
   - Table-driven test cases for fault ordering and recovery hold behavior.
2. **Protocol call-site deduplication**
   - Introduce bridge helper APIs and migrate scalar/sensor/calibration commands.
3. **Loop diagnostics MVP**
   - Track loop execution time and overruns; print summary every diagnostics period.

## Phase 2 (Medium term: 2–4 weeks)

1. **Type model refactor for leg/joint state**
   - Unify raw vs processed joint containers and conversion logic.
2. **Config parser hardening**
   - Create schema descriptors and test fixtures for valid/invalid examples.
3. **Control/gait TODO burn-down**
   - Implement and test stance/swing foot placement policy and fallback speed estimator.

## Phase 3 (Longer term: 1–2 months)

1. **Simulation-first validation path**
   - Add a host-side simulated hardware bridge to validate control and safety behavior without robot hardware.
2. **Protocol compatibility strategy**
   - Add explicit protocol feature flags/capabilities negotiation in handshake.
3. **Operational readiness improvements**
   - Add structured status snapshots (JSON or compact binary log) for post-run analysis.

## Suggested ownership split

- **Runtime/Safety owner**: safety supervisor tests, loop metrics, command timeout policy.
- **Protocol/Comms owner**: bridge helper APIs, ACK/NACK consistency, firmware route metadata.
- **Config/DevEx owner**: parser schema utilities, contributor tooling, docs synchronization.
