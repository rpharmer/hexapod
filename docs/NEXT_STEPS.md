# Next Steps, Fixes, and Improvements

This plan turns the current codebase into a safer, more testable, and easier-to-operate platform.

## 1) Immediate fixes (high impact, low effort)

1. **Keep TOML dependency source unambiguous (already resolved)**
   - Status: `hexapod-server/include/toml.hpp.tmp` is not present in the repository.
   - Action: continue using external `toml11` and avoid reintroducing vendored temporary headers.
   - Why: a single authoritative TOML dependency path reduces maintenance confusion.

2. **Establish one canonical firmware build reference**
   - Action: keep exact build/flash details in one place (`docs/FIRMWARE.md`) and link from `README.md`.
   - Why: avoids drift between duplicated command blocks.

3. **Document a default runbook for bring-up**
   - Action: add a short “power-on checklist” for serial path, config validation, and safe shutdown.
   - Why: makes first boot and field recovery safer and faster.

## 2) Reliability and safety hardening

1. **Protocol-level resiliency tests**
   - Add unit tests for framing edge-cases:
     - truncated frame
     - invalid CRC/checksum
     - unknown command IDs
     - duplicate/out-of-order sequence handling (if used)
   - Outcome: reduces regressions in host↔firmware communication.

2. **Stricter calibration/config validation**
   - Validate all joint ranges and relationships (`min < max`, mechanical limits, servo index completeness).
   - Fail fast at startup with actionable error messages.
   - Outcome: prevents unsafe motion due to malformed config.

3. **Safety supervisor coverage expansion**
   - Add explicit handling for:
     - stale heartbeat
     - serial timeout bursts
     - estimator/controller divergence thresholds
   - Outcome: clear and deterministic transition to safe state.

## 3) Developer experience improvements

1. **Add a single top-level verification script**
   - Example: `scripts/verify.sh` to run formatting/lint/build checks for server+client.
   - Outcome: repeatable local checks and easy CI reuse.

2. **CI baseline**
   - Add GitHub Actions (or equivalent) for:
     - server configure/build
     - client SDK setup + firmware build
     - framing/unit tests
   - Outcome: catches integration breakage before merge.

3. **Tooling consistency**
   - Add `.clang-format` and apply formatting in touched C++ files.
   - Add a brief style section to docs to keep naming/include ordering consistent.

## 4) Architecture and maintainability

1. **Define module contracts in docs**
   - For each major server subsystem (`estimator`, `gait_scheduler`, `robot_control`, `hardware_bridge`), document:
     - inputs/outputs
     - update rate assumptions
     - ownership/lifecycle
   - Outcome: lowers onboarding cost and design ambiguity.

2. **Protocol spec completion**
   - Ensure one authoritative protocol doc includes:
     - packet formats
     - versioning strategy
     - backward compatibility expectations
     - recovery semantics after link reset
   - Outcome: safer future evolution and easier tooling interop.

3. **Configuration schema versioning**
   - Add `config_version` and migration notes.
   - Outcome: cleaner upgrades and less accidental breakage.

## 5) Suggested execution order (30/60/90 day view)

- **Next 30 days**
  - Keep TOML dependency guidance explicit in docs (no vendored `toml.hpp.tmp`).
  - Consolidate docs for firmware commands.
  - Add startup config validation and clearer failure messages.

- **Next 60 days**
  - Add framing unit tests and safety-supervisor scenario tests.
  - Introduce top-level verify script and CI baseline.

- **Next 90 days**
  - Finalize protocol spec/versioning policy.
  - Expand architecture docs and config migration strategy.

## Definition of done for this roadmap

- Any new feature or fix includes:
  - test or scripted verification,
  - documentation update,
  - explicit safety impact note.
