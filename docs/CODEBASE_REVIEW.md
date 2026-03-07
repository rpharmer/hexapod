# Codebase Review: Hexapod

## Scope reviewed

This pass reviewed every Markdown file and every C/C++ source/header file currently tracked in the repository:

- Markdown/docs: `README.md`, `docs/*.md`, `hexapod-client/README.md`
- Host/server code: `hexapod-server/include/*.hpp`, `hexapod-server/src/*.cpp`
- Shared protocol code: `hexapod-common/include/*.hpp`, `hexapod-common/framing.cpp`
- Firmware code: `hexapod-client/*.cpp`, `hexapod-client/*.hpp`
- Build/config files checked for consistency: `hexapod-server/CMakeLists.txt`, `hexapod-client/CMakeLists.txt`, `hexapod-server/config.txt`

## Executive summary

The codebase is in better shape than prior review notes suggested:

- both host and firmware targets build successfully in this environment,
- framing/protocol helpers are shared cleanly between host and firmware,
- server-side control stack modules are structured and compile together.

The most visible gaps are now documentation and onboarding consistency rather than obvious compile-time defects.

## What is working well

1. **Cross-target protocol reuse is solid**
   - `hexapod-common` provides a shared framing implementation used by both `hexapod-server` and `hexapod-client`, reducing divergence risk.

2. **Server architecture is modular**
   - Distinct components for estimator, gait scheduling, body control, hardware bridge, IK/FK, and safety supervision are separated into focused modules.

3. **Firmware command path appears defensive**
   - Firmware-side serial command handling validates packet framing and supports command-response patterns for host integration.

4. **Both primary builds are currently reproducible**
   - `hexapod-server` and `hexapod-client` configured and built successfully during this review.

## Findings

### 1) Top-level README is stale versus current repo layout (high)

`README.md` still describes files that are no longer present or no longer canonical (for example, references to `protocol.md` and a `makefile` path under `hexapod-server`) and omits the newer server module layout.

**Impact:** new contributors can be led to incorrect entry points and setup assumptions.

**Recommendation:** refresh `README.md` repository tree and build notes to reflect current CMake-first structure and actual file locations.

### 2) `hexapod-client/README.md` remains upstream boilerplate (medium)

Firmware folder README is generic Pimoroni boilerplate and does not document this project’s specific protocol commands, pin mappings, calibration flow, or flashing expectations.

**Impact:** increases onboarding and maintenance overhead for project-specific firmware behavior.

**Recommendation:** replace boilerplate README with project-specific firmware docs (build, flash, runtime protocol expectations, hardware assumptions).

### 3) Documentation overlap/duplication across `docs/` (medium)

`docs/FIRMWARE.md` and parts of `README.md` cover protocol/build behavior with partial overlap; divergence risk is high as behavior evolves.

**Impact:** future drift between docs and implementation.

**Recommendation:** make one document canonical for wire protocol (`docs/FIRMWARE.md`) and have other docs link to it instead of repeating command tables.

### 4) Minor language/format quality issues in docs (low)

`docs/HARDWARE.md` contains typos and inconsistent phrasing/formatting; technically understandable but less polished for external users.

**Impact:** low runtime impact, moderate readability cost.

**Recommendation:** do a docs polish pass (typos, sentence clarity, consistency for units and capitalization).

## Validation run in this review

- `hexapod-server` configure/build completed successfully.
- `hexapod-client` configure/build completed successfully (including pico toolchain integration in this environment).

## Suggested next actions

1. Update top-level `README.md` to match current code layout and build flow.
2. Replace firmware boilerplate README with project-specific instructions.
3. Reduce protocol documentation duplication by designating `docs/FIRMWARE.md` as canonical.
4. Perform a small editorial pass on `docs/HARDWARE.md` for typos and style consistency.
