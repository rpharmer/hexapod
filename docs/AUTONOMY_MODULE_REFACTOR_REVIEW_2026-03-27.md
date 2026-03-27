# Autonomy module refactoring review (2026-03-27)

Scope: `hexapod-server/src/autonomy/modules/*` plus `autonomy_stack.cpp` orchestration.

## Summary

- Reviewed all 12 autonomy modules wired by `AutonomyStack`.
- Immediate high-value refactors are concentrated in cross-cutting orchestration duplication (`autonomy_stack.cpp`) and repeated contract-envelope plumbing.
- Most module shells are intentionally thin wrappers; for those, the recommendation is mostly consistency and maintainability improvements (small helpers, explicit policies, and reduced duplication), not behavior changes.

## Module-by-module findings

### 1) `mission_executive_module_`
- **Current shape:** module wrapper around mission lifecycle logic.
- **Refactor need:** **Low**.
- **Recommended action:** Keep as-is; only add helper methods if additional mission transitions make `AutonomyStack::step` grow further.

### 2) `mission_scripting_module_`
- **Current shape:** parsing/ingress adapter for mission scripts.
- **Refactor need:** **Low**.
- **Recommended action:** Keep current shell boundary. If grammar/features expand, isolate script-validation diagnostics into a dedicated formatter utility to avoid log/message duplication.

### 3) `navigation_manager_module_`
- **Current shape:** very thin pass-through (`computeIntent`) that stores `last_update_`.
- **Refactor need:** **Medium-Low**.
- **Recommended action:** Introduce a shared helper for “set-last-and-return” pattern used by several shells, or consistently mark these as deliberate façade modules in comments to reduce perceived redundancy.

### 4) `localization_module_`
- **Current shape:** observation validation + freshness check + estimate cache.
- **Refactor need:** **Medium**.
- **Recommended action:** Extract stale/validity policy into a named policy object (similar to local planner’s stale policy style) so timeout/frame rules are configurable and centrally testable.

### 5) `world_model_module_`
- **Current shape:** fuses localization and map slice into world snapshot state.
- **Refactor need:** **Medium**.
- **Recommended action:** separate pure data-fusion math/helpers from state mutation to improve test granularity and future process-split IPC serialization reuse.

### 6) `traversability_analyzer_module_`
- **Current shape:** policy-driven traversability scoring.
- **Refactor need:** **Medium**.
- **Recommended action:** isolate threshold constants and score composition terms into a small config struct with defaults to make policy tuning explicit and auditable.

### 7) `global_planner_module_`
- **Current shape:** contains substantial route-search and risk-proxy logic.
- **Refactor need:** **High**.
- **Recommended action:** split file into:
  1. grid/risk utility unit,
  2. route-search unit,
  3. module shell orchestration.
  This reduces cognitive load and simplifies targeted tests for each algorithmic layer.

### 8) `local_planner_module_`
- **Current shape:** short-horizon selection + stale-plan fallback + feasibility checks.
- **Refactor need:** **Medium**.
- **Recommended action:** promote magic constants into an exposed planner-policy struct (execution horizon, max step, blend gain, stale timeout) so tuning does not require code edits.

### 9) `motion_arbiter_module_`
- **Current shape:** thin adapter around arbiter select call.
- **Refactor need:** **Low**.
- **Recommended action:** no structural change needed now; maintain thin shell unless additional arbitration context is introduced.

### 10) `locomotion_interface_module_`
- **Current shape:** dispatch gate + write + status mapping + logging.
- **Refactor need:** **Medium**.
- **Recommended action:** extract status-construction helpers for the three paths (dispatched/dispatch-failed/suppressed) to reduce branch-local object construction duplication and improve readability.

### 11) `progress_monitor_module_`
- **Current shape:** thin wrapper with reset/evaluate cache.
- **Refactor need:** **Low**.
- **Recommended action:** keep as-is; only consider dedup helper shared with other shell wrappers.

### 12) `recovery_manager_module_`
- **Current shape:** thin wrapper with reset/onNoProgress cache.
- **Refactor need:** **Low**.
- **Recommended action:** keep as-is; optionally mirror progress-monitor style docs/comments for consistency.

## Cross-cutting refactors (highest ROI)

1. **Autonomy stack envelope/validation pipeline extraction (High).**
   `AutonomyStack::step` repeats internal envelope construction + validation for each stream. Introduce a small helper (or step-stage runner) to centralize this pattern and reduce maintenance risk when contracts evolve.

2. **Module shell boilerplate consolidation (Medium).**
   Several shells share identical “compute, store last value, return” behavior. A lightweight templated utility or explicit coding convention can reduce repetitive code while keeping boundaries clear.

3. **Policy surfacing and tuning knobs (Medium).**
   Planner/localization/traversability logic still relies on multiple file-local constants. Converting these into explicit policy structs improves testability, runtime configuration options, and reviewability.

4. **Algorithm decomposition in global planner (High).**
   Keep module shell small and move pathfinding internals to dedicated compilation units to improve compile times and unit-level ownership.

## Proposed implementation order

1. Refactor `autonomy_stack.cpp` envelope/validation repetition first (safest broad maintainability win).
2. Extract `global_planner` algorithm internals into dedicated helper units.
3. Surface planner/localization/traversability policy structs.
4. Optionally deduplicate thin-shell “last state cache” boilerplate.

## Out of scope for this pass

- Behavioral retuning of thresholds.
- Contract schema changes.
- Process boundary changes (true multi-process split).
