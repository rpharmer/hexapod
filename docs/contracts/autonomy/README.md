# Autonomy Contracts (v1)

This directory contains the contract-first interface set for autonomy modules and their supervision/runtime boundaries.

## Files
- `common_types.md`
- `mission_executive.md`
- `navigation_manager.md`
- `recovery_manager.md`
- `motion_arbiter.md`
- `localization.md`
- `world_model.md`
- `traversability_analyzer.md`
- `locomotion_interface.md`
- `process_supervision_and_ipc.md`
- `contract_review_checklist.md`
- `traceability_matrix.md`

## Current implementation scope (2026-03-27)

- v1 contract semantics (units, frames, timing/QoS/fault behavior) are implemented in autonomy module flows and validated in tests.
- Supervision and IPC contracts are represented as explicit runtime metadata and in-process enforcement in `AutonomyStack`.
- The repository still tracks a follow-on milestone for true multi-process runtime split while preserving these same contract boundaries.

## Maintenance guidance

When behavior changes:
1. update the relevant contract document;
2. update the corresponding tests;
3. update `PLAN.md` status to keep implementation and roadmap aligned.
