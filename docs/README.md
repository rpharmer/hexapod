# Documentation Index

This directory is the canonical documentation hub for the Hexapod monorepo.

## Architecture and codebase review

- `../PLAN.md` — current autonomy implementation status and next execution focus.
- `CODEBASE_REVIEW_2026-03-27.md` — current monorepo major-unit review, risks, and prioritized roadmap.
- `SUBSYSTEM_CONTRACTS.md` — subsystem invariants, boundaries, and failure contracts.
- `contracts/autonomy/README.md` — autonomy contract set and implementation scope notes.
- `DYNAMIC_GAIT_POLICY.md` — gait-planning policy behavior and tuning rationale.

## Runtime and protocol references

- `AUTONOMY_OPERATIONS_RUNBOOK.md` — startup/supervision/degraded-mode/safe-stop operations, HIL acceptance, and reliability campaign procedures.
- `FIRMWARE.md` — wire protocol framing, command IDs, and firmware-side details.
- `VISUALISER_TELEMETRY.md` — telemetry schema and visualiser integration contract.
- `EXTENDING_IO_AND_HARDWARE.md` — how to add new control devices and hardware backends.

## Hardware reference

- `HARDWARE.md` — mechanical/electrical notes and build details.

## Maintenance guidance

When behavior changes, update docs in the same pull request and prefer linking to exact implementation files/tests from the changed subsystem.
