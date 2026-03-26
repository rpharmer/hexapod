# Documentation Index

This directory is the canonical documentation hub for the Hexapod monorepo.

## Architecture and codebase review

- `CODEBASE_REVIEW_2026-03-26.md` — monorepo-level architecture review, risks, and prioritized roadmap.
- `CODEBASE_SOURCE_FILE_MATRIX_2026-03-26.md` — detailed source-level inventory and follow-up notes.
- `SUBSYSTEM_CONTRACTS.md` — subsystem invariants, boundaries, and failure contracts.
- `contracts/autonomy/README.md` — v1 autonomy contract set (units/frames/timing/QoS/fault semantics).
- `DYNAMIC_GAIT_POLICY.md` — gait-planning policy behavior and tuning rationale.

## Runtime and protocol references

- `FIRMWARE.md` — wire protocol framing, command IDs, and firmware-side details.
- `VISUALISER_TELEMETRY.md` — telemetry schema and visualiser integration contract.
- `EXTENDING_IO_AND_HARDWARE.md` — how to add new control devices and hardware backends.

## Hardware reference

- `HARDWARE.md` — mechanical/electrical notes and build details.

## Maintenance guidance

When behavior changes, update docs in the same pull request and prefer linking to exact implementation files/tests from the changed subsystem.
