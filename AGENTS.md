# AGENTS

Practical guidance for coding agents working in this repository.

## Scope and priorities

- Keep changes minimal, focused, and reviewable.
- Prefer updating existing docs/code over introducing parallel alternatives.
- Do not silently change command semantics in README examples; if behavior changes, update docs in the same change.

## Repo layout (high value areas)

- `hexapod-server/`: host runtime, control loops, config parsing, scenario runner.
- `hexapod-physics-sim/`: minphys3d simulation engine and demo/serve mode.
- `hexapod-opengl-visualiser/`: native UDP visualiser for sim packets and server telemetry.
- `hexapod-client/`: firmware and host-side firmware tests.
- `docs/`: architecture, algorithms, reference frames, and configuration references.

## Build and test commands

Run from repo root unless noted otherwise.

- Full repo verification: `./scripts/verify.sh`
- Server tests:
  - `cd hexapod-server`
  - `cmake --preset tests`
  - `cmake --build --preset tests -j`
  - `ctest --preset tests`
- Firmware host tests:
  - `cd hexapod-client`
  - `cmake --preset host-tests`
  - `cmake --build --preset host-tests -j`
  - `ctest --preset host-tests --output-on-failure`
- Physics sim build:
  - `cd hexapod-physics-sim`
  - `cmake -S . -B build`
  - `cmake --build build -j`
- Visualiser build:
  - `cd hexapod-opengl-visualiser`
  - `cmake -S . -B build`
  - `cmake --build build -j`

## Runtime/script workflows

- Sim scenario sweep: `scripts/run_server_scenarios.sh`
- Sim stack (server + visualiser): `scripts/run_sim_stack.sh`
- Server telemetry launcher: `scripts/run_server_with_telemetry.sh`
- Visualiser launcher: `scripts/run_visualiser.sh -- --udp-port 9870`

## Config and runtime facts to keep in sync

- `hexapod-server` runtime modes are `serial`, `sim`, and `physics-sim`.
- Hardware bridges currently include `SimpleHardwareBridge`, `SimHardwareBridge`, and `PhysicsSimBridge`.
- If adding/changing config keys, update:
  - parser/validation code in `hexapod-server/src/config/`
  - `docs/SERVER_CONFIG_REFERENCE.md` and/or `docs/PHYSICS_SIM_CONFIG_REFERENCE.md`
  - README command/config examples affected by the change

## Documentation expectations

- Normalize command context in docs (state whether commands run from repo root or component directory).
- Prefer short, accurate command blocks over long narrative.
- When adding new scripts or CLI flags, document at:
  - component README
  - top-level `README.md` when user-facing workflow changes
