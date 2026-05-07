# Legacy Browser Visualiser Contract

This file is retained only as historical context for the retired browser visualiser.

The replacement workflow is the OpenGL renderer in
`[hexapod-opengl-visualiser/README.md](/home/volly/pico/hexapod/hexapod-opengl-visualiser/README.md)`
paired with the simulator notes in
`[hexapod-physics-sim/README.md](/home/volly/pico/hexapod/hexapod-physics-sim/README.md)`.

Do not treat this document as the current source of truth for visualisation or telemetry wiring.

## Current note

Current control-step and replay telemetry in `hexapod-server` now include explicit locomotion support diagnostics:

- `planned_stance`
- `raw_contact`
- `fused_support`
- `fused_contact_phase`
- `fused_contact_confidence`

Use the active sources of truth for those fields instead:

- `docs/ALGORITHMS_SERVER_CONFIG_TELEMETRY.md`
- `docs/ALGORITHMS_SERVER_LOCOMOTION.md`
- `hexapod-server/src/control/telemetry_json.cpp`
- `hexapod-server/src/control/replay_json.cpp`
