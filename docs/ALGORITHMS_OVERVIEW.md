# Algorithms Documentation Overview

This documentation set covers the algorithmic architecture used by:

- `hexapod-physics-sim` (simulation and physics solving)
- `hexapod-server` (runtime locomotion control and supervision)

## Reading order

1. `[ALGORITHMS_PHYSICS_SIM.md](ALGORITHMS_PHYSICS_SIM.md)`
2. `[ALGORITHMS_SERVER_LOCOMOTION.md](ALGORITHMS_SERVER_LOCOMOTION.md)`
3. `[ALGORITHMS_SERVER_CONFIG_TELEMETRY.md](ALGORITHMS_SERVER_CONFIG_TELEMETRY.md)`
4. `[ALGORITHMS_GLOSSARY.md](ALGORITHMS_GLOSSARY.md)`
5. `[ALGORITHMS_TRACEABILITY_CHECKLIST.md](ALGORITHMS_TRACEABILITY_CHECKLIST.md)`

## Primary source anchors

- Simulation step orchestration: `hexapod-physics-sim/src/core/world.cpp`
- Contact and narrowphase pipeline: `hexapod-physics-sim/src/core/world_collision.cpp`
- Solver stack: `hexapod-physics-sim/src/core/world_solver.cpp`
- Runtime control orchestration: `hexapod-server/src/control/robot_runtime.cpp`
- Locomotion pipeline: `hexapod-server/src/control/control_pipeline.cpp`
- Safety policy state machine: `hexapod-server/src/control/safety_supervisor.cpp`
- Command governor policy: `hexapod-server/src/control/command_governor.cpp`

## Cross-domain relationship

The server locomotion pipeline produces commanded joint targets and gait/contact intent assumptions. The physics simulator resolves contact/joint dynamics under those assumptions. Tuning and debugging should consider both documents together.