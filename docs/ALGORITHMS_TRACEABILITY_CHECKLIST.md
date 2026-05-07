# Algorithms Traceability Checklist

Use this checklist to verify docs stay aligned with source behavior.

## `hexapod-physics-sim`

- Step order documented from `World::Step` in `src/core/world.cpp`.
- Broadphase tree/proxy/pair-cache behavior documented from `src/core/world_broadphase.cpp`.
- Narrowphase + manifold build documented from `src/core/world_collision.cpp`.
- GJK/EPA fallback path documented from `src/narrowphase/gjk.cpp` and `src/narrowphase/epa.cpp`.
- Block solver and island solving documented from `src/core/world_solver.cpp`, `src/solver/block2_solver.cpp`, `src/solver/block4_solver.cpp`.
- Island ordering documented from `src/solver/island_ordering.cpp`.
- TOI pipeline/sweep scope documented from `src/core/world_solver.cpp` and `src/core/world_private_methods.cpp`.
- Sleep logic documented from `src/core/sleep_system.cpp`.

## `hexapod-server`

- Runtime loop ownership documented from `src/control/robot_control.cpp` and `src/control/robot_runtime.cpp`.
- Control pipeline stage order documented from `src/control/control_pipeline.cpp`.
- Governor algorithm and outputs documented from `src/control/command_governor.cpp`.
- Supervisor fault policy/lifecycle documented from `src/control/safety_supervisor.cpp`.
- Freshness gate role documented from `src/control/robot_runtime.cpp` and `include/control/runtime_freshness_gate.hpp`.
- Navigation/bridge interaction documented from `src/control/navigation_manager.cpp` and `src/control/nav_locomotion_bridge.cpp`.
- Config-to-runtime mapping documented from `src/control/control_config.cpp`.
- Telemetry mapping documented from `src/control/telemetry_json.cpp`.

## Cross-check caveats

- README pipeline mismatch called out.
- Governor config wiring caveat called out.
- Supervisor/freshness terminology distinction called out.

## Doc set coverage pointers

- Index: `docs/ALGORITHMS_OVERVIEW.md`
- Sim algorithms: `docs/ALGORITHMS_PHYSICS_SIM.md`
- Server locomotion architecture: `docs/ALGORITHMS_SERVER_LOCOMOTION.md`
- Server config + telemetry map: `docs/ALGORITHMS_SERVER_CONFIG_TELEMETRY.md`
- Glossary: `docs/ALGORITHMS_GLOSSARY.md`