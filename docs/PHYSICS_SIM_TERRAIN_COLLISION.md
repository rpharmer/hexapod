# Physics Sim Terrain Collision Note

This note explains the serve-mode terrain collision change introduced in
`hexapod-physics-sim/src/demo/serve_mode.cpp` and gives a safe rollback path if
we need to revisit it later.

## Why this changed

In the serve-mode physics simulator, the terrain patch is made from many small
bodies. The old approach tried to retarget the robot bodies so they would
collide with the terrain bodies directly. That worked, but it also made flat
ground contact resolution depend on a large number of adjacent terrain boxes.

That coupling was noisy and expensive:

- contact solving had to consider a lot of tiny terrain shapes
- stepping the world in a single large update made the sim more sensitive to
  frame-time spikes
- the terrain grid is useful for sensing and visualization even when we do not
  want every cell to participate in collision response

The current change keeps the terrain grid available to the simulator, but
disables collision on the terrain bodies after the patch is created. In the same
serve loop, physics is also advanced in smaller substeps when needed so the
simulation stays more stable under larger incoming `dt` values.

## What changed

Relevant code paths:

- `hexapod-physics-sim/src/demo/serve_mode.cpp`
- `hexapod-server/src/control/safety_supervisor.cpp`
- `hexapod-server/src/control/physics_sim_estimator.cpp`

Key behavior changes:

1. Terrain bodies are still created, but their collision masks are set to zero
   after `TerrainPatch::initialize(...)`.
2. The old `RetargetRobotToTerrain(...)` helper was removed from serve mode.
3. Serve mode now slices large physics steps into smaller substeps before
   calling `world.Step(...)`.
4. The server-side safety check now treats missing power-state data as
   non-fatal for the motor-fault rule, instead of assuming the values are valid
   when they are not present.

## Why this is the preferred behavior

This keeps the simulator focused on two separate responsibilities:

- terrain bodies describe the world for sensing, preview, and map generation
- the physics solver handles robot motion and contact against the intended
  collision surfaces without being overloaded by the full terrain grid

That separation is easier to reason about and usually gives more stable serve
mode behavior.

## Rollback guidance

If we ever need to restore the previous terrain-contact behavior, the minimal
rollback is:

1. Reintroduce the old `RetargetRobotToTerrain(...)` logic in
   `hexapod-physics-sim/src/demo/serve_mode.cpp`.
2. Remove the `collisionMask = 0` loop that currently disables terrain-body
   collisions.
3. If sim timing issues reappear, also revert the substep logic and step the
   world once per incoming frame as before.

If the rollback is only meant to debug contact behavior, it is safest to do it
temporarily and compare against the current behavior using the same scene and
input trace.

## Validation checklist

After making any change here, confirm:

- the robot still stands and walks normally in serve mode
- terrain remains visible in preview and usable for sensing
- no unexpected motor-faults are triggered when power-state data is absent
- the server and simulator still pass the usual smoke and regression tests

