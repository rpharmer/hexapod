# Whole-Body Proximal Contact Dynamics Plan

## Summary

Replace the hexapod’s fragmented per-leg ABA/contact treatment with a staged, measurable transition to whole-body dynamics using Pinocchio 4.1.

The production solver will use:

1. Whole-tree floating-base ABA to compute unconstrained motion.
2. Minphys3d collision detection and manifold generation.
3. Pinocchio `PointContactConstraintModel` constraints for unilateral Coulomb contacts.
4. `DelassusOperatorRigidBody` and `ADMMConstraintSolver` with proximal regularisation for contact impulses.
5. Semi-implicit integration of the resulting generalized velocity.

This is preferable to applying constrained ABA alone: foot contacts are unilateral frictional constraints, whereas constrained/loop ABA primarily handles equality constraints. Pinocchio 4.1 now provides both the articulated dynamics and proximal friction-contact machinery required for this design. [Pinocchio 4.1 release](https://github.com/stack-of-tasks/pinocchio/releases)

The migration will remain staged. The existing solver stays available as `legacy-pgs` for regression comparison. The WSL production configuration is `pinocchio-proximal` after the proximal stand CTest and exact-replay safety/behaviour/cadence gates; crouched PGS CTests in `./scripts/verify.sh` are a known baseline, not a switch blocker. How to close those remaining gates: [`PLAN_PINOCCHIO_DEFAULT_SWITCH.md`](PLAN_PINOCCHIO_DEFAULT_SWITCH.md). Allocator and ADMM follow-ups are [`PLAN_PINOCCHIO_PROXIMAL_RESOURCE_REDUCTION.md`](PLAN_PINOCCHIO_PROXIMAL_RESOURCE_REDUCTION.md) and [`PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md`](PLAN_PINOCCHIO_ADMM_DELASSUS_RETRIES.md); they are not substitutes for the locomotion/performance evidence.

## Implementation Changes

### 1. Preserve the movement baseline and correct actuator physics

- Create a dedicated checkpoint commit containing the current user-validated gait, controller, visualiser, tests, and documentation changes. Keep all subsequent physics work in separate commits.
- Record the currently intermittent walk-distance regression as a known baseline issue rather than attributing it prematurely to the new solver.
- Replace the unrealistic servo defaults with the selected MG996R-at-6-V model:
  - Stall torque: `1.471 N·m`.
  - No-load speed: `7.48 rad/s`.
  - Position-loop natural frequency: `25 rad/s`.
  - Damping ratio: `1.0`.
- Replace the hard, velocity-independent torque clamp with a motor torque-speed envelope:
  - When commanded torque assists current motion, available torque falls linearly to zero at the no-load speed.
  - Braking torque remains capped at stall torque.
  - Do not impose velocity through a constraint row; velocity must emerge from the motor model and mechanical load.
- Fix the existing torque/impulse unit error. Every iterative actuator row must clamp accumulated impulse to `available_torque × substep_dt`, never directly to torque.
- Keep structural joint-axis and anchor impulses independent from actuator torque limits. Their purpose is maintaining the revolute joint, not emulating motor output.
- Make servo warm starts timestep-aware:
  - Scale cached impulse by `new_dt / old_dt`.
  - Apply the existing decay after scaling.
  - Reproject it into the new torque-speed impulse limit.
  - Reset it on solver-mode changes, discontinuous target changes, lost contact identities, rollback, or invalid state.
- Update the saturation tests so they validate torque over elapsed time and produce equivalent motion at 120, 240, and 480 Hz.

### 2. Add the Pinocchio whole-body model

- Require Pinocchio `4.1.x` when `MINPHYS3D_ENABLE_PINOCCHIO=ON`; enable it by default for Linux production builds and retain `OFF` for legacy-only diagnostics.
- Extend the dependency setup to install Pinocchio from the official robotpkg Ubuntu repository into `/opt/openrobots`. Build and launch scripts will provide `CMAKE_PREFIX_PATH`, `PKG_CONFIG_PATH`, and `LD_LIBRARY_PATH` locally rather than modifying the user’s shell profile.
- Link through `pinocchio::pinocchio` and fail configuration with a clear install command when the requested solver is unavailable.
- Construct the Pinocchio model programmatically from the existing scene rather than maintaining a second URDF:
  - One free-flyer chassis root.
  - All 18 revolute joints and 18 leg links in the existing wire order.
  - Joint axes and parent placements copied from the existing servo-joint definitions.
  - Mass, centre of mass, and inertia copied from the corresponding minphys3d bodies.
- Create explicit, unit-tested mappings between:
  - Minphys3d world poses and Pinocchio free-flyer configuration.
  - Body twists and Pinocchio generalized velocity.
  - The 18 protocol servo indices and Pinocchio joint indices.
  - Pinocchio link poses and minphys3d collision/visual bodies.
- In Pinocchio mode, disable minphys3d’s internal robot servo and anchor constraint rows. The generalized coordinates preserve the robot tree; running both systems would double-apply constraints.
- Continue using minphys3d for terrain, obstacles, broad phase, narrow phase, manifold persistence, LiDAR, UDP serving, and visualisation.
- Initially support the production world used by the physics stack: the articulated hexapod contacting static terrain and static obstacles. A contact with an unsupported independently dynamic object must report `UnsupportedIsland` and hold the last valid state instead of silently using a partially coupled solver.

### 3. Implement proximal frictional contact stepping

For each physics substep:

1. Read the complete robot configuration and velocity into the Pinocchio state.
2. Convert servo target errors into generalized torques using the MG996R controller and torque-speed envelope.
3. Run whole-tree ABA with gravity and external forces to obtain the free velocity `v_free`.
4. Convert each persistent minphys3d manifold point into a Pinocchio point-contact constraint:
   - Use the manifold world normal to construct a stable contact frame.
   - Preserve the existing material friction and restitution values.
   - Apply the existing penetration slop and correction cap as a velocity-level bias.
   - Map static terrain and obstacles to Pinocchio’s universe joint.
5. Build the articulated Delassus operator and solve the nonlinear complementarity problem using `ADMMConstraintSolver`:
   - `solve_ncp = true`.
   - Spectral ADMM update rule.
   - Proximal regularisation `mu_prox = 1e-6`.
   - Absolute feasibility and complementarity tolerance `1e-8` on standing
     contacts. Sliding contacts (tangential free speed above 2 cm/s) use
     ADMM stop `1e-3`. Do not use `1e-3` as the standing ADMM stop.
   - Relative tolerance `1e-6`.
   - Delassus regularisation `1e-10`.
   - Maximum 50 iterations.
6. Warm-start impulses by persistent manifold ID, scaling them with timestep changes and projecting them back into the current friction cone.
7. Apply `M⁻¹Jᵀλ` to the free velocity.
8. Integrate using semi-implicit Euler: update velocity first, then call Pinocchio’s manifold-aware `integrate`.
9. Write the resulting link transforms and twists back to minphys3d for collision, sensing, and rendering.

Use `ConstraintCholeskyDecomposition` as the correctness oracle in tests. The production path remains `DelassusOperatorRigidBody`; it must agree with the dense oracle within the specified residual tolerances.

LCABA/constrained ABA will be exercised for bilateral-constraint validation, but will not replace the unilateral friction solver.

### 4. Safety, interfaces, diagnostics, and rollout

- Introduce `PhysicsSolverMode`:
  - `LegacyPgs = 0`
  - `PinocchioProximal = 1`
- Rename the existing reserved configuration fields in [physics_sim_protocol.hpp](/home/volly/pico/hexapod/hexapod-common/include/physics_sim_protocol.hpp) without changing `ConfigCommand` size:
  - `reserved_i32` → solver mode.
  - Four reserved floats → proximal `mu`, absolute tolerance, relative tolerance, and Delassus regularisation.
  - Existing `solver_iterations` becomes the active solver’s maximum iteration count.
- Add server configuration keys:
  - `Runtime.PhysicsSim.SolverMode`
  - `Runtime.PhysicsSim.SolverIterations`
  - `Runtime.PhysicsSim.ProximalMu`
  - `Runtime.PhysicsSim.AbsoluteTolerance`
  - `Runtime.PhysicsSim.RelativeTolerance`
  - `Runtime.PhysicsSim.ContactRegularization`
- Append solver diagnostics to `StateResponse`:
  - Status: `Healthy`, `RecoveredRetry`, `HeldLastGood`, or `UnsupportedIsland`.
  - Iteration count.
  - Final primal/complementarity residual.
  - Cumulative rollback count.
  - Rebuild the server and simulator together; mixed protocol binaries are not supported.
- Validate the state before integration, not only after it:
  - Reject non-finite state.
  - Reject chassis or link linear speed above `2 m/s`.
  - Reject angular speed above `10 rad/s`.
  - Reject non-finite impulses, residuals, or energy deltas.
- Snapshot the complete dynamic state at the start of every substep. On rejection:
  - Restore the snapshot.
  - Clear the affected contact and actuator warm starts.
  - Retry once as two half-substeps.
  - If either retry fails, restore the last valid state and return `HeldLastGood`; no corrupt pose may reach the server or visualiser.
- A recovered retry produces a warning but remains usable. `HeldLastGood` immediately invalidates that sample and inhibits gait commands; motion can automatically recover after 30 consecutive healthy responses with no safety fault.
- Add per-step and aggregate telemetry for:
  - Solver mode, iterations, primal/dual/complementarity residuals.
  - Peak normal, friction, structural, and actuator impulses.
  - Servo torque utilisation.
  - Pre-integration linear/angular speed.
  - Mechanical energy change and actuator work.
  - Warm-start resets, retries, rollbacks, held states, and unsupported islands.
  - Contact/manifold IDs involved in the worst residual.
- Roll out in four commits:
  1. Current locomotion checkpoint.
  2. Servo units, MG996R model, pre-integration guard, and diagnostics.
  3. Pinocchio model adapter plus dense-oracle tests.
  4. Proximal contact runtime, protocol/configuration, documentation, and default switch.
- Keep `legacy-pgs` for comparison for at least one release. Never fall back to it in the middle of a substep; changing equations silently would introduce another discontinuity.

## Test and Acceptance Plan

- Actuator tests:
  - Verify the impulse bound is `τ_available × dt`.
  - Verify stall torque, no-load speed, braking, saturation, gravity hold, and target reversal.
  - Compare joint trajectories at 120, 240, and 480 Hz; final angle and velocity must agree within 5%.
- Model mapping tests:
  - Round-trip 1,000 randomized valid poses and velocities.
  - Validate every joint’s axis, sign, zero offset, parent, inertia, and wire index.
  - Compare ABA against dense mass-matrix forward dynamics to `1e-9` relative error in double precision.
- Contact tests:
  - Single foot, tripod, all-six-foot stance, edge contact, duplicated contact, redundant contact, sliding, static friction, and turn-in-place.
  - Production articulated Delassus results must agree with the dense Cholesky oracle to `1e-8` absolute or `1e-6` relative residual.
  - Contact impulses must satisfy non-negative normal force and the Coulomb cone.
- Failure-injection tests:
  - NaN impulse, singular/redundant constraints, extreme penetration, stale warm start, timestep change, and excessive velocity.
  - Confirm rollback occurs before integration and no invalid state is published.
- Deterministic locomotion replay:
  - Capture the exact StepCommand stream for standing, forward, reverse, strafe, diagonal travel, turning in place, and command transitions.
  - Replay without wall-clock or controller-device dependencies.
  - Repeat across 100 perturbed initial-pose/contact-order seeds.
- Behaviour gates:
  - Stand for 60 seconds without non-finite values, rollbacks, or solver exhaustion.
  - Maintain commanded body height within `±10 mm`.
  - Keep stance-foot drift below `3 mm RMS` and penetration below `3 mm`.
  - Achieve at least 70% of integrated commanded translation and yaw after the acceleration transient.
  - Keep unintended lateral travel below 10% of path length plus `10 mm`.
  - During turn-in-place, keep chassis translation below `50 mm` over 10 seconds.
  - Produce equivalent displacement and yaw within 5% at 120, 240, and 480 Hz.
  - After the default switch, soak with a 10-minute randomized feasible-gait run (zero held states, non-finite values, or speed-limit violations). That soak is post-default, not a switch blocker.
- Performance gate:
  - Release build at 240 Hz under WSL.
  - Physics-step p99 below `4.0 ms`.
  - Nominal ADMM p99 at or below 20 iterations, with no 50-iteration exhaustion.
- Run the proximal production gates (60 s stand CTest at 0.14 m, exact-replay safety+behaviour including 100 seeds and 120/240/480 Hz) before changing the WSL default to `pinocchio-proximal`. Crouched PGS live-physics CTests and the offline scenario height envelope in `./scripts/verify.sh` are a known baseline, not a switch blocker.

## Assumptions and Defaults

- The target robot remains the built-in 18-servo hexapod with static terrain and obstacles.
- MG996R 6 V specifications are the production actuator defaults; later measured torque curves can replace them without changing solver architecture.
- Pinocchio 4.1 is an intentional production runtime dependency.
- Double precision remains mandatory throughout dynamics and constraint solving.
- Minphys3d remains responsible for collision detection, terrain, sensors, networking, and rendering.
- The Pinocchio proximal solver is the WSL default after those proximal gates. `legacy-pgs` (`SolverMode = 0`) remains for comparison. `./scripts/verify.sh` PGS/offline misses do not block the switch.
- Existing uncommitted locomotion and visualiser work is preserved and checkpointed separately; no reset or unrelated cleanup is included.
