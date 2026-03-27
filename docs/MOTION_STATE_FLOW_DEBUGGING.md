# Motion-State Flow (Kinematics, Estimation, Telemetry)

This note documents the control/estimation/telemetry path used during simulator scenarios, with a focus on diagnosing mismatches between rendered world motion and leg motion.

## End-to-end flow

1. **Motion intent ingestion**
   - `MotionIntent` is produced by CLI/scenario/autonomy layers.
   - Key fields: `speed_mps`, `heading_rad`, and `body_pose_setpoint`.

2. **Gait and body target generation**
   - `GaitScheduler` computes stance/swing timing and gait phase.
   - `BodyController` computes per-leg foot targets in body frame.
   - `RobotControl` converts these into joint targets.

3. **Simulator hardware bridge**
   - `SimHardwareBridge` applies a first-order lag from joint target to simulated joint state.
   - This updates **joint angles**, but does not directly advance a world-frame body pose.

4. **Estimator**
   - `SimpleEstimator` computes body orientation/height from contact geometry.
   - It computes translational velocity only from `body_trans_m` deltas.
   - In simulator mode, without external localization/body pose input, planar `body_trans_mps.{x,y}` can remain near zero.

5. **Telemetry publisher**
   - Publishes:
     - joint angles (`angles_deg`) for rendering limb pose;
     - `dynamic_gait` state including estimated body velocities;
     - `autonomy_debug.current_pose` used by visualiser for world/grid anchoring.
   - `autonomy_debug.current_pose` falls back to dead-reckoned odometry when localization pose is absent.

6. **Visualiser**
   - Renders legs from `angles_deg`.
   - Moves grid/world overlays from `autonomy_debug.current_pose` (or fallback sources).

## Investigated mismatch: fast world/grid vs slow feet

Observed in `scripts/run_sim_stack.sh --scenario scenarios/05_long_walk_observability.toml`:
- feet appeared to progress slowly, while grid/world pose advanced faster.

Root cause:
- telemetry fallback odometry integrated **commanded** speed/heading when estimated planar velocity was zero.
- this can report world motion that does not match actual kinematic motion represented by joint trajectories.

Fix implemented:
- fallback odometry now integrates only finite, non-zero **estimated** planar velocity.
- commanded intent no longer drives fallback pose integration by itself.

## Suggested regression test matrix

1. **Telemetry consistency tests**
   - If estimated velocity is zero and only commanded velocity is non-zero, `autonomy_debug.current_pose` should not drift.
   - Timestamp regressions and long pauses should reset fallback odometry.

2. **Kinematic-vs-pose consistency tests**
   - In simulator mode, compare:
     - rendered/world pose displacement from telemetry,
     - stance-leg implied displacement (from FK over time).
   - Require bounded divergence over scenario windows.

3. **Scenario-level observability tests**
   - For long-walk scenarios, assert:
     - non-trivial gait phase progression,
     - coherent relationship between estimated body velocity and joint/stance progression.

4. **Explicit provenance in telemetry**
   - Include a source tag for world pose (`localization`, `estimator`, `fallback_odometry`) to simplify triage in UI and tests.

