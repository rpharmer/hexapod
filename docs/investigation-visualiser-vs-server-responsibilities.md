# Investigation: visualiser responsibilities vs server responsibilities

## Scope
This investigation answers two questions:
1. Should pose fallback/legacy compatibility live in the visualiser?
2. Should ground-height determination be the server's responsibility?

## Findings

### 1) Runtime ownership split
- `hexapod-server` owns robot/runtime truth and publishes telemetry.
- `hexapod-visualiser` should render the telemetry and avoid reconstructing robot state semantics when avoidable.

### 2) Pose ownership and legacy payload handling
- Legacy payload compatibility and client-side dead-reckoning were creating split ownership for pose.
- For current visualiser features (world-anchored grid, waypoint overlays, mission distance), a complete pose is required.
- Therefore pose fields should be treated as required contract data (`x_m`, `y_m`, `yaw_rad`), and fallback pose inference should not live in the visualiser runtime path.

### 3) Ground-height ownership
Current renderer logic sets a stable ground plane from observed foot kinematics (`min(foot.z)`) and then reuses it.

Assessment:
- This is a **rendering baseline choice**, not authoritative robot/world state.
- Keeping this heuristic in visualiser is acceptable for UI continuity and does not affect robot control behavior.
- If terrain-aware accuracy is needed, server should publish an explicit terrain/plane estimate and visualiser should prefer that value.

## Conclusions
1. Pose estimation authority should be server-only.
2. Visualiser should require complete metric pose fields and remove legacy/fallback pose reconstruction paths.
3. Ground-plane baseline can remain a visualiser responsibility unless and until server publishes authoritative terrain-plane telemetry.
