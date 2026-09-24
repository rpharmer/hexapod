# Interactive navigation and moving ground-reference campaign

Date: 2026-09-24. Tree: `24cf1c0` plus the preserved dirty worktree. Scope:
visualiser ground reference, click-to-goal mapping, server command navigation,
and goal-directed progress in the WSL Pinocchio physics stack. The physics
solver and gait gates are not changed by this campaign.

## User-observed issues

1. The green terrain grid covers only a small area. A foot-position reference
   should remain beneath the robot as it travels.
2. Click-to-navigation does not behave reliably. Acceptance of a UDP command
   is not the same as reaching its goal.

## Current findings and changes

| Layer | Finding | Action/status |
|---|---|---|
| Ground display | The physics terrain patch is local and updates on correction packets, so its visible extent can leave the moving chassis. | Added a visual-only, world-fixed 10 cm reference grid in a 2 m window around the telemetry chassis. It follows the robot every display frame and leaves the measured terrain patch unobscured. It does not create collision terrain, map observations, or navigation free space. Visualiser test passes; on-screen check pending. |
| Ground click | GLFW cursor positions are logical window coordinates; the pick calculation used physical framebuffer dimensions. A scaled WSLg display therefore shifted the goal. The ray also assumed ground height zero. | Pick now uses window dimensions, the displayed plane height, finite/in-window checks, and a projection/unprojection test at 2× pixel scaling. Click-to-goal and drafted waypoints retain the current body yaw instead of silently demanding yaw zero. On-screen check pending. |
| Server command | `nav.goto` and `nav.waypoints` used `FollowWaypoints` defaults (initial pivot, 0.22 m/s drive cap, 0.55 rad/s yaw cap, four-second stall timeout), unlike the conservative live-physics acceptance profile. | Interactive commands now use the 0.05 m/s, 0.22 position-gain, no-entry-pivot, six-second-stall profile from the passing test, with yaw capped at 0.25 rad/s. The server command test checks these limits. |
| Start-cell waypoint | A* returned the current grid cell as waypoint zero and assigned it the path-tangent yaw. A goal behind the robot therefore commanded an about-face at its current position before any reverse translation. The heading did not settle, so the effective planar command stayed near zero while navigation reported Running. | Map-aware execution omits this already-reached cell when it is within the containing-cell radius. Explicit raw waypoints are unchanged. A reverse-goal unit test requires an immediate negative-X command. |
| Route blocking | The obstacle check only sampled between stored waypoints; a route with one destination checked no segment at all. | It now samples from the current pose through the remaining waypoints, including a single-destination route. Replan telemetry counts attempted plans even when a newly blocked route has no feasible replacement. Dynamic-obstacle replanning remains a regression gate. |
| Status | The UI called a successful `nav.goto` reply “applied,” which can be mistaken for completed travel. | It now says “accepted (not completed)” and shows navigation lifecycle/block reason near the command controls. |

## Goal-progress gap discovered by live replay

`physics_sim_navigation_acceptance` passes its `direct_path` case when there is
bounded *path length*, even if displacement toward the goal is nearly zero.
The test-local `--interactive-profile-direct` screen runs the same live plant
with WSL local-map settings and the interactive command profile. These are
single-run diagnoses, not a statistical qualification:

| Goal from settled pose | Profile variant | Max toward-goal progress | Path length | Final goal error | Lifecycle |
|---|---|---:|---:|---:|---|
| +X 0.14 m | interactive, WSL map | 0.090 m | 0.331 m | 0.050 m | Completed |
| −X 0.14 m, before start-cell fix | interactive, WSL map | 0.001 m | 0.470 m | 0.176 m | Running |
| −X 0.14 m | plus test integral gains | <0.001 m | 0.495 m | 0.162 m | Running |
| −X 0.14 m | faster 0.10 m/s / gain 0.75 | <0.001 m | 0.476 m | 0.172 m | Running |
| −X 0.14 m | old `FollowWaypoints` defaults | <0.001 m | 0.692 m | 0.235 m | Running |
| −X 0.14 m, after start-cell fix | interactive, WSL map | 0.090 m | 0.319 m | 0.050 m | Completed |

Repeat screen after the route-blocking fix: reverse interactive direct path
passed **5/5** on the same binary. Signed goal progress was 0.0900–0.0902 m,
final goal error 0.0499–0.0500 m, and path length 0.330–0.348 m. The final
error reflects the current local planner's roughly 50 mm goal tolerance; the
path length is still much larger than net progress, so this is a direction/
completion fix, not a claim of precise or efficient navigation.

The existing acceptance test uses a −X target but overrides map size,
inflation, search horizon, and integral gains; its earlier `direct_path`
sample passed with only 0.049 m max toward-goal progress versus a 0.14 m
target. This explains why “navigation tests green” did not establish useful
click-to-goal behavior. The decisive trace showed waypoint zero was the start
cell, with path-tangent yaw near pi on a reverse goal: effective planar speed
was essentially zero at steps 0, 1, 10, and 100 while the first waypoint stayed
active. Removing that redundant waypoint made the same live plant command about
−0.033 m/s at step zero and collect 0.090 m of signed reverse progress. This
is a navigation execution defect, not a reverse gait sign error. The old test
still needs a goal-progress gate; it should not pass merely by wandering.

Reproduce from `hexapod-server/` after sourcing
`../scripts/lib/pinocchio_env.sh` and setting
`HEXAPOD_PHYSICS_SIM_EXE=/home/volly/pico/hexapod/hexapod-physics-sim/build/hexapod-physics-sim`:

```text
./build-tests/test_physics_sim_navigation_acceptance --interactive-profile-direct
./build-tests/test_physics_sim_navigation_acceptance --interactive-profile-direct --interactive-profile-reverse
```

## Next batches and exit gates

1. **Done: short cardinal screen.** +X/−X/±Y each passed five empty-scene
   runs with goal-aligned command and progress; retain the reverse start-cell
   regression. This does not qualify obstacle paths or longer goals.
2. **Done: first-class short-goal gate.** The normal
   `physics_sim_navigation_directional` CTest requires signed progress,
   completion, XY/yaw accuracy, bounded cross-track/path travel, and a
   goal-aligned command. Path length alone cannot satisfy it. Safety, body
   height, support and solver-health gates remain unchanged.
3. **Test the actual UDP/UI path.** At normal and 2× window scaling, verify
   click marker and server-reported goal agree, command ref is accepted, nav
   lifecycle advances, cancellation works, and the goal remains reachable
   while the grid follows the chassis. Repeat with waypoint drafts and a
   missing/stale command endpoint. This requires a working GUI session; this
   host's WSLg copy mode currently prevents local click-through.
4. **Only then tune navigation.** If the signed command is correct but the
   plant stalls in a direction, compare command amplitude, governor scale,
   stance contacts, and net displacement before changing controller gains,
   planner policy, or gait. Re-run established navigation and locomotion
   suites after any such lever. Do not mark this campaign complete on an
   accepted UDP reply or a path-length-only CTest pass.

The moving green grid is a *visual ruler*, not a claim that collision terrain
has been generated outside the measured patch. A future true terrain-expansion
change belongs in the physics terrain/map pipeline and needs separate contact
and LiDAR validation.

## Batch N1 — choose translation versus facing (2026-09-24)

**Decision for short click-to-goal moves:** keep the current body heading and
use the hexapod's holonomic forward/reverse/strafe capability. Do not turn to
every path tangent. `nav.goto` with an explicit yaw is a pose goal: translate
first, then satisfy that final heading. If yaw is omitted, the command ingress
now captures the current estimated heading rather than silently using world
zero. An unavailable heading rejects the position-only command. Waypoints
without yaw inherit the current or last explicitly supplied heading.

The live 0.14 m empty-scene screens on the same physics stack gave:

| Goal | Policy | Signed progress | Path length | Final XY error | Final yaw error | Result |
|---|---|---:|---:|---:|---:|---|
| Left | Hold heading / strafe | 0.090 m | 0.328 m | 0.050 m | not recorded in first run | Completed |
| Right | Hold heading / strafe | 0.090 m | 0.327 m | 0.050 m | not recorded in first run | Completed |
| Left | Turn to path, translate, return to requested heading | 0.121 m | 1.010 m | 0.024 m | 1.021 rad | Still Running after the longer diagnostic window |

These are **single-run policy screens**, not lateral reliability or obstacle
qualification. The turn-first path also previously reported Completed with
1.382 rad of yaw error after 0.789 m of travel; the manager had accepted XY
proximity without checking yaw. Completion now requires both XY and yaw, and
an orientation-only final step is blocked if the inflated start cell is
occupied. A unit test covers yaw-only completion and the blocked rotation.
The turn-first experiment remains test-local and is not a selectable UI mode.

Four further empty-scene runs per forward and lateral direction were made after
the pose-completion fix. Together with the first runs, heading-preserving
forward, left and right each passed **5/5** (reverse passed 5/5 in the earlier screen):
signed progress stayed near 0.090 m, final XY error
near 0.050 m, path length 0.318–0.372 m, and the newly recorded final yaw
error 0.001–0.013 rad. This supports the short-click default in an empty
scene; it does not qualify obstacle corners or longer routes.

The project's separate matched locomotion tests found forward/reverse useful
progress around 0.207 m versus left/right strafe around 0.166–0.167 m for
comparable requests, so forward motion has an efficiency advantage *during
translation*. That does not repay a pair of slow, failure-prone turns on a
14 cm click. Longer routes may justify a facing preference, but only after
testing turn reliability, time-to-goal, energy/contact load, swept footprint,
and final-yaw completion. Nav2 likewise exposes omnidirectional motion and
rotate-to-heading as separate configurable choices, rather than assuming a
2D path tangent must be obeyed at every node: [MPPI Omni model](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/),
[Rotation Shim](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_rotation_shim_controller/).

### Navigation test inventory and unresolved gaps

The nine navigation-related CTests are `nav_primitives`,
`nav_locomotion_bridge`, `navigation_manager`, `navigation_matrix_lidar`,
`navigation_runtime`, `physics_sim_nav_waypoints`,
`physics_sim_navigation_acceptance`, `physics_sim_navigation_directional`,
and `command_channel`. All nine passed after the pose-completion,
intermediate-heading and optional-yaw changes. The older live acceptance suite
still permits `direct_path` to pass on total path length with negligible
goal-directed displacement; the new normal directional CTest closes that gap
for short empty-scene cardinal goals.

| Open issue | Missing proof / proposed gate |
|---|---|
| A* supplies path-tangent yaw at intermediate nodes, which previously caused `GoToPose` to stop and rotate even with `rotate_first=false`. | Holonomic map execution now holds the current heading at intermediate and horizon-limited nodes, retaining explicit yaw at the terminal goal; a synthetic corner unit test covers the mapping. A live obstacle-corner progress test is still needed. |
| Pose-goal completion now checks yaw, but the turn-first live screen was still Running with 1.021 rad of final yaw error after the longer window. | Dedicated live yaw-only and translate-then-final-yaw cases with bounded time, body path and safety; unit convergence alone is insufficient. |
| Goal completion can occur at roughly one 50 mm map cell, and live path length is 3–4× net progress on short clicks. | Separate XY, yaw, cross-track, path-efficiency, time-to-goal and stop-drift limits; do not use path length as a substitute for signed progress. |
| `LocalPlannerReplanPeriodS` is parsed but not used; replanning is event/stall driven. | Moving-obstacle and stale-map tests that distinguish a timely new plan from a safe Blocked/MapUnavailable result. |
| The local planner is 2D and obstacle inflation is a fixed circular margin, not a heading-dependent leg/turn swept footprint. | Test obstacle corridors and in-place turns against the full measured collision geometry before enabling turn-first near obstacles. |
| Command acceptance is not execution, and the actual WSLg click path has not been exercised end to end. | At 1×/2× scaling, compare clicked point, parsed goal and telemetry goal; verify cancellation, stale endpoint and authority handoff. |

The four cardinal directions are now registered as one normal
`physics_sim_navigation_directional` CTest using named limits in the gate
manifest. It requires actual completion, signed progress, bounded XY/yaw
error, cross-track error, path length and a goal-aligned command; neither
mere motion nor an accepted UDP reply can pass it. Follow with a
multi-waypoint corner and dynamic-obstacle campaign before offering any
optional “face travel” setting. Keep the current heading-preserving behavior
as the default; do not tune gait or loosen safety gates to compensate for a
navigation policy mismatch.

## Batch N2 — final-approach and late-obstacle audit (2026-09-24)

**Kept:** A* now checks the inflated start and goal cells before declaring a
within-cell goal reached. The navigation manager also refuses to complete a
pose goal when its planner reports Blocked or MapUnavailable. Previously, a
click at the current XY inside an inflated obstacle could be reported
Completed without an occupancy check. Planner and manager unit tests cover
that coincident occupied-goal case, including a safe zero-motion command.

**Rejected precision trial:** shrinking map-aware arrival from one 50 mm cell
to 30 mm and the interactive primitive's tolerance from 35 to 25 mm did reach
about 30 mm final XY error on all four cardinal directions. However, the
extra 20 mm of useful progress raised total body travel from roughly
0.32–0.37 m to 0.41–0.50 m. One full-suite run failed the unchanged 0.50 m
path-length gate at 0.5006 m. The trial was reverted; the normal 50 mm
completion behavior and movement profile remain in effect. Do not tighten
the click tolerance until short-goal path efficiency is understood and the
existing directional gate passes repeatedly. This was a navigation/plant
interaction cost, not a reason to loosen the path gate.

**Late-obstacle test repair:** `nav_midrun_intrusion.json` has a moving body,
but terrain contact damps its initial velocity; it is already visible in the
first map (typically six occupied cells) and does not reliably cross the
active route later. A scene-position/velocity-only screen could make the test
pass by increasing the initial path length without causing a late replan, so
that scene experiment was reverted. The live acceptance harness now adds a
test-only occupied map observation at 75% of the route *after* at least 40 mm
of goal-directed travel. It records initial occupied cells, initial segment
size, whether injection occurred, and progress at injection. The original
physics obstacle remains present and the no-physical-footprint-contact gate
is unchanged. A representative run injected at 40.0 mm, replanned from count
1 to 2, and ended Failed/StartOccupied with bounded 47 mm progress and no
physical-footprint contact. This is a deterministic **perception/replanning**
test, not proof that a moving physical obstacle can be dodged. A separate
physical crossing scenario remains needed.

The ten navigation-related CTests, now including `local_planner`, passed
10/10 after these changes. The live navigation-acceptance CTest also passed
three consecutive repeats. The actual WSLg click/UI path and a live physical
obstacle-corner route are still unverified. No solver, gait, map-inflation, or
navigation tolerance defaults changed in this batch.
