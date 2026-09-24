# Visualiser Control Room usability campaign

Started: 2026-09-24
Status: U1/U2 code complete; U3 WSLg click-through blocked by host copy mode;
U4 first observation slice implemented, field coverage and trace comparison open.
Scope: make the OpenGL visualiser's existing controls safe, reachable and clear
without moving authority out of `hexapod-server` or changing the locomotion
solver, gait or safety thresholds.

This tracks the *user-facing controls*. [VISUALISER_COMMAND_CHANNEL.md](VISUALISER_COMMAND_CHANNEL.md)
defines the UDP protocol and server authority model;
[LOCOMOTION_PROGRESS_CAPACITY_CAMPAIGN.md](LOCOMOTION_PROGRESS_CAPACITY_CAMPAIGN.md#batch-a12--start-live-visualiser-command-qualification-2026-09-24)
records why trustworthy interactive trials matter to the walking campaign.

## Current baseline

- The visualiser has scenario list/run/stop, click-to-goal, draft waypoints,
  nav cancel, and idle motion controls. The server remains the final authority.
- A12 removed the former 750 ms render-thread reply wait. The client now shows
  pending status and accepts only matching `command_result.ref` apply replies;
  stale replies and timeouts are handled. All 11 visualiser CTests pass.
- A local headless `sim` server returned matching live replies for
  `scenario.list`, `scenario.stop`, and idle `motion.set`. That does **not**
  establish that the actual WSLg buttons, ground picking, focus, and feedback
  work comfortably together. No GUI click-through is recorded yet.
- The default visualiser window is 1280×720. The Control Room now defaults to a
  bounded 440×620, scrollable panel with task-oriented sections. It has not
  been visually measured under WSLg yet.

## Issue register

| ID / priority | Evidence and user impact | Required result | Status |
|---|---|---|---|
| CR-1 / P0 | **Stop motion** sent unsupported `mode:"IDLE"`. | Ordinary action is **Stand & hold**, not emergency stop or torque-off. | Fixed in client/UI; exact payload and server parser tested. |
| CR-2 / P1 | **Stop scenario** was hidden until a nonempty list arrived. | Stop is independent of list availability; active authority and apply result visible. | Fixed in UI; GUI qualification pending. |
| CR-3 / P1 | Auto-sizing Control Room could obscure the scene. | Bounded, scrollable sections with Stop/Cancel at the top; inspect two window sizes. | Code complete; WSLg visual inspection blocked. |
| CR-4 / P1 | UDP send was easy to mistake for server application; stale authority was not obvious. | Per-ref pending/result history, reply/telemetry age, stale authority unknown. | Code complete; GUI qualification pending. |
| CR-5 / P2 | Motion action hid WALK/TRIPOD and lacked a Stand choice. | Explicit Walk (TRIPOD) and Stand & hold actions; show values and disabled reason. | Code complete for supported actions; no unsupported gait choices added. |
| CR-6 / P2 | No compact planned-versus-measured observation panel. | Fresh server-owned contact/foot data; missing fields marked `n/a`. | First panel slice implemented; aggregate simulator servo utilisation available, per-joint headroom, reach margin and actual speed remain open. |
| CR-7 / P1 | Server sends `active_fault` as a string, but legacy overlay parsed only an integer, silently displaying `NONE`. | Parse named faults; missing or unknown fault must not appear healthy. | Fixed; name mapping tested. |
| CR-8 / P1 | The command wireframe is deliberately hidden when a measured scene exists, but the View section was collapsed. Worse, pose-only UDP entities counted as measured geometry even when the viewer missed their one-shot shape descriptions, leaving neither robot drawable. | Make the active robot view obvious; keep command wireframe as fallback until measured geometry is drawable; refresh shape packets for late/lossy viewers. | Fixed; user confirmed the robot is visible on screen. |
| CR-9 / P1 | User sees a floating blue grid and incorrectly angled command legs. The command model used a mount rotation 90° from server FK and stale fallback hip locations/lengths; the blue infinite plane and local terrain grid could both be drawn over the same area, while one terrain-grid pass ignored the world-fixed origin. | Match server servo-to-joint FK and geometry; draw only one local ground grid when terrain is available; use the same grid origin on both axes. | Code and visualiser CTests pass; on-screen qualification of both appearances pending. |
| CR-10 / P1 | User wants a persistent foot-position ruler and reliable click-to-goal. The finite terrain patch does not cover travel; scaled-window picking and server navigation acceptance hid a start-cell waypoint defect. | Follow the robot with a visual-only ground reference; verify clicked world goals and actual signed travel separately. | Moving grid and DPI pick tests pass; omitting the already-reached start cell restored a test-local −X goal to 9 cm signed progress in 5/5 runs. On-screen click and wider directional checks remain in [navigation campaign](NAVIGATION_INTERACTION_CAMPAIGN.md). |

## Work order and gates

### Batch U1 — correctness and reachable stop controls

- [x] Adopt the recommended ordinary stop semantics: **Stand and hold** is a
  different action from **Safe idle**. Keep emergency stop/safety faults outside
  this convenience UI. The button now uses Stand and hold;
  expose Safe idle separately only after its support/torque behaviour is
  verified on the intended hardware and simulator.
- [x] Replace the unsupported `IDLE` payload and add a test proving the
  visualiser sends a server-accepted mode with the intended result.
- [x] Make Stop scenario independent of scenario-list state in the UI. The
  client test covers successful, empty and timed-out lists; actual GUI cases
  await U3.
- [x] Rebuild visualiser and server test targets; run `test_command_client`,
  `test_command_channel`, and all visualiser CTests. No gait or solver change.

### Batch U2 — status and layout

- [x] Distinguish **sent/pending**, **applied**, **rejected**, and **timed out**
  per request. Retain a short visible history or pending count; do not report a
  UDP send as a successful application.
- [x] Expose telemetry age and endpoint health separately from robot fault and
  command authority. Stale/missing authority must not masquerade as idle.
- [x] Group View, Scenarios, Navigation, Motion and Diagnostics into bounded
  sections/tabs. Keep scenario Stop and nav Cancel reachable without scrolling
  through camera sliders or geometry details.
- [ ] Verify the Control Room does not cover the ground-pick target area at
  1280×720 and remains usable when the window is smaller. WSLg screenshot and
  input are blocked on this host by copy mode, not by a command reply timeout.

### Batch U3 — real interactive qualification

- [ ] In WSLg, launch the local stack with command channel enabled and click
  List → Run → Stop on a stock scenario. Confirm each reply ref and observed
  authority/mode transition; no callback may freeze the render loop.
- [ ] Test ground click → nav goal → cancel, waypoint draft/send, and idle
  motion → ordinary stop. A rejected action must remain visibly rejected.
- [ ] Verify scenario authority suppresses gamepad motion and restores it on
  stop; verify nav/scenario restrictions are enforced by the server even if
  telemetry is delayed.
- [ ] Repeat with the command endpoint absent, a stale reply, and stale
  telemetry. The scene/camera must remain responsive and no command should be
  shown as applied without a matching server result.

### Batch U4 — observation panel (after command UI is trustworthy)

- [x] Inventory existing telemetry for planned/measured feet and contacts,
  reach, requested/governed/actual speed, joint target/measured rate, and motor
  headroom. `locomotion_debug` already has planned/raw/fused support, foot
  positions, commanded tracking error and target-clamp distortion. The
  governor owns requested/governed speed, now serialized. Physics-sim reports
  peak aggregate servo torque utilisation, now forwarded through server
  telemetry. Actual planar speed, reach margin, joint rate and **per-joint**
  motor headroom are not in the live JSON and are shown as `n/a`, not inferred
  from commands or defaulted to zero.
- [x] Add a first per-leg observation panel with freshness and units; suppress
  stale/missing foot markers and do not render partially parsed arrays as zeros.
- [ ] Mark every value's freshness and unit; compare the panel against a saved
  trace before using it to judge stride or placement experiments. Field
  freshness/units are displayed; saved-trace comparison is still open.

### Remaining batch — qualification and field provenance

1. Restore a targetable WSLg window (or use an external X server), then run
   U2's two-size layout check and all U3 click paths. Record screenshots,
   command refs/replies and authority transitions before calling the controls
   qualified. Do not infer GUI success from the passing UDP tests.
2. Capture one server `joints` telemetry trace from stand → walk → stand and
   compare each displayed foot/contact/speed/torque value and its stale/absent
   state against that trace. Keep the observation panel diagnostic-only until
   this comparison passes.
3. Only if stride decisions require them, define the provenance and sample age
   for measured planar speed and foot reach margin in the server. Avoid
   deriving them from visualiser frame timing or renaming a command setpoint
   as a measurement. Per-joint motor headroom would need corresponding
   per-joint simulator/bridge data; today's peak utilisation is aggregate.

## Decisions and non-goals

- No command-channel schema change is required for U1. Do not broaden robot
  authority or create a visualiser-side safety bypass.
- Do not silently interpret an absent UDP reply as successful application.
- Do not treat a headless socket test as a GUI usability pass.
- Keep the unrun ten-minute gait soak and full repository verification tracked
  in the locomotion campaign; this UI campaign does not close them.

## Experiment log

Record each UI trial with date, tree/binary hashes, WSLg window size, command
and ref, reply/result, observed authority and mode, endpoint/telemetry state,
and decision. Start with the first U3 click-through; no such trial has been
completed at campaign creation.

### 2026-09-24 — first implementation and host screen

- Tree: `24cf1c0` plus the pre-existing dirty campaign work and this batch.
  Visualiser binary SHA-256: `ede77c5ecbaded517930e8f925a2ef3ab5830d66f50d06a83dfb6d47a851f407`.
- `Stand & hold` now sends `motion.set` with supported `STAND`, zero speed and
  yaw; Stop scenario and Cancel navigation stay visible above the sections.
  The visualiser client and server parser tests pass. All 11 visualiser CTests
  and the two focused server CTests pass; `git diff --check` is clean.
- The UI now displays per-ref command history, telemetry/reply age, unknown
  authority on stale telemetry, a bounded panel and the first foot/contact
  observation table. The simulator's peak servo torque utilisation and the
  governor's requested/governed speed are server-owned telemetry fields.
  Named fault parsing no longer mistakes a live fault for `NONE`.
- WSLg inspection did **not** pass: `scripts/check_wslg_gui.sh` reports
  `[WARN:COPY MODE]`; the system-distro `/mnt/shared_memory` virtiofs probe
  fails with `Function not implemented`. The visualiser exits without a
  targetable window on this host. No button click, focus, ground-pick or
  small-window claim is recorded. U3 stays open until the host GUI works or an
  external X server is used.

### 2026-09-24 — missing wireframe follow-up

- Tree: `24cf1c0` plus the preserved dirty worktree. Final visualiser binary
  SHA-256: `449b9022612a2ae702ca5a3211294522a650e94df69b2bbd394e35f816bcdbe8`;
  simulator binary SHA-256: `982551f31b889b550073f0590b8d854bfeef53c9e9d5856711c35fe14b12a07d`.
- Code audit found a two-part blank-robot path: `frame_sink` only sent
  `viz.entity_static` once per body; if the visualiser started late or lost
  those datagrams, it received `viz.entity_frame` poses but could not draw the
  entities. `HasMeasuredSceneGeometry` nevertheless returned true for those
  frame-only entities and hid the telemetry command wireframe.
- The simulator now resends unchanged static descriptors every 30 preview
  frames; the receiver's measured-geometry gate requires both shape and pose.
  The Control Room states which robot view is active and offers a visible
  **Show command wireframe too** action when the command model is suppressed.
- `test_udp_static_refresh` verifies first-frame, intervening pose-only and
  overdue shape-packet behaviour even when the exact refresh frame was
  dropped. The scene-visibility test also verifies that pose-only entities
  cannot suppress the command fallback. These, the existing binary protocol test,
  and all 11 visualiser tests pass. WSLg remains in copy mode here, so this is
  a transport/code fix. The user subsequently confirmed the robot appeared on
  screen; other GUI behaviours in U3 remain unqualified on this host.

### 2026-09-24 — command geometry and ground-grid follow-up

- Command-wireframe FK now uses the server's calibrated servo offsets/signs and
  `pi - mountAngle` leg-frame rotation. The fallback geometry now uses the
  production 43/60/104 mm links and server-body hip locations, rather than an
  older layout. The render path uses the tested kinematics implementation.
- When a drawable terrain patch is visible, the infinite blue plane grid is
  suppressed so it cannot overlap a local surface at a different height. The
  terrain grid's row and column passes both use the world-fixed origin.
  Without terrain data, the blue physics plane remains visible.
- The visualiser build and all 11 CTests pass, including an independent
  server-FK foot-position check. This host still cannot perform WSLg visual
  qualification; the user's on-screen check is needed for CR-9.
