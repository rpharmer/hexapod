# OpenGL visualiser

Living overview of `hexapod-opengl-visualiser`: what it is for, how it talks to
the rest of the stack, and where to extend it. Prefer this doc for architecture
and growth notes; keep the component [`README`](../hexapod-opengl-visualiser/README.md)
for build/run and WSL day-to-day commands.

## Role

Native OpenGL + ImGui **Control Room** for live hexapod diagnostics:

1. **Observe** physics scene packets (measured plant)
2. **Observe** server telemetry (commanded / estimated robot, nav, fusion)
3. **Control** (opt-in) via the command channel — scenarios first; nav/motion next

It is not a second controller runtime. Authority and locomotion stay in
`hexapod-server`; the visualiser is a client.

## Doc index

| Doc | Contents |
|-----|----------|
| This file | Overview, ports, UI map, extension points |
| [`VISUALISER_COMMAND_CHANNEL.md`](VISUALISER_COMMAND_CHANNEL.md) | Command UDP protocol, authority, code map, roadmap |
| [`VISUALISER_CONTROL_ROOM_CAMPAIGN.md`](VISUALISER_CONTROL_ROOM_CAMPAIGN.md) | Control usability defects, priorities, and WSLg acceptance gates |
| [`REFERENCE_FRAMES.md`](REFERENCE_FRAMES.md) | Server ↔ scene frame handoff |
| [`ALGORITHMS_SERVER_CONFIG_TELEMETRY.md`](ALGORITHMS_SERVER_CONFIG_TELEMETRY.md) | Telemetry field mapping |
| [`VISUALISER_TELEMETRY.md`](VISUALISER_TELEMETRY.md) | **Legacy** browser visualiser only — not current |
| [`WSL_XBOX_CONTROLLER.md`](WSL_XBOX_CONTROLLER.md) | Controller + related WSL notes |
| [`hexapod-opengl-visualiser/README.md`](../hexapod-opengl-visualiser/README.md) | Build, run, WSLg copy-mode |

## Ports and data paths

| Port | Payload | Direction |
|------|---------|-----------|
| **9870** | JSON telemetry (`geometry`, `joints`, …) | server → visualiser |
| **9870** | Binary MPV1 scene | physics-sim → visualiser (same socket) |
| **9872** | JSON commands + `command_result` | visualiser → server (opt-in) |
| **9871** | Physics serve protocol | server ↔ sim — **not** visualiser |

Launchers:

- `scripts/run_physics_stack.sh` — viz + sim serve + server (command channel **on** by default)
- `scripts/run_sim_stack.sh` — viz + server telemetry (command channel **on** by default)
- `scripts/run_visualiser.sh` — viz only
- `scripts/run_server_with_telemetry.sh` — server only (command **opt-in**)

## UI map (current)

Control Room overlay (`F1` toggles):

- **View and camera** — measured scene / command robot / terrain / camera
- **Robot view status** — measured physics, command fallback or both; a visible
  action reveals the command wireframe when measured geometry hides it
- **Telemetry** — mode, fault, bus, estimator; **authority** when present
- **Quick actions** — Stop scenario and Cancel navigation stay at the top,
  independently of whether scenario listing succeeded
- **Commands** — List / Run scenarios, click-to-goal, draft waypoints,
  Walk (TRIPOD) and Stand & hold; per-ref pending/applied/rejected/timeout
  history and explicit stale-authority warning
- **Locomotion observation** — fresh planned/raw/fused support, commanded and
  measured foot height, tracking error, target-clamp distortion and
  requested/governed speed; aggregate physics-sim servo torque use when
  available; unavailable values read `n/a`
- **Navigation / Fusion** — summaries from `joints` packets when present
- **Robot geometry / leg angles** — from telemetry

Primary implementation still lives in
[`legacy_main.cpp`](../hexapod-opengl-visualiser/src/app/legacy_main.cpp), with
modular helpers under `include/visualiser/` and `src/` (net, parsing, render, ui).
The panel defaults to a bounded, scrollable size; its real WSLg click-through
remains open while the host is in shared-memory copy mode.

## Code layout (high value)

| Area | Paths |
|------|-------|
| App / overlay (live UI) | `src/app/legacy_main.cpp` |
| CLI options | `include/visualiser/app/options.hpp`, `src/app/options.cpp` |
| Telemetry + MPV1 receive | `src/net/udp_receiver.cpp`, `src/parsing/*` |
| Command send | `include/visualiser/net/udp_command_client.hpp`, `src/net/udp_command_client.cpp` |
| Frames / FK draw | `include/visualiser/math/frame.hpp`, `src/robot/*` |
| Modular overlay stub | `src/ui/overlay.cpp` (thinner than legacy; prefer extending legacy until unified) |

## Extension guide

When fleshing out the visualiser, prefer this order:

1. **Protocol first** — add or extend types in
   [`VISUALISER_COMMAND_CHANNEL.md`](VISUALISER_COMMAND_CHANNEL.md) and server
   `command_channel` / telemetry JSON; keep authority rules explicit.
2. **Thin client** — nonblocking `CommandClient` helpers for scenario, nav and
   motion commands; poll matched replies on the render loop.
3. **UI** — Control Room section that calls the client; show authority from
   telemetry so the user sees who is in charge.
4. **Docs** — update this overview’s UI map + the command-channel roadmap table.

Suggested UI growth (see command-channel roadmap for protocol detail):

| Feature | Likely command / data |
|---------|------------------------|
| Click ground → goal | `nav.goto` |
| Waypoint polyline edit | `nav.waypoints` |
| Idle motion sliders | `motion.set` |
| Scenario browser filters | `scenario.list` + local UI |
| Show planned feet / support | telemetry fields already published |

Avoid:

- Sending plant step traffic on 9871 from the visualiser
- A parallel control stack that bypasses `RobotControl` / `NavigationManager`
- Treating the visualiser as the source of truth for authority

## WSL / GUI

Focus and “invisible window” issues are usually **WSLg copy mode**, not the
command channel. Diagnose with `scripts/check_wslg_gui.sh` and the visualiser
README. Stack scripts already raise the window and prefer the X11 GLFW backend.

## Changelog (docs)

| Date | Note |
|------|------|
| 2026-09-22 | Command channel v1 (scenario-first); stack launchers enable 9872 by default |
| 2026-09-22 | This overview doc added as the growable visualiser hub |
