# Visualiser command channel

Design and implementation notes for the reverse UDP JSON control path from the
OpenGL visualiser (or any client) into `hexapod-server` interactive mode.

**Status:** scenario, navigation and idle-motion commands are implemented.
The visualiser client now submits without blocking the render loop and shows
matched apply-result replies. A live server-socket round trip is verified;
manual visualiser click-through is still pending. The channel is default
**off** on the server binary and enabled by default in the stack launchers.
For navigation, an accepted reply means the server started the task, **not**
that the robot reached the goal; see the
[navigation interaction campaign](NAVIGATION_INTERACTION_CAMPAIGN.md).

Related:

- Visualiser overview (growing): [`VISUALISER.md`](VISUALISER.md)
- Control Room usability campaign: [`VISUALISER_CONTROL_ROOM_CAMPAIGN.md`](VISUALISER_CONTROL_ROOM_CAMPAIGN.md)
- Frames / topology: [`REFERENCE_FRAMES.md`](REFERENCE_FRAMES.md)
- Component READMEs: [`hexapod-server/README.md`](../hexapod-server/README.md),
  [`hexapod-opengl-visualiser/README.md`](../hexapod-opengl-visualiser/README.md)

## Goals

1. Let the visualiser **select / run / stop scenarios** without restarting the
   server as `--scenario` batch mode.
2. Keep a thin protocol that can also drive **nav goals** and **motion probes**.
3. Avoid fighting the gamepad: explicit **scenario > nav > gamepad** authority.
4. Stay aligned with existing telemetry style (UDP JSON, localhost-oriented).

## Non-goals (v1)

- HTTP/REST or TCP sessions
- Fault / safety / map-override injection from the visualiser (use scenario TOML)
- Driving the plant via physics serve **9871**
- Extending scenario TOML with raw waypoint lists (use `nav.waypoints` JSON)
- VirtualGamepad-over-UDP (can reuse `motion.set` later)

## Topology

| Port | Direction | Format | Role |
|------|-----------|--------|------|
| **9870** | server → viz | JSON telemetry | Observe |
| **9870** | sim → viz | Binary MPV1 | Observe scene |
| **9871** | server ↔ sim | `physics_sim_protocol` | Plant step/state |
| **9872** | viz → server | JSON commands + `command_result` replies | Control (opt-in) |

Do **not** overload 9870 or 9871 with commands.

```text
visualiser --UDP JSON cmd--> :9872  hexapod-server (InteractiveRunner)
hexapod-server --UDP JSON telem--> :9870  visualiser
physics-sim --UDP MPV1--> :9870  visualiser
server <-> sim serve :9871
```

## Authority model

Owned by the **server**, not the visualiser.

| Level | When | Who owns motion | Accepted from command channel |
|-------|------|-----------------|-------------------------------|
| **scenario** | `ScenarioSession` active | Scenario timeline (+ nav events it starts) | `scenario.list`, `scenario.stop`; reject `nav.*` / `motion.set` |
| **nav** | Nav manager active, no scenario | `NavigationManager::mergeIntent` over gamepad base | `scenario.run`, `nav.cancel`, `nav.goto` / `nav.waypoints`; reject `motion.set` |
| **idle** | Neither | Gamepad or `motion.set` | All command types |

Gamepad `setMotionIntent` is skipped while a scenario session is active
([`mode_runners.cpp`](../hexapod-server/src/control/mode_runners.cpp)).

CLI `--scenario` still runs exclusive `ScenarioRunner` (batch). The command
channel hosts scenarios **inside interactive mode** via `ScenarioSession`.

## Protocol (schema_version 1)

### Inbound

Common fields: `"schema_version": 1`, `"type": "…"`, optional `"ref": "…"`.

| type | Body | Effect |
|------|------|--------|
| `scenario.list` | — | Reply with ids under `--command-scenarios-dir` |
| `scenario.run` | `"id"` or `"path"` | Load TOML, start `ScenarioSession` |
| `scenario.stop` | — | Stop session, cancel nav, idle intent |
| `nav.goto` | `goal_x_m`, `goal_y_m`, optional `goal_yaw_rad`, `gait`, `body_height_m` | `startNavigateToPose`; omitted yaw preserves the current estimated heading, explicit yaw is a final pose requirement |
| `nav.waypoints` | `poses: [{x_m,y_m,yaw_rad?}, …]` | `startRawFollowWaypoints`; omitted yaw inherits the current or last explicitly supplied heading |
| `nav.cancel` | — | `NavigationManager::cancel` |
| `motion.set` | subset of scenario motion fields (`mode`, `gait`, `body_height_m`, `speed_mps`, …) | `setMotionIntent` when idle |

Examples:

```json
{"schema_version":1,"type":"scenario.list","ref":"viz-1"}
{"schema_version":1,"type":"scenario.run","ref":"viz-2","id":"06_map_aware_navigation"}
{"schema_version":1,"type":"scenario.stop","ref":"viz-3"}
{"schema_version":1,"type":"nav.goto","ref":"viz-4","goal_x_m":1.0,"goal_y_m":0.0,"goal_yaw_rad":0.0}
{"schema_version":1,"type":"motion.set","ref":"viz-5","mode":"WALK","speed_mps":0.05,"heading_rad":0.0}
```

### Outbound reply

Best-effort UDP datagram back to the sender:

```json
{"type":"command_result","schema_version":1,"ok":true,"ref":"viz-1","reason":"listed","scenarios":["01_…","02_…"]}
```

`scenario.list` is answered on the listener thread. Other types are queued
for the interactive loop; the server replies with the **actual apply result**
after it accepts or rejects the command. The visualiser sends immediately,
polls the nonblocking socket each frame, matches `command_result.ref` to its
outstanding request, ignores stale/unmatched replies, and reports a local
timeout after 2 seconds. “Sent” is not displayed as “applied.”

### Telemetry

`joints` packets may include:

```json
"command":{"authority":"idle|nav|scenario","nav_active":false,"scenario":"optional-name"}
```

Filled from `RobotControl::setCommandAuthorityTelemetry` each interactive tick.

## Code map

| Piece | Location |
|-------|----------|
| Protocol parse / serialize / list / resolve | [`command_channel.hpp`](../hexapod-server/include/control/command_channel.hpp) / [`.cpp`](../hexapod-server/src/control/command_channel.cpp) |
| `CommandIngress`, `UdpCommandListener` | same |
| Tickable scenario host | [`scenario_session.hpp`](../hexapod-server/include/scenario/scenario_session.hpp) / [`.cpp`](../hexapod-server/src/scenario/scenario_session.cpp) |
| Batch `--scenario` uses session | [`scenario_driver.cpp`](../hexapod-server/src/scenario/scenario_driver.cpp) |
| Interactive wire + gamepad gate | [`mode_runners.cpp`](../hexapod-server/src/control/mode_runners.cpp) |
| CLI flags | [`cli_options.cpp`](../hexapod-server/src/control/cli_options.cpp) |
| Telemetry `command` object | [`telemetry_json.cpp`](../hexapod-server/src/control/telemetry_json.cpp) |
| Visualiser UDP client | [`udp_command_client.hpp`](../hexapod-opengl-visualiser/include/visualiser/net/udp_command_client.hpp) |
| Overlay List/Run/Stop | [`legacy_main.cpp`](../hexapod-opengl-visualiser/src/app/legacy_main.cpp) Control Room section |
| Unit tests | [`test_command_channel.cpp`](../hexapod-server/tests/test_command_channel.cpp) |

## Enable / run

### Binary (default off)

```bash
# from hexapod-server/
./build/hexapod-server --telemetry-enable --command-enable \
  --command-host 127.0.0.1 --command-port 9872 \
  --command-scenarios-dir scenarios
```

Flags: `--command-enable` / `--command-disable`, `--command-host` (bind),
`--command-port` (default 9872), `--command-scenarios-dir`.

Rebuild the **default** preset (`hexapod-server/build/`) after pulling these
changes; `--skip-build` on stack scripts will keep a stale binary that rejects
`--command-enable`.

### Stack launchers (default on for interactive)

```bash
# from repo root
scripts/run_physics_stack.sh --controller-optional
scripts/run_sim_stack.sh

scripts/run_physics_stack.sh --command-disable --controller-optional
```

Also: `--command-port`, `--command-scenarios-dir`. With `--scenario` (batch),
the command listener is not started (interactive-only).

`scripts/run_server_with_telemetry.sh` stays opt-in (`--command-enable`).

### Visualiser client

```bash
# from hexapod-opengl-visualiser/
./build/hexapod-opengl-visualiser --udp-port 9870 \
  --command-host 127.0.0.1 --command-port 9872
```

Overlay: **Control Room** — always-visible Stop scenario / Cancel navigation,
then bounded View, Scenarios, Navigation, Motion and Diagnostics sections.
Idle motion offers **Walk (TRIPOD) with settings** and **Stand & hold**;
the latter sends supported `STAND` with zero planar speed and yaw rate. It is
an ordinary motion command, **not** an emergency stop or torque-off command.
The panel shows pending versus matched applied/rejected/timed-out results by
reference, recent reply and telemetry age, and treats stale authority as
unknown. Rejected waypoints retain their draft for correction.

## Validation checklist

- [x] Parse/serialize + CLI flags unit-tested (`ctest -R command_channel`)
- [x] Stack scripts pass `--command-enable` (physics + sim) unless disabled
- [x] Nonblocking visualiser client tests cover matched/stale `ref`, apply rejection and timeout (`test_command_client`)
- [x] Local sim server socket returns `listed`, `stopped` and `motion.set accepted` apply replies with matching refs
- [ ] Interactive + command: list/run/stop a stock scenario from the overlay
- [ ] Gamepad ignored while scenario runs; idle restores gamepad
- [ ] `nav.goto` / `nav.cancel` / `motion.set` authority matrix on a live stack
- [ ] Default binary without `--command-enable`: no listen socket

## Roadmap (not committed)

| Priority | Item |
|----------|------|
| Implemented; GUI check pending | Click-to-goal in the scene → `nav.goto` |
| Implemented; GUI check pending | Correlated apply-result replies for all command types |
| Implemented; GUI check pending | Motion sliders → `motion.set` when idle |
| Implemented; GUI check pending | Waypoint edit UI → `nav.waypoints` |
| Later | Optional VirtualGamepad device feeding the interactive mapper |
| Later | Hot-reload / scenario lint from the overlay |

When adding command types: keep schema_version bumps explicit, reject unknown
`type` values, and document authority rules in the table above.
