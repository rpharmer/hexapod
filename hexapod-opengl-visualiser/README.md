# hexapod-opengl-visualiser

OpenGL + ImGui visualiser for live hexapod diagnostics.

**Docs:** [`docs/VISUALISER.md`](../docs/VISUALISER.md) (overview) ·
[`docs/VISUALISER_COMMAND_CHANNEL.md`](../docs/VISUALISER_COMMAND_CHANNEL.md)
(command stream) ·
[`docs/VISUALISER_CONTROL_ROOM_CAMPAIGN.md`](../docs/VISUALISER_CONTROL_ROOM_CAMPAIGN.md)
(current control-UI issues and acceptance gates).

It can render either:

- `hexapod-physics-sim` scene preview (**binary** `MPV1` wire format: `viz.scene_clear`, `viz.entity_static`, `viz.entity_frame`, `viz.terrain_patch_meta` + `viz.terrain_floats` chunks; see `hexapod-common/include/minphys_viz_protocol.hpp`)
- `hexapod-server` telemetry packets (**JSON**: `geometry`, `joints`, nav/fusion summaries)
- and can send optional reverse **command** packets to the server (UDP JSON on port 9872; scenarios / nav / motion)

## Dependencies

- CMake 3.16+
- OpenGL 3.3 **core** profile (programmable pipeline; bundled GLAD loader in `third_party/glad/`)
- GLFW 3.3+

### Rendering notes

- Wireframe lines use the programmable pipeline; **line width** may appear as 1px on some drivers (OpenGL core profile does not guarantee wide lines).
- **MSAA** is enabled via `GLFW_SAMPLES=4` when supported.

## Build

Run from `hexapod-opengl-visualiser/`:

```bash
cd hexapod-opengl-visualiser
cmake -S . -B build
cmake --build build -j
```

## Run

Run from `hexapod-opengl-visualiser/`:

```bash
./build/hexapod-opengl-visualiser --udp-port 9870
```

Prefer the repo launcher (builds on demand and raises focus under WSLg):

```bash
# from repo root
./scripts/run_visualiser.sh -- --udp-port 9870
```

Under WSL/WSLg the launchers prefer the **X11** GLFW backend and try to raise
the window via `scripts/raise_wslg_window.ps1`.

If the window title starts with **`[WARN:COPY MODE]`**, this is a WSLg
shared-memory failure (not a hexapod focus bug). Alt+Tab cannot show a usable
surface either. Diagnose with:

```bash
./scripts/check_wslg_gui.sh
```

Typical symptom in the system distro: `ls /mnt/shared_memory` → `Function not
implemented` while `weston.log` has `enable_copy_warning_title = 1`. See that
script’s workaround notes (WSL update, or external X server / `guiApplications=false`).

The renderer listens on UDP port `9870` by default.

For simulator scene packets:

```bash
cd <repo-root>/hexapod-physics-sim
cmake -S . -B build
cmake --build build -j
./build/hexapod-physics-sim --sink udp
```

For live robot telemetry:

```bash
cd <repo-root>/hexapod-server
./build/hexapod-server --telemetry-enable --telemetry-port 9870
```

Optional command channel (scenario, navigation and idle-motion controls from the overlay):

```bash
# server
./build/hexapod-server --telemetry-enable --command-enable \
  --command-port 9872 --command-scenarios-dir scenarios

# visualiser
./build/hexapod-opengl-visualiser --udp-port 9870 --command-port 9872
```

Protocol: [`docs/VISUALISER_COMMAND_CHANNEL.md`](../docs/VISUALISER_COMMAND_CHANNEL.md).

In the visualiser:

- `F1` toggles the overlay panel
- the bounded **Control Room** keeps Stop scenario and Cancel navigation at
  the top, with sections for view, commands and observation. Idle motion has
  explicit Walk (TRIPOD) and Stand & hold actions; Stand & hold is not an
  emergency stop. Results are tracked by request reference as pending,
  applied, rejected or timed out without pausing the render loop
- telemetry and command-reply ages are shown separately; stale authority is
  unknown, and missing foot/contact measurements show `n/a`
- when physics scene entities are present, the measured scene is shown by default and the
  commanded telemetry robot is hidden to avoid servo-lag double images
- the Control Room names the active robot view and offers **Show command
  wireframe too** when the measured scene has hidden the command model; the
  same setting is under View and camera as `Overlay command on measured scene`
- if the visualiser joins an already-running simulator, shape descriptions are
  refreshed every 30 preview frames; pose-only entities no longer hide the
  command-wireframe fallback while the viewer waits for those descriptions
- the command wireframe uses the server's calibrated servo geometry; when a
  terrain patch is available, its local ground grid replaces the overlapping
  blue plane grid (the blue plane remains the fallback without terrain data)
- a visual-only 10 cm green ground reference extends around the moving robot;
  it is not additional collision terrain or navigable-map evidence
- click-to-goal uses logical window coordinates on scaled displays and the
  displayed ground height. A successful command reply means navigation was
  accepted, not that the goal was reached; check the lifecycle shown above
  the controls
