# hexapod-opengl-visualiser

OpenGL + ImGui visualiser for live hexapod diagnostics.

It can render either:

- `hexapod-physics-sim` scene preview (**binary** `MPV1` wire format: `viz.scene_clear`, `viz.entity_static`, `viz.entity_frame`, `viz.terrain_patch_meta` + `viz.terrain_floats` chunks; see `hexapod-common/include/minphys_viz_protocol.hpp`)
- `hexapod-server` telemetry packets (**JSON**: `geometry`, `joints`, nav/fusion summaries)

## Dependencies

- CMake 3.16+
- OpenGL 3.3 **core** profile (programmable pipeline; bundled GLAD loader in `third_party/glad/`)
- GLFW 3.3+

### Rendering notes

- Wireframe lines use the programmable pipeline; **line width** may appear as 1px on some drivers (OpenGL core profile does not guarantee wide lines).
- **MSAA** is enabled via `GLFW_SAMPLES=4` when supported.

## Build

```bash
cd hexapod-opengl-visualiser
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/hexapod-opengl-visualiser --udp-port 9870
```

The renderer listens on UDP port `9870` by default.

For simulator scene packets:

```bash
cd ../hexapod-physics-sim
cmake -S . -B build
cmake --build build -j
./build/hexapod-physics-sim --sink udp
```

For live robot telemetry:

```bash
cd ../hexapod-server
./build-tests/hexapod-server --telemetry-enable --telemetry-port 9870
```

In the visualiser:

- `F1` toggles the overlay panel
- the overlay exposes camera sliders, scene toggles, and telemetry summaries
