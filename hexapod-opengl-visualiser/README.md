# hexapod-opengl-visualiser

OpenGL + ImGui visualiser for live hexapod diagnostics.

It can render either:

- `hexapod-physics-sim` scene packets (`entity_static`, `entity_frame`, `terrain_patch`)
- `hexapod-server` telemetry packets (`geometry`, `joints`, nav/fusion summaries)

## Dependencies

- CMake 3.16+
- OpenGL development headers/libs
- GLFW 3.3+

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
