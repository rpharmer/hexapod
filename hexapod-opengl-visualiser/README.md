# hexapod-opengl-visualiser

Minimal OpenGL renderer that listens for `hexapod-physics-sim` UDP scene packets and renders the live bodies.

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

The renderer listens on UDP port `9870` by default and consumes the simulator's
`entity_static` and `entity_frame` packets.

To wire it to the simulator from a second terminal:

```bash
cd ../hexapod-physics-sim
cmake -S . -B build
cmake --build build -j
./build/hexapod-physics-sim --sink udp
```

You should then see the default demo scene animate in the OpenGL window.
