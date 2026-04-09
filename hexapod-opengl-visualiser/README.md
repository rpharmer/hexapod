# hexapod-opengl-visualiser

Minimal OpenGL renderer scaffold for local rendering experiments.

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
./build/hexapod-opengl-visualiser
```

The renderer opens a window and draws a rotating color triangle over a pulsing background.
