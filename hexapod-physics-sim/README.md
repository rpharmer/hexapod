# hexapod-physics-sim

Headless 3D rigid-body physics engine (**minphys3d**) plus a small **demo driver** (`hexapod-physics-sim`) used to simulate stacks, joints, compound shapes, and a hexapod-style articulated scene. The same engine is linked into many focused unit/regression tests.

This crate is intentionally lightweight: no rendering inside the simulator. Optional **UDP** output streams pose updates to external tools (for example the sibling [`hexapod-opengl-visualiser`](../hexapod-opengl-visualiser) project).

## Contents

| Piece | Role |
|--------|------|
| `include/minphys3d/` | Public headers: world, bodies, joints, collision, solver types |
| `src/core/` | World integration, broadphase, narrowphase dispatch, contact pipeline, constraint solver |
| `src/narrowphase/` | GJK / EPA helpers |
| `src/solver/` | Block solvers and island ordering |
| `src/demo/` | CLI demo, JSON scene loader, frame sinks, interactive terminal |
| `tests/` | `CTest`-registered executables (math, GJK, cylinders, servos, scenes, …) |
| `assets/scenes/examples/` | Example **minphys** JSON scenes (`schema_version` 1 or 2) |

## Requirements

- **CMake** 3.16 or newer  
- **C++17** compiler (GCC, Clang, or MSVC)  
- **POSIX**-style OS for the **UDP** sink (Linux, macOS, WSL). On native **Windows**, `--sink udp` falls back to a dummy sink (see `src/demo/frame_sink.cpp`).

## Configure and build

From this directory:

```bash
cmake -S . -B build
cmake --build build -j
```

### CMake options

| Option | Default | Meaning |
|--------|---------|---------|
| `MINPHYS3D_BUILD_DEMO` | `ON` | Build `hexapod-physics-sim` |
| `MINPHYS3D_BUILD_TESTS` | `ON` | Build tests and register them with CTest |

Examples:

```bash
cmake -S . -B build -DMINPHYS3D_BUILD_DEMO=OFF
cmake -S . -B build -DMINPHYS3D_BUILD_TESTS=OFF
```

The static library **`minphys3d_core`** holds all shared engine sources; the demo and each test executable link it once.

## Running the demo

Executable: `build/hexapod-physics-sim` (run from the build tree or pass paths accordingly).

### Command-line interface

| Flag | Description |
|------|-------------|
| `--sink dummy\|udp` | Output: no network (`dummy`) or UDP packets to `--udp-host` / `--udp-port` (`udp`) |
| `--model default\|hexapod` | Built-in scene when **not** using `--scene-file` |
| `--scene-file PATH` | Load a **minphys** JSON scene (supported `schema_version`: **1** or **2**) |
| `--serve` | **UDP physics server** (`hexapod-server` IPC): binary `ConfigCommand` / `StepCommand` → `ConfigAck` / `StateResponse` (listen **9871**, `--serve-port`). Add **`--sink udp`** to also emit minphys **scene JSON** each stepped frame to `--udp-host`:**`--udp-port`** for [`hexapod-opengl-visualiser`](../hexapod-opengl-visualiser) (same wire as the demo). |
| `--udp-host HOST` | UDP destination (default `127.0.0.1`) |
| `--udp-port PORT` | UDP destination port (default `9870`) |
| `--frames N` | Number of simulation frames (default `1200`) |
| `--realtime` | Pace the loop near ~60 Hz (useful when driving a live viewer) |
| `--zero-gravity` | Use zero gravity instead of `(0, -9.81, 0)` |
| `--interactive`, `-i` | Terminal REPL (configure sink/host/scene, then run) |
| `-h`, `--help` | Print usage |

### Interactive mode (`-i`)

Inside the REPL, `presets` (or `catalog` / `scenarios`) prints a **large catalog** of curated `preset <name>` entries for UDP visual debugging: built-in classic/hexapod, the original `assets/scenes/examples/*.json` demos, and **38** additional scenes under `assets/scenes/visual/` (`vis_*.json`: collisions, joints, cylinders, half-cylinders, sleep-friendly stacks, and more). Most rows expose several **pipe-separated aliases** (for example `vis-hc-cyl|col-hc-cyl`). Each preset turns on **UDP** to `127.0.0.1:9870`, **Earth gravity** (unless the preset is zero-G), and **realtime pacing** by default (except `classic-soak`, which is CPU-fast for long regression-style runs).

Built-in demos:

```bash
./build/hexapod-physics-sim --sink dummy --model default
./build/hexapod-physics-sim --sink dummy --model hexapod --frames 600
```

JSON-driven run (paths relative to your cwd):

```bash
./build/hexapod-physics-sim --sink dummy --scene-file ../hexapod-physics-sim/assets/scenes/examples/stack_minimal.json
```

### UDP and OpenGL viewer

On Linux/WSL/macOS, UDP mode emits JSON lines with `message_type` `entity_static` (when shape/material changes), `entity_frame` (pose per frame), and `terrain_patch` (the current height-map / local terrain debug overlay). See [`hexapod-opengl-visualiser`](../hexapod-opengl-visualiser/README.md) for a matching receiver.

Typical two-terminal workflow:

```bash
# Terminal A — viewer listens
cd ../hexapod-opengl-visualiser && ./build/hexapod-opengl-visualiser --udp-port 9870

# Terminal B — sim sends to localhost
cd ../hexapod-physics-sim/build && ./hexapod-physics-sim --sink udp --realtime --model hexapod
```

### Run log

Each demo run writes a text log when the `logs/` directory can be created next to the working directory (see `BuildDebugLogPath()` in `src/demo/scenes.cpp` / `scene_json.cpp`):

- Preferred: `logs/latest.log` (or `hexapod-physics-sim/logs/latest.log` when cwd is the repo root)
- Fallback: `hexapod-physics-sim.log` in the current directory if directory creation fails

## Scene JSON (minphys)

- **`schema_version`**: must be **1** or **2** (values above **2** are rejected by the loader).  
- Example files live under **`assets/scenes/examples/`** (`stack_minimal.json`, `compound_preview.json`, `joints_distance.json`, …). Extra **visual-debug** scenes for the interactive `preset` catalog live under **`assets/scenes/visual/`** (`vis_*.json`).  
- The loader is implemented in `src/demo/scene_json.cpp`; it accepts the fields used there (bodies, shapes, materials, joints, solver iteration count, etc.).  
- Example JSON may contain `//` comments for human readers; strict JSON-only tools should strip those before parsing outside this project.

Supported **primitive** shape types include sphere, box, plane, capsule, cylinder, and half-cylinder; **compound** aggregates child primitives with local transforms (see `IsCompoundChildShapeSupported` in the headers for allowed child shapes).

## Tests

With `MINPHYS3D_BUILD_TESTS=ON`, CMake registers one CTest entry per `add_minphys3d_test` target (collision, GJK, broadphase, block solvers, servo stability, regression scenes, etc.), plus **`test_servo_visual_presets_json`**, which loads the **servo** `assets/scenes/visual/vis_servo_*.json` presets (including a zero-gravity pass on `vis_servo_chain.json`) and checks bounded motion and servo angle error over multi-second runs. Set **`MINPHYS_SERVO_JSON_TEST_VERBOSE=1`** when running that test to print per-interval peaks to stderr.

```bash
cd build
ctest --output-on-failure
# or a single test:
ctest -R test_cylinder_collision --output-on-failure
ctest -R test_servo_visual_presets_json --output-on-failure
```

You can also run an executable directly, for example `./build/test_unit_math`.

For **per-frame** contact and servo diagnostics while running a JSON scene from the demo binary, set environment variable **`MINPHYS_JSON_SCENE_DIAG=1`** (writes the same line format as `logs/latest.log` to **stderr** every frame; best with a short `--frames` count).

## Engine notes (short)

- **Gravity** defaults to `(0, -9.81, 0)`; example scenes often use a ground **plane** with normal `(0, 1, 0)`.  
- **World** owns bodies, joints, contacts, manifolds, broadphase proxies, and the constraint solver configuration.  
- **Narrowphase** mixes analytic / specialized manifolds for common pairs (boxes, cylinders, capsules, half-cylinders, …) with convex fallbacks where appropriate.  
- **Determinism**: the contact pipeline can sort manifolds and contacts for stable ordering when enabled in solver config (used heavily by regression tests).

For API details, start from `include/minphys3d/core/world.hpp` and `include/minphys3d/core/body.hpp`.

## Repository layout reminder

This folder is part of the larger **hexapod** workspace; paths like `../hexapod-opengl-visualiser` assume the usual sibling checkout layout.
