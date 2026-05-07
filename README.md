# Hexapod

Monorepo for a serial-controlled hexapod robot, containing Linux host control software, Servo 2040 firmware, physics simulation, visualisers, and shared protocol definitions.

## Start here

- `docs/ALGORITHMS_OVERVIEW.md` — architecture map for simulation and control algorithms.
- `docs/REFERENCE_FRAMES.md` — frame conventions and transforms across sim/server/visualisers.
- `docs/SERVER_CONFIG_REFERENCE.md` and `docs/PHYSICS_SIM_CONFIG_REFERENCE.md` — complete configuration references.

## Components

- `**hexapod-server/**` — Linux host application that loads calibration/configuration, runs real-time control loops, and communicates with firmware over framed serial packets.
- `**hexapod-client/**` — Pimoroni Servo 2040 (RP2040) firmware that drives 18 servos, handles handshake/lifecycle state, and serves sensing + power commands.
- `**hexapod-common/**` — shared protocol constants, IDs, and framing helpers used by both host and firmware.
- `**hexapod-physics-sim/**` — headless rigid-body simulation engine/demo used for offline control testing and serve-mode state exchange.
- `**hexapod-opengl-visualiser/**` — native OpenGL visualiser for sim scene packets and server telemetry overlays.
- `**hexapod-visualiser/**` — additional visualiser tooling and experiments.
- `**docs/**` — architecture, algorithms, frame conventions, and configuration references in addition to hardware/firmware guides.

## Repository layout

```text
hexapod/
├── README.md
├── docs/                        # architecture + algorithms + config references
├── hexapod-common/              # shared protocol and framing
├── hexapod-server/              # host runtime and control loops
├── hexapod-client/              # firmware (RP2040 Servo 2040)
├── hexapod-physics-sim/         # rigid-body simulation + serve mode
├── hexapod-opengl-visualiser/   # native visualiser
├── hexapod-visualiser/          # additional visualiser tooling
└── scripts/                     # verification and workflow helpers
```

## End-to-end communication flow

### Hardware path (serial)

1. Server parses `hexapod-server/config.txt`.
2. Server opens serial transport (commonly `/dev/ttyACM0`).
3. Server performs protocol handshake and uploads calibrations.
4. Runtime loop exchanges heartbeat, motion, power, and sensing commands with firmware.

### Simulation path (`sim` / `physics-sim`)

1. Server runs in simulator mode using `config.sim.txt` or `config.physics-sim.txt`.
2. `hexapod-server` executes control loops against simulated hardware/physics bridge.
3. Telemetry and/or sim scene packets stream to visualisers for inspection.

Protocol source of truth:

- `hexapod-common/include/hexapod-common.hpp`
- `hexapod-common/include/framing.hpp`
- `docs/FIRMWARE.md`

## Quick start

### Host (server)

```bash
cd hexapod-server
cmake -S . -B build
cmake --build build -j
./build/hexapod-server
```

For simulator-first runs, use `hexapod-server/config.sim.txt` (or physics bridge config) and the scripts in `scripts/` below.

### Firmware (client)

```bash
cd hexapod-client
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=ON
cmake --build build --target setup-sdks
cmake -S . -B build -DHEXAPOD_CLIENT_SETUP_SDKS_ONLY=OFF
cmake --build build --target hexapod-client
```

Flash by copying `hexapod-client/build/hexapod-client.uf2` to the board in BOOTSEL mode.

### Firmware host tests (native)

```bash
cd hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

## Unified verification entrypoint

Run the default repository quality gates from the repository root:

```bash
cd <repo-root>
./scripts/verify.sh
```

This single command runs, in order (fail-fast):

1. `hexapod-server` CMake test preset (`tests`)
2. `hexapod-client` firmware host-test preset (`host-tests`)
3. Lightweight server scenario smoke (`scenarios/01_nominal_stand_walk.toml`) when available

Each section prints clear markers so CI and local runs can quickly identify where failures occur.

## OpenGL visualiser flow

To start the OpenGL visualiser without manually rebuilding each time:

```bash
cd <repo-root>
scripts/run_visualiser.sh -- --udp-port 9870
```

For simulator-mode scenario sweeps without copying configs by hand:

```bash
cd <repo-root>
scripts/run_server_scenarios.sh
```

To launch the server and visualiser together in simulator mode:

```bash
cd <repo-root>
scripts/run_sim_stack.sh
```

Optional scenario-run example:

```bash
cd <repo-root>
scripts/run_sim_stack.sh --scenario scenarios/01_nominal_stand_walk.toml
```

For a **serial-connected robot** (or when `hexapod-server` runs on a different machine), run server telemetry with an explicit OpenGL visualiser IP:

```bash
# on the server machine
cd <repo-root>
scripts/run_server_with_telemetry.sh --mode serial --telemetry-host <VISUALISER_IP> --telemetry-port 9870
```

```bash
# on the OpenGL visualiser machine
cd <repo-root>
scripts/run_visualiser.sh -- --udp-port 9870
```

The OpenGL visualiser is the primary native GUI for the project. It can render both
`hexapod-physics-sim` scene packets and live `hexapod-server` telemetry, including
joint angles, navigation state, fusion diagnostics, and terrain overlays.

## Development workflow

Use this loop when iterating on protocol/control changes that affect both host and firmware:

1. Update protocol IDs/payloads in `hexapod-common/include/hexapod-common.hpp`.
2. Update firmware handlers in `hexapod-client/*commands*.cpp` / router code.
3. Update host-side transport/control consumers in `hexapod-server/src/*`.
4. Run server tests in simulator mode before hardware testing.

Suggested verification commands:

```bash
cd hexapod-server
cmake --preset tests
cmake --build --preset tests -j
ctest --preset tests --output-on-failure
```

```bash
cd hexapod-client
cmake --preset host-tests
cmake --build --preset host-tests -j
ctest --preset host-tests --output-on-failure
```

Then run a scenario sweep in simulator mode:

```bash
cd hexapod-server
cp config.sim.txt config.txt
for s in scenarios/*.toml; do
  ./build/hexapod-server --scenario "$s" || break
done
```

## Documentation map

- `hexapod-server/README.md` — host runtime architecture, simulation flow, and test commands.
- `hexapod-client/README.md` — firmware build/flash workflow and protocol command handling notes.
- `docs/ALGORITHMS_OVERVIEW.md` — index for simulation and server locomotion algorithm documentation.
- `docs/ALGORITHMS_PHYSICS_SIM.md` — physics-sim step, collision, solver, TOI, and sleep algorithms.
- `docs/ALGORITHMS_SERVER_LOCOMOTION.md` — server locomotion roles, module interactions, and control flow.
- `docs/ALGORITHMS_SERVER_CONFIG_TELEMETRY.md` — config-to-runtime and telemetry mapping for control diagnostics.
- `docs/ALGORITHMS_GLOSSARY.md` — terminology glossary for supervisors, governors, modules, and solver concepts.
- `docs/ALGORITHMS_TRACEABILITY_CHECKLIST.md` — source-to-doc coverage checklist.
- `docs/REFERENCE_FRAMES.md` — canonical frame conventions and transforms across sim, server, visualiser, and sensors.
- `docs/SERVER_CONFIG_REFERENCE.md` — exhaustive `hexapod-server` TOML key reference, validation ranges, and consumers.
- `docs/PHYSICS_SIM_CONFIG_REFERENCE.md` — exhaustive `hexapod-physics-sim` CLI/JSON/terrain/serve configuration reference.
- `docs/CONFIG_DOCS_COVERAGE.md` — parser-to-doc coverage checklist for server and simulator config surfaces.
- `docs/FIRMWARE.md` — wire protocol framing, constants, and payload definitions.
- `docs/HARDWARE.md` — mechanical/electrical build reference and dimensions.
- `docs/EXTENDING_IO_AND_HARDWARE.md` — how to add new control input devices and hardware bridge backends.

## Safety notes

- Validate relay defaults, servo enable sequencing, and emergency stop behavior before full-body motion testing.
- Start with the robot unloaded and low-amplitude commands after calibration changes.
- Prefer simulator mode (`hexapod-server/config.sim.txt`) for early control-policy validation.
- Keep one hand on power disconnect / E-stop whenever first exercising new gait or calibration logic on hardware.

