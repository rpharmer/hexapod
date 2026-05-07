# Config Documentation Coverage Checklist

This checklist maps parser/loader sources to documentation coverage.

## Server coverage (`hexapod-server`)

### Schema and parser entrypoint

- [x] `hexapod-server/src/config/toml_parser.cpp` covered in `docs/SERVER_CONFIG_REFERENCE.md`
  - schema header keys (`title`, `Schema`, `SchemaVersion`)
  - section parser invocation order

### Runtime section

- [x] `hexapod-server/src/config/runtime_section_parser.cpp` covered
  - `Runtime.Mode`
  - `Runtime.Sim.*`
  - `Runtime.PhysicsSim.*`
  - `Runtime.Log.*`
  - `Runtime.ReplayLog.*`
  - `Runtime.Telemetry.*` primary keys
  - telemetry compatibility aliases
  - `Runtime.Investigation.*`

### Transport/calibration/geometry/tuning sections

- [x] `hexapod-server/src/config/transport_section_parser.cpp` covered (`SerialDevice`, `BaudRate`, `Timeout_ms`)
- [x] `hexapod-server/src/config/calibrations_section_parser.cpp` covered (`MotorCalibrations`)
- [x] `hexapod-server/src/config/geometry_section_parser.cpp` covered (`Geometry.*` scalar/list/dynamics keys)
- [x] `hexapod-server/src/config/tuning_section_parser.cpp` covered (all `Tuning.*` families)

### Validation and runtime mapping

- [x] `hexapod-server/src/config/config_validation.cpp` behavior summarized
- [x] `hexapod-server/src/control/control_config.cpp` consumer mapping summarized
- [x] `hexapod-server/src/app/hexapod-server.cpp` CLI override precedence documented

## Physics sim coverage (`hexapod-physics-sim`)

### CLI surface

- [x] `hexapod-physics-sim/src/main.cpp` covered in `docs/PHYSICS_SIM_CONFIG_REFERENCE.md`
  - all flags including interactive/serve/resource-monitoring variants

### JSON and terrain patch surface

- [x] `hexapod-physics-sim/src/demo/scene_json.cpp` covered
  - top-level keys
  - `bodies[]`
  - `joints[]`
  - `compound.children[]`
  - `terrain_patch` keys
- [x] `hexapod-physics-sim/src/demo/terrain_patch.hpp` covered (`TerrainPatchConfig` fields)

### Serve/protocol/environment

- [x] `hexapod-physics-sim/src/demo/serve_mode.cpp` serve controls and behavior covered
- [x] `hexapod-common/include/physics_sim_protocol.hpp` serve protocol configurable structs covered
- [x] Environment variables covered:
  - `MINPHYS_JSON_SCENE_DIAG`
  - `MINPHYS_SERVO_JSON_TEST_VERBOSE`

## Discoverability

- [x] Documentation map links added in `README.md`:
  - `docs/SERVER_CONFIG_REFERENCE.md`
  - `docs/PHYSICS_SIM_CONFIG_REFERENCE.md`
  - `docs/CONFIG_DOCS_COVERAGE.md`
