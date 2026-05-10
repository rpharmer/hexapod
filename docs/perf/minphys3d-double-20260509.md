# minphys3d `Real` = `double` — CPU timing snapshot

Measurements taken after migrating `hexapod-physics-sim` internals from `float` to `minphys3d::Real` (`double`), with UDP / `physics_sim_protocol` fields unchanged (`float32`).

## Environment

| Field | Value |
| --- | --- |
| Date | 2026-05-09 |
| Git SHA | `3dd1add755b54624e6c5f1faa7ee157375d82b7e` |
| CPU | Intel Core i7-8750H @ 2.20 GHz |
| Parallel jobs | 12 (`nproc`) |
| Compiler | g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0 |
| CMake build | `RelWithDebInfo` (`hexapod-physics-sim/build`) |
| OS | Linux 6.6.87.2-microsoft-standard-WSL2+ |

## Full `ctest` suite (52 tests)

From `hexapod-physics-sim/build`:

```text
/usr/bin/time -f 'elapsed=%e user=%U sys=%S' ctest -j$(nproc)
elapsed=13.72 user=27.11 sys=0.21
```

(`elapsed` is wall-clock seconds; `user` aggregates CPU time across parallel tests.)

## Representative tests (5× wall seconds from `/usr/bin/time`)

| Test | Runs (s) | min | median | max |
| --- | --- | --- | --- | --- |
| `test_broadphase_dynamic_tree` | 12.59, 12.74, 12.60, 12.72, 12.72 | 12.59 | 12.68 | 12.74 |
| `regression_scene_suite` | 4.92, 4.84, 4.81, 4.89, 5.24 | 4.81 | 4.89 | 5.24 |
| `test_hexapod_stand_quiescence` | 0.35, 0.34, 0.36, 0.34, 0.35 | 0.34 | 0.35 | 0.36 |
| `test_hexapod_substep_convergence` | 0.64, 0.64, 0.62, 0.64, 0.64 | 0.62 | 0.64 | 0.64 |

Each row used: `ctest -R '^<name>$' -j1` inside `/usr/bin/time -f '%e'`.

## Baseline (`float`) pairing

No pre-migration `float` run was captured in the same session. For a fair A/B on this machine, check out the parent commit before the `Real` migration, rebuild with the same `CMAKE_BUILD_TYPE` and compiler flags, and repeat the commands above (same governor / background load if possible).

## `perf`

`perf stat` was not available on this WSL2 kernel (`linux-tools` package missing). Install distro `linux-tools-*` matching the running kernel to collect `cycles` / `instructions` on a fixed workload.
