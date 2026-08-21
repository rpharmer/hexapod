# minphys3d solver speedups — CPU A/B snapshot

Measurements for the combined physics-sim patch (`~/hexapod_physics_speedups_combined.patch`): friction `K` cache, servo PGS deferred writes, inlined `SolveIslands`, and persistent-point capture reuse/dedup.

The patch is applied in the working tree. Baseline binaries were kept in `hexapod-physics-sim/build-before`; patched binaries in `hexapod-physics-sim/build-after`. Raw logs: `docs/perf/speedup-ab-2026-08-20/`.

## Environment

| Field | Value |
| --- | --- |
| Date | 2026-08-20 |
| Recorded sim git | `3dc8294` (`record_hexapod_stability`) |
| CPU | Intel Core i7-8750H @ 2.20 GHz |
| Parallel jobs | 12 (`nproc`) |
| Compiler | g++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0 |
| CMake build | `RelWithDebInfo` (`-O3 -march=native -ffp-contract=off`) |
| OS | Linux 6.18.40.1-microsoft-standard-WSL2 |

`MINPHYS3D_PROFILE_INNER_LOOPS` was left at the default `0`.

## Correctness

| Check | Before | After |
| --- | --- | --- |
| `ctest -j$(nproc)` | 52/52 passed, elapsed=14.71 s | 52/52 passed, elapsed=13.81 s |
| `record_hexapod_stability` pose-hold / standing window | see below | **identical** printed scalars |

Pose-hold fields that matched exactly: `peak_linear_speed=0.333871`, `peak_linear_settled_speed=0.046383`, `peak_angular_speed=0.170694`, `peak_joint_error_rad=0.040473`, `final_chassis_y=0.096639`, `final_speed=0.035582`, standing height/roll/pitch/joint-speed window.

Call counts in `world_resource_profile_oneoff` were unchanged (`world.step` 240, inner island sections 7200). Persistence-age growth is slower after the capture-dedup, but it did not move this pose-hold envelope.

## Primary wall clock: dummy hexapod soak

`hexapod-physics-sim --sink dummy --model hexapod --frames 6000` (5× `/usr/bin/time`).

| Side | Runs (s) | min | median | max |
| --- | --- | --- | --- | --- |
| before | 2.94, 2.79, 2.80, 2.84, 2.91 | 2.79 | **2.84** | 2.94 |
| after | 2.47, 2.46, 2.61, 2.51, 2.41 | 2.41 | **2.47** | 2.61 |

Median **13.0% faster** (after/before = 0.870). This is the more reliable wall-clock of the two workloads: ~2.8 s vs ~50 ms.

## Section profiler: `world_resource_profile_oneoff`

Workload: 120 outer frames @ 60 Hz, 2 substeps, 30 solver iters/substep (`kHexapodPoseHoldBenchmark*` in `hexapod_stability.hpp`). Five runs; values are `total_ms` self-time. The 120-frame job is short enough that WSL noise is visible (one after run spiked to 65.8 ms); use medians.

| Section | before median (ms) | after median (ms) | after/before |
| --- | --- | --- | --- |
| wall `wall_ms` | 56.322 | 51.941 | 0.922 (**7.8% faster**) |
| `world.step` | 56.256 | 51.878 | 0.922 |
| `world.solve_islands` | 29.775 | 25.625 | 0.861 (**13.9% faster**) |
| `world.solve_islands.joints_servo` | 10.942 | 7.297 | 0.667 (**33.3% faster**) |
| `world.solve_islands.contacts_forward` | 14.933 | 14.881 | 0.997 (flat) |
| `world.build_manifolds` | 2.648 | 3.042 | 1.149 (ranges overlap; likely noise / larger `Contact`) |

Servo PGS is the clear win. Friction `K` cache did not move `contacts_forward` on this standing-hexapod workload. `build_manifolds` medians moved the wrong way but the before range (2.30–3.78) and after range (2.55–4.26) overlap.

## How to re-run

From repo root:

```bash
# patched tree
cmake -S hexapod-physics-sim -B hexapod-physics-sim/build-after -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build hexapod-physics-sim/build-after -j"$(nproc)"
./hexapod-physics-sim/build-after/world_resource_profile_oneoff
./hexapod-physics-sim/build-after/hexapod-physics-sim --sink dummy --model hexapod --frames 6000
ctest --test-dir hexapod-physics-sim/build-after -j"$(nproc)" --output-on-failure
```
