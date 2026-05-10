#include "demo/terrain_patch.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

namespace {

bool Near(minphys3d::Real a, minphys3d::Real b, minphys3d::Real eps) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    using namespace minphys3d;
    using namespace minphys3d::demo;

    World world;
    TerrainPatch patch{};
    patch.initialize(world, Vec3{0.0, 0.0, 0.0}, 0.0);

    const Real flat_height = patch.SampleHeightWorld(0.0, 0.0);
    if (!Near(flat_height, 0.0, 1.0e-3)) {
        std::cerr << "expected flat terrain near 0, got " << flat_height << "\n";
        return 1;
    }
    if (!Near(patch.grid_origin_x(), -0.35, 1.0e-5) ||
        !Near(patch.grid_origin_z(), -0.35, 1.0e-5)) {
        std::cerr << "unexpected default grid origin\n";
        return 1;
    }

    const Vec3 flat_normal = patch.SampleNormalWorld(0.0, 0.0);
    if (flat_normal.y < 0.95) {
        std::cerr << "expected upward normal on flat terrain, got y=" << flat_normal.y << "\n";
        return 2;
    }

    Real flat_hit = 0.0;
    if (!patch.RaycastWorld(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0}, flat_hit)) {
        std::cerr << "expected raycast hit on flat terrain\n";
        return 3;
    }
    if (!Near(flat_hit, 1.0, 0.08)) {
        std::cerr << "expected flat terrain hit near 1.0, got " << flat_hit << "\n";
        return 4;
    }

    std::vector<TerrainSample> bump_samples{
        TerrainSample{Vec3{0.0, 0.0, 0.0}, 0.18, 1.0},
    };
    patch.update(world, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 1.0, 0.0}, bump_samples, 0.0);

    const Real bump_height = patch.SampleHeightWorld(0.0, 0.0);
    if (bump_height <= 0.03) {
        std::cerr << "expected terrain bump to raise height, got " << bump_height << "\n";
        return 5;
    }

    Real bump_hit = 0.0;
    if (!patch.RaycastWorld(Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0}, bump_hit)) {
        std::cerr << "expected raycast hit on bumped terrain\n";
        return 6;
    }
    if (!(bump_hit < flat_hit - 0.03)) {
        std::cerr << "expected bumped terrain hit closer than flat hit (flat=" << flat_hit
                  << " bump=" << bump_hit << ")\n";
        return 7;
    }

    patch.update(world, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 1.0, 0.0}, {}, 5.0);
    const Real decayed_height = patch.SampleHeightWorld(0.0, 0.0);
    if (!(decayed_height < bump_height && decayed_height < 0.08)) {
        std::cerr << "expected bump to decay back toward plane, bump=" << bump_height
                  << " decayed=" << decayed_height << "\n";
        return 8;
    }

    Real conf_at = 0.0;
    (void)patch.SampleHeightAndConfidenceWorld(0.0, 0.0, &conf_at);
    if (!std::isfinite(conf_at) || conf_at < 0.0 || conf_at > 1.0) {
        std::cerr << "SampleHeightAndConfidenceWorld confidence out of range\n";
        return 9;
    }

    {
        TerrainPatchConfig narrow_cfg{};
        narrow_cfg.rows = 5;
        narrow_cfg.cols = 5;
        narrow_cfg.cell_size_m = 0.02;
        narrow_cfg.base_update_blend = 1.0;
        narrow_cfg.decay_update_boost = 0.0;
        World world_n;
        TerrainPatch narrow{narrow_cfg};
        narrow.initialize(world_n, Vec3{0.0, 0.0, 0.0}, 0.0);
        narrow.update(world_n,
                        Vec3{0.0, 0.0, 0.0},
                        0.0,
                        Vec3{0.0, 1.0, 0.0},
                        std::vector<TerrainSample>{
                            TerrainSample{Vec3{0.0, 0.0, 0.0}, 0.09, 1.0},
                        },
                        0.0);
        Real narrow_hit = 0.0;
        if (!narrow.RaycastWorld(Vec3{0.0, 0.25, 0.0}, Vec3{0.0, -1.0, 0.0}, narrow_hit)) {
            std::cerr << "expected narrow-grid raycast hit\n";
            return 10;
        }
        if (narrow_hit > 0.22) {
            std::cerr << "expected narrow bump hit well below 0.25, got " << narrow_hit << "\n";
            return 11;
        }
    }

    {
        TerrainPatchConfig ray_cfg{};
        ray_cfg.rows = 9;
        ray_cfg.cols = 9;
        ray_cfg.cell_size_m = 0.05;
        World world_r;
        TerrainPatch pr{ray_cfg};
        pr.initialize(world_r, Vec3{0.0, 0.0, 0.0}, 0.0);

        Real shallow_hit = 0.0;
        if (!pr.RaycastWorld(Vec3{-0.18, 0.003, 0.0}, Vec3{1.0, -0.01, 0.0}, shallow_hit)) {
            std::cerr << "expected shallow grazing ray to hit flat terrain\n";
            return 12;
        }
        if (!Near(shallow_hit, 0.30, 0.03)) {
            std::cerr << "unexpected shallow grazing hit distance " << shallow_hit << "\n";
            return 13;
        }

        Real tangent_miss = 0.0;
        if (pr.RaycastWorld(Vec3{-0.18, 0.02, 0.0}, Vec3{1.0, 0.0, 0.0}, tangent_miss)) {
            std::cerr << "horizontal ray above terrain should miss\n";
            return 14;
        }

        Real edge_hit = 0.0;
        if (!pr.RaycastWorld(Vec3{-0.25, 0.004, 0.20}, Vec3{1.0, -0.01, 0.0}, edge_hit)) {
            std::cerr << "expected patch-edge grazing ray to hit\n";
            return 15;
        }

        Real boundary_hit = 0.0;
        if (!pr.RaycastWorld(Vec3{-0.15, 0.006, -0.15}, Vec3{1.0, -0.02, 1.0}, boundary_hit)) {
            std::cerr << "expected diagonal cell-boundary ray to hit\n";
            return 16;
        }
    }

    {
        TerrainPatchConfig bin_cfg{};
        bin_cfg.rows = 9;
        bin_cfg.cols = 9;
        bin_cfg.use_sample_binning = true;
        bin_cfg.sample_bin_size_m = 0.2;
        bin_cfg.base_update_blend = 1.0;
        World world_b;
        TerrainPatch pb{bin_cfg};
        pb.initialize(world_b, Vec3{0.0, 0.0, 0.0}, 0.0);
        std::vector<TerrainSample> many{};
        for (int i = 0; i < 24; ++i) {
            const Real x = 0.01 * static_cast<float>(i - 12);
            many.push_back(TerrainSample{Vec3{x, 0.0, 0.02 * static_cast<float>(i % 5)}, 0.04, 0.5});
        }
        pb.update(world_b, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 1.0, 0.0}, many, 0.0);
        if (!std::isfinite(pb.SampleHeightWorld(0.0, 0.0))) {
            std::cerr << "binning path produced non-finite height\n";
            return 17;
        }
    }

    {
        TerrainPatchConfig cons_cfg{};
        cons_cfg.rows = 7;
        cons_cfg.cols = 7;
        cons_cfg.use_conservative_collision = true;
        cons_cfg.base_update_blend = 1.0;
        cons_cfg.decay_update_boost = 0.0;
        World world_c;
        TerrainPatch pc{cons_cfg};
        pc.initialize(world_c, Vec3{0.0, 0.0, 0.0}, 0.0);
        pc.update(world_c,
                    Vec3{0.0, 0.0, 0.0},
                    0.0,
                    Vec3{0.0, 1.0, 0.0},
                    std::vector<TerrainSample>{
                        TerrainSample{Vec3{0.0, 0.0, 0.0}, 0.14, 1.0},
                    },
                    0.0);
        if (!pc.has_collision_layer()) {
            std::cerr << "expected collision layer when conservative collision enabled\n";
            return 18;
        }
        for (std::size_t i = 0; i < pc.surface_heights_m().size(); ++i) {
            if (pc.collision_heights_m()[i] + 1.0e-4 < pc.surface_heights_m()[i]) {
                std::cerr << "collision height below surface at " << i << "\n";
                return 19;
            }
        }
    }

    {
        TerrainPatchConfig perf_cfg{};
        perf_cfg.rows = 64;
        perf_cfg.cols = 64;
        perf_cfg.cell_size_m = 0.025;
        perf_cfg.use_sample_binning = true;
        perf_cfg.sample_bin_size_m = 0.12;
        perf_cfg.base_update_blend = 1.0;
        perf_cfg.decay_update_boost = 0.0;
        World world_p;
        TerrainPatch pp{perf_cfg};
        pp.initialize(world_p, Vec3{0.0, 0.0, 0.0}, 0.0);
        std::vector<TerrainSample> samples;
        samples.reserve(256);
        for (int i = 0; i < 256; ++i) {
            const Real x = -0.70 + 0.10 * static_cast<float>(i % 15);
            const Real z = -0.70 + 0.10 * static_cast<float>((i / 15) % 15);
            const Real h = 0.03 * std::sin(static_cast<float>(i) * 0.11);
            samples.push_back(TerrainSample{Vec3{x, h, z}, h, 0.4});
        }

        const auto start = std::chrono::steady_clock::now();
        pp.update(world_p, Vec3{0.0, 0.0, 0.0}, 0.0, Vec3{0.0, 1.0, 0.0}, samples, 0.0);
        int hits = 0;
        for (int i = 0; i < 512; ++i) {
            Real t = 0.0;
            const Real z = -0.70 + 1.40 * static_cast<float>(i % 64) / 63.0;
            if (pp.RaycastWorld(Vec3{-0.75, 0.20, z}, Vec3{1.0, -0.18, 0.03}, t)) {
                ++hits;
            }
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        if (hits < 64 || elapsed.count() > 2000) {
            std::cerr << "64x64 terrain perf guard failed hits=" << hits
                      << " elapsed_ms=" << elapsed.count() << "\n";
            return 20;
        }
    }

    std::cout << "test_terrain_patch ok height=" << decayed_height << "\n";
    return 0;
}
