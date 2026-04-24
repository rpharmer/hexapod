#include "demo/terrain_patch.hpp"
#include "minphys3d/core/world.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

const resource_monitoring::ResourceSectionSnapshot* findSection(
    const minphys3d::world_resource_monitoring::SectionSummary& summary,
    const char* label) {
    for (std::size_t index = 0; index < summary.count; ++index) {
        const auto& section = summary.sections[index];
        if (section.label != nullptr && std::string_view{section.label} == label) {
            return &section;
        }
    }
    return nullptr;
}

bool testTerrainHeavyWorldProfiling() {
    minphys3d::World world{minphys3d::Vec3{0.0f, -9.81f, 0.0f}};
    minphys3d::demo::TerrainPatchConfig terrain_cfg{};
    terrain_cfg.rows = 10;
    terrain_cfg.cols = 10;
    terrain_cfg.cell_size_m = 0.08f;
    terrain_cfg.base_margin_m = 0.03f;
    terrain_cfg.min_cell_thickness_m = 0.02f;
    minphys3d::demo::TerrainPatch terrain_patch{terrain_cfg};
    terrain_patch.initialize(world, minphys3d::Vec3{0.0f, 0.0f, 0.0f}, 0.0f);
    world.SetTerrainHeightfield(terrain_patch.BuildTerrainHeightfieldAttachment());

    minphys3d::Body sphere{};
    sphere.shape = minphys3d::ShapeType::Sphere;
    sphere.radius = 0.05f;
    sphere.mass = 1.0f;
    sphere.position = minphys3d::Vec3{0.0f, 0.04f, 0.0f};
    const std::uint32_t sphere_id = world.CreateBody(sphere);
    (void)sphere_id;

    std::uint64_t terrain_contact_total = 0;
    for (int i = 0; i < 3; ++i) {
        world.Step(1.0f / 60.0f, 8);
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        terrain_contact_total += world.GetSolverTelemetry().terrainContactAdds;
#endif
    }

    const auto summary = world.SnapshotResourceSections(false);
    if (!expect(summary.count > 0, "world profiler should emit section samples")) {
        return false;
    }

    const auto* generate_contacts = findSection(summary, "world.generate_contacts");
    const auto* solve_islands = findSection(summary, "world.solve_islands");
    if (!expect(generate_contacts != nullptr, "world generate_contacts section should be present") ||
        !expect(solve_islands != nullptr, "world solve_islands section should be present")) {
        return false;
    }

    if (!expect(generate_contacts->call_count > 0, "generate_contacts section should count calls") ||
        !expect(solve_islands->call_count > 0, "solve_islands section should count calls")) {
        return false;
    }

#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    return expect(terrain_contact_total > 0,
                  "terrain-heavy scene should record terrain contact adds");
#else
    return expect(generate_contacts->total_self_ns > 0 && solve_islands->total_self_ns > 0,
                  "terrain-heavy scene should accumulate world section time");
#endif
}

} // namespace

int main() {
    if (!testTerrainHeavyWorldProfiling()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
