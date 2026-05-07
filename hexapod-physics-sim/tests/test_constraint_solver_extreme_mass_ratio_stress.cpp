#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

int runCase() {
    World world({0.0f, -9.81f, 0.0f});
    ContactSolverConfig cfg = world.GetContactSolverConfig();
    cfg.useBlockSolver = true;
    cfg.enableDeterministicOrdering = true;
    world.SetContactSolverConfig(cfg);

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.isStatic = true;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    world.CreateBody(plane);

    Body light;
    light.shape = ShapeType::Box;
    light.halfExtents = {0.15f, 0.15f, 0.15f};
    light.mass = 0.02f;
    light.position = {0.0f, 0.18f, 0.0f};
    light.restitution = 0.0f;
    light.linearDamping = 0.03f;
    light.angularDamping = 0.03f;
    const std::uint32_t light_id = world.CreateBody(light);

    Body heavy;
    heavy.shape = ShapeType::Box;
    heavy.halfExtents = {0.22f, 0.22f, 0.22f};
    heavy.mass = 5000.0f;
    heavy.position = {0.0f, 0.75f, 0.0f};
    heavy.restitution = 0.0f;
    heavy.linearDamping = 0.02f;
    heavy.angularDamping = 0.02f;
    const std::uint32_t heavy_id = world.CreateBody(heavy);

    float max_abs_speed = 0.0f;
    float min_light_y = std::numeric_limits<float>::infinity();
    std::size_t max_manifolds = 0;
    constexpr float kDt = 1.0f / 240.0f;
    for (int step = 0; step < 900; ++step) {
        world.Step(kDt, 6);
        const Body& b_light = world.GetBody(light_id);
        const Body& b_heavy = world.GetBody(heavy_id);
        if (!IsFiniteVec3(b_light.position) || !IsFiniteVec3(b_light.velocity) || !IsFiniteQuat(b_light.orientation)
            || !IsFiniteVec3(b_heavy.position) || !IsFiniteVec3(b_heavy.velocity) || !IsFiniteQuat(b_heavy.orientation)) {
            std::cerr << "mass_ratio_stress produced non-finite state at step=" << step << "\n";
            return 1;
        }
        max_abs_speed = std::max(max_abs_speed, std::max(Length(b_light.velocity), Length(b_heavy.velocity)));
        min_light_y = std::min(min_light_y, b_light.position.y);
        max_manifolds = std::max(max_manifolds, world.DebugManifolds().size());
    }

    if (max_abs_speed > 140.0f) {
        std::cerr << "mass_ratio_stress max_speed=" << max_abs_speed << " cap=140\n";
        return 1;
    }
    if (min_light_y < -0.35f) {
        std::cerr << "mass_ratio_stress min_light_y=" << min_light_y << " floor=-0.35\n";
        return 1;
    }
    if (max_manifolds == 0) {
        std::cerr << "mass_ratio_stress expected contact manifolds but found none\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
