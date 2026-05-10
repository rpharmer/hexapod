#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct ChainRig {
    std::uint32_t base = 0;
    std::vector<std::uint32_t> links;
};

ChainRig buildFloatingChain(World& world) {
    ChainRig rig;
    Body base;
    base.shape = ShapeType::Box;
    base.position = {0.0, 0.0, 0.0};
    base.halfExtents = {0.18, 0.10, 0.10};
    base.mass = 2.2;
    base.linearDamping = 0.2;
    base.angularDamping = 0.25;
    rig.base = world.CreateBody(base);

    std::uint32_t parent = rig.base;
    Vec3 anchor = {0.18, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        Body link = MakeArmLink(anchor + Vec3{0.22, 0.0, 0.0}, 0.44, 0.35);
        link.linearDamping = 0.2;
        link.angularDamping = 0.25;
        const std::uint32_t link_id = world.CreateBody(link);
        world.CreateServoJoint(
            parent,
            link_id,
            anchor,
            {0.0, 0.0, 1.0},
            0.0,
            10.0,
            20.0,
            2.2);
        rig.links.push_back(link_id);
        parent = link_id;
        anchor += Vec3{0.44, 0.0, 0.0};
    }
    return rig;
}

int runCase() {
    World world({0.0, 0.0, 0.0});
    const ChainRig rig = buildFloatingChain(world);
    world.GetBody(rig.base).velocity = {3.5, 0.0, 0.0};
    world.GetBody(rig.base).angularVelocity = {0.0, 0.0, 3.0};

    Real initial_speed = 0.0;
    Real final_speed = 0.0;
    Real peak_speed = 0.0;
    constexpr Real kDt = 1.0 / 240.0;
    for (int step = 0; step < 720; ++step) {
        world.Step(kDt, 24);
        const Body& base = world.GetBody(rig.base);
        if (!IsFiniteVec3(base.position) || !IsFiniteVec3(base.velocity) || !IsFiniteVec3(base.angularVelocity)
            || !IsFiniteQuat(base.orientation)) {
            std::cerr << "articulation_decay non-finite base state at step=" << step << "\n";
            return 1;
        }
        const Real speed = Length(base.velocity) + 0.3 * Length(base.angularVelocity);
        if (step == 0) {
            initial_speed = speed;
        }
        final_speed = speed;
        peak_speed = std::max(peak_speed, speed);
    }

    if (initial_speed <= 1.0e-4) {
        std::cerr << "articulation_decay initial disturbance too small\n";
        return 1;
    }
    if (final_speed > 0.80 * initial_speed) {
        std::cerr << "articulation_decay final_speed=" << final_speed
                  << " initial_speed=" << initial_speed << " ratio=" << (final_speed / initial_speed)
                  << " cap_ratio=0.80\n";
        return 1;
    }
    if (peak_speed > 1.25 * initial_speed) {
        std::cerr << "articulation_decay peak_speed=" << peak_speed << " cap=" << (1.25 * initial_speed) << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
