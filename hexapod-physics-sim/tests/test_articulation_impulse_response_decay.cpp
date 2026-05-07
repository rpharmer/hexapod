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
    base.position = {0.0f, 0.0f, 0.0f};
    base.halfExtents = {0.18f, 0.10f, 0.10f};
    base.mass = 2.2f;
    base.linearDamping = 0.2f;
    base.angularDamping = 0.25f;
    rig.base = world.CreateBody(base);

    std::uint32_t parent = rig.base;
    Vec3 anchor = {0.18f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        Body link = MakeArmLink(anchor + Vec3{0.22f, 0.0f, 0.0f}, 0.44f, 0.35f);
        link.linearDamping = 0.2f;
        link.angularDamping = 0.25f;
        const std::uint32_t link_id = world.CreateBody(link);
        world.CreateServoJoint(
            parent,
            link_id,
            anchor,
            {0.0f, 0.0f, 1.0f},
            0.0f,
            10.0f,
            20.0f,
            2.2f);
        rig.links.push_back(link_id);
        parent = link_id;
        anchor += Vec3{0.44f, 0.0f, 0.0f};
    }
    return rig;
}

int runCase() {
    World world({0.0f, 0.0f, 0.0f});
    const ChainRig rig = buildFloatingChain(world);
    world.GetBody(rig.base).velocity = {3.5f, 0.0f, 0.0f};
    world.GetBody(rig.base).angularVelocity = {0.0f, 0.0f, 3.0f};

    float initial_speed = 0.0f;
    float final_speed = 0.0f;
    float peak_speed = 0.0f;
    constexpr float kDt = 1.0f / 240.0f;
    for (int step = 0; step < 720; ++step) {
        world.Step(kDt, 24);
        const Body& base = world.GetBody(rig.base);
        if (!IsFiniteVec3(base.position) || !IsFiniteVec3(base.velocity) || !IsFiniteVec3(base.angularVelocity)
            || !IsFiniteQuat(base.orientation)) {
            std::cerr << "articulation_decay non-finite base state at step=" << step << "\n";
            return 1;
        }
        const float speed = Length(base.velocity) + 0.3f * Length(base.angularVelocity);
        if (step == 0) {
            initial_speed = speed;
        }
        final_speed = speed;
        peak_speed = std::max(peak_speed, speed);
    }

    if (initial_speed <= 1.0e-4f) {
        std::cerr << "articulation_decay initial disturbance too small\n";
        return 1;
    }
    if (final_speed > 0.80f * initial_speed) {
        std::cerr << "articulation_decay final_speed=" << final_speed
                  << " initial_speed=" << initial_speed << " ratio=" << (final_speed / initial_speed)
                  << " cap_ratio=0.80\n";
        return 1;
    }
    if (peak_speed > 1.25f * initial_speed) {
        std::cerr << "articulation_decay peak_speed=" << peak_speed << " cap=" << (1.25f * initial_speed) << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
