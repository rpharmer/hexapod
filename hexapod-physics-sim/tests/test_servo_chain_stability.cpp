#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace {

using namespace minphys3d;

struct ServoChain {
    std::uint32_t rootBody = 0;
    std::vector<std::uint32_t> linkBodies;
    std::vector<std::uint32_t> servoJoints;
};

Body MakeBoxBody(const Vec3& position, const Vec3& half_extents, Real mass, bool is_static = false) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.isStatic = is_static;
    body.restitution = 0.0;
    body.staticFriction = 0.8;
    body.dynamicFriction = 0.6;
    body.linearDamping = 0.1;
    body.angularDamping = 0.3;
    return body;
}

Body MakeLinkBody(const Vec3& center, Real half_length, Real mass) {
    return MakeBoxBody(center, {half_length, 0.035, 0.035}, mass, false);
}

Real WrapAngle(Real angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

Real MaxServoAngleError(const World& world, const std::vector<std::uint32_t>& servo_joints) {
    Real max_error = 0.0;
    for (const std::uint32_t joint_id : servo_joints) {
        const ServoJoint& joint = world.GetServoJoint(joint_id);
        const Real error = WrapAngle(world.GetServoJointAngle(joint_id) - joint.targetAngle);
        max_error = std::max(max_error, std::abs(error));
    }
    return max_error;
}

ServoChain BuildPlanarServoChain(
    World& world,
    std::uint32_t root_body,
    const Vec3& root_anchor,
    int link_count,
    Real link_length,
    Real link_mass,
    Real target_angle,
    Real max_servo_impulse,
    Real position_gain,
    Real damping_gain) {
    ServoChain chain;
    chain.rootBody = root_body;

    std::uint32_t parent_body = root_body;
    Vec3 anchor = root_anchor;
    const Real half_length = 0.5 * link_length;
    for (int i = 0; i < link_count; ++i) {
        const Vec3 center = anchor + Vec3{half_length, 0.0, 0.0};
        const std::uint32_t link_body = world.CreateBody(MakeLinkBody(center, half_length, link_mass));
        const std::uint32_t servo_joint = world.CreateServoJoint(
            parent_body,
            link_body,
            anchor,
            {0.0, 0.0, 1.0},
            target_angle,
            max_servo_impulse,
            position_gain,
            damping_gain);
        chain.linkBodies.push_back(link_body);
        chain.servoJoints.push_back(servo_joint);
        parent_body = link_body;
        anchor += Vec3{link_length, 0.0, 0.0};
    }

    return chain;
}

bool CheckFreefallServoChain(int link_count, Real allowed_error) {
    World world({0.0, -9.81, 0.0});

    Body base = MakeBoxBody({0.0, 3.0, 0.0}, {0.20, 0.12, 0.12}, 3.0, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
        link_count,
        0.60,
        0.8,
        0.0,
        15.0,
        18.0,
        2.0);

    Real max_error = 0.0;
    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 240; ++step) {
        world.Step(kDt, 24);
        max_error = std::max(max_error, MaxServoAngleError(world, chain.servoJoints));
    }

    if (max_error > allowed_error) {
        std::cerr << "freefall chain link_count=" << link_count
                  << " max_error=" << max_error
                  << " allowed=" << allowed_error << "\n";
        return false;
    }
    return true;
}

bool CheckStaticBaseLoadedServoChain(int link_count, Real allowed_error) {
    World world({0.0, -9.81, 0.0});

    Body base = MakeBoxBody({0.0, 2.0, 0.0}, {0.20, 0.12, 0.12}, 1.0, true);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
        link_count,
        0.18,
        0.08,
        0.20,
        18.0,
        22.0,
        2.5);

    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 28);
    }

    const Real max_error = MaxServoAngleError(world, chain.servoJoints);
    if (max_error > allowed_error) {
        std::cerr << "loaded chain link_count=" << link_count
                  << " max_error=" << max_error
                  << " allowed=" << allowed_error << "\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    bool ok = true;

    ok = CheckFreefallServoChain(1, 0.25) && ok;
    ok = CheckFreefallServoChain(3, 0.30) && ok;
    ok = CheckFreefallServoChain(5, 0.40) && ok;

    ok = CheckStaticBaseLoadedServoChain(1, 0.25) && ok;

    return ok ? 0 : 1;
}
