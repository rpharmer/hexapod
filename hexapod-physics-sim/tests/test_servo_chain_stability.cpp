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

Body MakeBoxBody(const Vec3& position, const Vec3& half_extents, float mass, bool is_static = false) {
    Body body;
    body.shape = ShapeType::Box;
    body.position = position;
    body.halfExtents = half_extents;
    body.mass = mass;
    body.isStatic = is_static;
    body.restitution = 0.0f;
    body.staticFriction = 0.8f;
    body.dynamicFriction = 0.6f;
    body.linearDamping = 0.1f;
    body.angularDamping = 0.3f;
    return body;
}

Body MakeLinkBody(const Vec3& center, float half_length, float mass) {
    return MakeBoxBody(center, {half_length, 0.035f, 0.035f}, mass, false);
}

float WrapAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

float MaxServoAngleError(const World& world, const std::vector<std::uint32_t>& servo_joints) {
    float max_error = 0.0f;
    for (const std::uint32_t joint_id : servo_joints) {
        const ServoJoint& joint = world.GetServoJoint(joint_id);
        const float error = WrapAngle(world.GetServoJointAngle(joint_id) - joint.targetAngle);
        max_error = std::max(max_error, std::abs(error));
    }
    return max_error;
}

ServoChain BuildPlanarServoChain(
    World& world,
    std::uint32_t root_body,
    const Vec3& root_anchor,
    int link_count,
    float link_length,
    float link_mass,
    float target_angle,
    float max_servo_impulse,
    float position_gain,
    float damping_gain) {
    ServoChain chain;
    chain.rootBody = root_body;

    std::uint32_t parent_body = root_body;
    Vec3 anchor = root_anchor;
    const float half_length = 0.5f * link_length;
    for (int i = 0; i < link_count; ++i) {
        const Vec3 center = anchor + Vec3{half_length, 0.0f, 0.0f};
        const std::uint32_t link_body = world.CreateBody(MakeLinkBody(center, half_length, link_mass));
        const std::uint32_t servo_joint = world.CreateServoJoint(
            parent_body,
            link_body,
            anchor,
            {0.0f, 0.0f, 1.0f},
            target_angle,
            max_servo_impulse,
            position_gain,
            damping_gain);
        chain.linkBodies.push_back(link_body);
        chain.servoJoints.push_back(servo_joint);
        parent_body = link_body;
        anchor += Vec3{link_length, 0.0f, 0.0f};
    }

    return chain;
}

bool CheckFreefallServoChain(int link_count, float allowed_error) {
    World world({0.0f, -9.81f, 0.0f});

    Body base = MakeBoxBody({0.0f, 3.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 3.0f, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
        link_count,
        0.60f,
        0.8f,
        0.0f,
        15.0f,
        18.0f,
        2.0f);

    float max_error = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
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

bool CheckStaticBaseLoadedServoChain(int link_count, float allowed_error) {
    World world({0.0f, -9.81f, 0.0f});

    Body base = MakeBoxBody({0.0f, 2.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 1.0f, true);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
        link_count,
        0.18f,
        0.08f,
        0.20f,
        18.0f,
        22.0f,
        2.5f);

    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 28);
    }

    const float max_error = MaxServoAngleError(world, chain.servoJoints);
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

    ok = CheckFreefallServoChain(1, 0.25f) && ok;
    ok = CheckFreefallServoChain(3, 0.30f) && ok;
    ok = CheckFreefallServoChain(5, 0.40f) && ok;

    ok = CheckStaticBaseLoadedServoChain(1, 0.25f) && ok;

    return ok ? 0 : 1;
}
