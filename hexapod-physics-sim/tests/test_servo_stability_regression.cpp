// Broader servo + gravity stability checks. Intended to catch regressions in joint solving,
// warm starting, and runaway motion. Thresholds are conservative; tighten as the solver improves.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
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

float MaxLinkLinearSpeed(const World& world, const std::vector<std::uint32_t>& link_ids) {
    float m = 0.0f;
    for (const std::uint32_t id : link_ids) {
        m = std::max(m, Length(world.GetBody(id).velocity));
    }
    return m;
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

bool CheckFreefallChainMotionBounded() {
    World world({0.0f, -9.81f, 0.0f});

    Body base = MakeBoxBody({0.0f, 3.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 3.0f, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
        3,
        0.60f,
        0.8f,
        0.0f,
        15.0f,
        18.0f,
        2.0f);

    float max_error = 0.0f;
    float peak_linear = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 24);
        max_error = std::max(max_error, MaxServoAngleError(world, chain.servoJoints));
        peak_linear = std::max(peak_linear, MaxLinkLinearSpeed(world, chain.linkBodies));
        peak_linear = std::max(peak_linear, Length(world.GetBody(base_id).velocity));
    }

    constexpr float kMaxError = 0.32f;
    constexpr float kMaxLinear = 120.0f;
    if (max_error > kMaxError) {
        std::cerr << "freefall3 max_error=" << max_error << " cap=" << kMaxError << "\n";
        return false;
    }
    if (peak_linear > kMaxLinear) {
        std::cerr << "freefall3 peak_linear=" << peak_linear << " cap=" << kMaxLinear << "\n";
        return false;
    }
    return true;
}

bool CheckLoadedSingleChainMotionBounded() {
    World world({0.0f, -9.81f, 0.0f});

    Body base = MakeBoxBody({0.0f, 2.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 1.0f, true);
    const std::uint32_t base_id = world.CreateBody(base);
    // Keep PD gains moderate: very high position gain + tiny link mass + gravity can spike
    // linear velocity into km/s even with a static base; this scenario is for bounded motion, not stress.
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
        1,
        0.18f,
        0.08f,
        0.20f,
        10.0f,
        14.0f,
        2.0f);

    float peak_linear = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 600; ++step) {
        world.Step(kDt, 28);
        peak_linear = std::max(peak_linear, MaxLinkLinearSpeed(world, chain.linkBodies));
    }

    // Baseline observed ~683 m/s peak on this scenario (2026-04); cap guards order-of-magnitude blowups.
    constexpr float kMaxLinear = 900.0f;
    if (peak_linear > kMaxLinear) {
        std::cerr << "loaded1 peak_linear=" << peak_linear << " cap=" << kMaxLinear << "\n";
        return false;
    }
    return true;
}

bool CheckDeterministicServoChain() {
    auto run_once = []() {
        World world({0.0f, -9.81f, 0.0f});
        Body base = MakeBoxBody({0.0f, 2.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 1.0f, true);
        const std::uint32_t base_id = world.CreateBody(base);
        const ServoChain chain = BuildPlanarServoChain(
            world,
            base_id,
            world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
            2,
            0.18f,
            0.08f,
            0.15f,
            18.0f,
            22.0f,
            2.5f);
        constexpr float kDt = 1.0f / 120.0f;
        for (int step = 0; step < 200; ++step) {
            world.Step(kDt, 28);
        }
        return MaxServoAngleError(world, chain.servoJoints);
    };

    const float a = run_once();
    const float b = run_once();
    if (std::abs(a - b) > 1e-5f) {
        std::cerr << "determinism mismatch a=" << a << " b=" << b << "\n";
        return false;
    }
    return true;
}

bool CheckZeroGravityServoDrift() {
    World world({0.0f, 0.0f, 0.0f});

    Body a = MakeBoxBody({0.0f, 0.0f, 0.0f}, {0.15f, 0.15f, 0.15f}, 2.0f, true);
    const std::uint32_t aid = world.CreateBody(a);
    Body b = MakeBoxBody({0.5f, 0.0f, 0.0f}, {0.20f, 0.04f, 0.04f}, 0.5f, false);
    const std::uint32_t bid = world.CreateBody(b);
    const std::uint32_t sj = world.CreateServoJoint(
        aid,
        bid,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        0.35f,
        4.0f,
        14.0f,
        2.0f);

    float max_err = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
    for (int i = 0; i < 400; ++i) {
        world.Step(kDt, 24);
        const ServoJoint& j = world.GetServoJoint(sj);
        const float err = std::abs(WrapAngle(world.GetServoJointAngle(sj) - j.targetAngle));
        max_err = std::max(max_err, err);
    }
    constexpr float kCap = 0.90f;
    if (max_err > kCap) {
        std::cerr << "zero_g servo max_err=" << max_err << " cap=" << kCap << "\n";
        return false;
    }
    return true;
}

bool CheckFiveLinkFreefallLong() {
    World world({0.0f, -9.81f, 0.0f});

    Body base = MakeBoxBody({0.0f, 3.0f, 0.0f}, {0.20f, 0.12f, 0.12f}, 3.0f, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20f, 0.0f, 0.0f},
        5,
        0.60f,
        0.8f,
        0.0f,
        15.0f,
        18.0f,
        2.0f);

    float peak_linear = 0.0f;
    float max_error = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 800; ++step) {
        world.Step(kDt, 24);
        peak_linear = std::max(peak_linear, MaxLinkLinearSpeed(world, chain.linkBodies));
        peak_linear = std::max(peak_linear, Length(world.GetBody(base_id).velocity));
        max_error = std::max(max_error, MaxServoAngleError(world, chain.servoJoints));
    }

    constexpr float kMaxLinear = 180.0f;
    constexpr float kMaxError = 0.42f;
    if (peak_linear > kMaxLinear) {
        std::cerr << "freefall5 peak_linear=" << peak_linear << " cap=" << kMaxLinear << "\n";
        return false;
    }
    if (max_error > kMaxError) {
        std::cerr << "freefall5 max_error=" << max_error << " cap=" << kMaxError << "\n";
        return false;
    }
    return true;
}

bool CheckVisualServoArmPresetRemainsBounded() {
    World world({0.0f, -9.81f, 0.0f});

    Body plane;
    plane.shape = ShapeType::Plane;
    plane.isStatic = true;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.staticFriction = 0.88f;
    plane.dynamicFriction = 0.58f;
    plane.restitution = 0.04f;
    world.CreateBody(plane);

    Body base = MakeBoxBody({0.0f, 0.18f, 0.0f}, {0.22f, 0.06f, 0.18f}, 1.0f, true);
    const std::uint32_t base_id = world.CreateBody(base);

    Body arm = MakeBoxBody({0.29f, 0.24f, 0.0f}, {0.14f, 0.05f, 0.06f}, 0.55f, false);
    arm.restitution = 0.05f;
    const std::uint32_t arm_id = world.CreateBody(arm);

    const std::uint32_t joint_id = world.CreateServoJoint(
        base_id,
        arm_id,
        {0.15f, 0.24f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        0.45f,
        2.0f,
        1.0f,
        0.4f,
        0.0f,
        0.5f,
        0.2f,
        0.2f);

    float peak_linear = 0.0f;
    float peak_angular = 0.0f;
    float max_error = 0.0f;
    float final_error = 0.0f;
    constexpr float kDt = 1.0f / 120.0f;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 24);
        const Body& current_arm = world.GetBody(arm_id);
        peak_linear = std::max(peak_linear, Length(current_arm.velocity));
        peak_angular = std::max(peak_angular, Length(current_arm.angularVelocity));
        const ServoJoint& joint = world.GetServoJoint(joint_id);
        final_error = std::abs(WrapAngle(world.GetServoJointAngle(joint_id) - joint.targetAngle));
        max_error = std::max(max_error, final_error);
    }

    constexpr float kMaxLinear = 0.10f;
    constexpr float kMaxAngular = 1.0f;
    constexpr float kMaxError = 0.36f;
    constexpr float kFinalError = 0.02f;
    if (peak_linear > kMaxLinear) {
        std::cerr << "vis_servo_arm peak_linear=" << peak_linear << " cap=" << kMaxLinear << "\n";
        return false;
    }
    if (peak_angular > kMaxAngular) {
        std::cerr << "vis_servo_arm peak_angular=" << peak_angular << " cap=" << kMaxAngular << "\n";
        return false;
    }
    if (max_error > kMaxError) {
        std::cerr << "vis_servo_arm max_error=" << max_error << " cap=" << kMaxError << "\n";
        return false;
    }
    if (final_error > kFinalError) {
        std::cerr << "vis_servo_arm final_error=" << final_error << " cap=" << kFinalError << "\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok = CheckFreefallChainMotionBounded() && ok;
    ok = CheckLoadedSingleChainMotionBounded() && ok;
    ok = CheckDeterministicServoChain() && ok;
    ok = CheckZeroGravityServoDrift() && ok;
    ok = CheckFiveLinkFreefallLong() && ok;
    ok = CheckVisualServoArmPresetRemainsBounded() && ok;
    return ok ? 0 : 1;
}
