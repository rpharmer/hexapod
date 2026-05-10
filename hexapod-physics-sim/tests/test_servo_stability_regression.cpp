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

struct ScenarioMetrics {
    Real peakLinear = 0.0;
    Real peakAngular = 0.0;
    Real maxError = 0.0;
    bool finite = true;
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    bool passCShortChainSkipped = false;
    bool chainPositionShortChainSkipped = false;
#endif
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

Real MaxLinkLinearSpeed(const World& world, const std::vector<std::uint32_t>& link_ids) {
    Real m = 0.0;
    for (const std::uint32_t id : link_ids) {
        m = std::max(m, Length(world.GetBody(id).velocity));
    }
    return m;
}

bool BodyStateFinite(const Body& body) {
    return std::isfinite(body.position.x) && std::isfinite(body.position.y) && std::isfinite(body.position.z)
        && std::isfinite(body.velocity.x) && std::isfinite(body.velocity.y) && std::isfinite(body.velocity.z)
        && std::isfinite(body.angularVelocity.x) && std::isfinite(body.angularVelocity.y)
        && std::isfinite(body.angularVelocity.z);
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

bool CheckFreefallChainMotionBounded() {
    World world({0.0, -9.81, 0.0});

    Body base = MakeBoxBody({0.0, 3.0, 0.0}, {0.20, 0.12, 0.12}, 3.0, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
        3,
        0.60,
        0.8,
        0.0,
        15.0,
        18.0,
        2.0);

    Real max_error = 0.0;
    Real peak_linear = 0.0;
    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 360; ++step) {
        world.Step(kDt, 24);
        max_error = std::max(max_error, MaxServoAngleError(world, chain.servoJoints));
        peak_linear = std::max(peak_linear, MaxLinkLinearSpeed(world, chain.linkBodies));
        peak_linear = std::max(peak_linear, Length(world.GetBody(base_id).velocity));
    }

    constexpr Real kMaxError = 0.32;
    constexpr Real kMaxLinear = 120.0;
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
    auto runScenario = [](bool articulation_enabled) {
        World world({0.0, -9.81, 0.0});
        World::ArticulationConfig articulation_cfg = world.GetArticulationConfig();
        articulation_cfg.enableVelocityPreCorrection = articulation_enabled;
        articulation_cfg.enableVelocityPreCorrectionFullForwardPass = false;
        articulation_cfg.enableChainPositionSolve = articulation_enabled;
        world.SetArticulationConfig(articulation_cfg);

        Body base = MakeBoxBody({0.0, 2.0, 0.0}, {0.20, 0.12, 0.12}, 1.0, true);
        const std::uint32_t base_id = world.CreateBody(base);
        const ServoChain chain = BuildPlanarServoChain(
            world,
            base_id,
            world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
            1,
            0.18,
            0.08,
            0.20,
            10.0,
            14.0,
            2.0);

        ScenarioMetrics metrics;
        constexpr Real kDt = 1.0 / 120.0;
        for (int step = 0; step < 600; ++step) {
            world.Step(kDt, 28);
            metrics.peakLinear = std::max(metrics.peakLinear, MaxLinkLinearSpeed(world, chain.linkBodies));
            metrics.maxError = std::max(metrics.maxError, MaxServoAngleError(world, chain.servoJoints));
            metrics.peakAngular = std::max(metrics.peakAngular, Length(world.GetBody(chain.linkBodies.front()).angularVelocity));
            metrics.finite = metrics.finite && BodyStateFinite(world.GetBody(base_id))
                && BodyStateFinite(world.GetBody(chain.linkBodies.front()));
        }
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        if (!world.GetSolverTelemetry().articulationChains.empty()) {
            const auto& sample = world.GetSolverTelemetry().articulationChains.front();
            metrics.passCShortChainSkipped = sample.passCSkippedShortChain;
            metrics.chainPositionShortChainSkipped = sample.chainPositionSkippedShortChain;
        }
#endif
        return metrics;
    };

    const ScenarioMetrics articulation_on = runScenario(true);
    const ScenarioMetrics articulation_off = runScenario(false);
    if (!articulation_on.finite || !articulation_off.finite) {
        std::cerr << "loaded1 encountered non-finite state\n";
        return false;
    }
    if (articulation_on.peakLinear > articulation_off.peakLinear + 25.0) {
        std::cerr << "loaded1 articulation_on peak_linear=" << articulation_on.peakLinear
                  << " articulation_off=" << articulation_off.peakLinear << "\n";
        return false;
    }
    if (articulation_on.maxError > articulation_off.maxError + 0.15) {
        std::cerr << "loaded1 articulation_on max_error=" << articulation_on.maxError
                  << " articulation_off=" << articulation_off.maxError << "\n";
        return false;
    }
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    if (!articulation_on.chainPositionShortChainSkipped) {
        std::cerr << "loaded1 expected short-chain position fallback to stay active\n";
        return false;
    }
#endif
    return true;
}

bool CheckDeterministicServoChain() {
    auto run_once = []() {
        World world({0.0, -9.81, 0.0});
        Body base = MakeBoxBody({0.0, 2.0, 0.0}, {0.20, 0.12, 0.12}, 1.0, true);
        const std::uint32_t base_id = world.CreateBody(base);
        const ServoChain chain = BuildPlanarServoChain(
            world,
            base_id,
            world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
            2,
            0.18,
            0.08,
            0.15,
            18.0,
            22.0,
            2.5);
        constexpr Real kDt = 1.0 / 120.0;
        for (int step = 0; step < 200; ++step) {
            world.Step(kDt, 28);
        }
        return MaxServoAngleError(world, chain.servoJoints);
    };

    const Real a = run_once();
    const Real b = run_once();
    if (std::abs(a - b) > 1e-5) {
        std::cerr << "determinism mismatch a=" << a << " b=" << b << "\n";
        return false;
    }
    return true;
}

bool CheckZeroGravityServoDrift() {
    World world({0.0, 0.0, 0.0});

    Body a = MakeBoxBody({0.0, 0.0, 0.0}, {0.15, 0.15, 0.15}, 2.0, true);
    const std::uint32_t aid = world.CreateBody(a);
    Body b = MakeBoxBody({0.5, 0.0, 0.0}, {0.20, 0.04, 0.04}, 0.5, false);
    const std::uint32_t bid = world.CreateBody(b);
    const std::uint32_t sj = world.CreateServoJoint(
        aid,
        bid,
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        0.35,
        4.0,
        14.0,
        2.0);

    Real max_err = 0.0;
    constexpr Real kDt = 1.0 / 120.0;
    for (int i = 0; i < 400; ++i) {
        world.Step(kDt, 24);
        const ServoJoint& j = world.GetServoJoint(sj);
        const Real err = std::abs(WrapAngle(world.GetServoJointAngle(sj) - j.targetAngle));
        max_err = std::max(max_err, err);
    }
    constexpr Real kCap = 0.90;
    if (max_err > kCap) {
        std::cerr << "zero_g servo max_err=" << max_err << " cap=" << kCap << "\n";
        return false;
    }
    return true;
}

bool CheckFiveLinkFreefallLong() {
    World world({0.0, -9.81, 0.0});

    Body base = MakeBoxBody({0.0, 3.0, 0.0}, {0.20, 0.12, 0.12}, 3.0, false);
    const std::uint32_t base_id = world.CreateBody(base);
    const ServoChain chain = BuildPlanarServoChain(
        world,
        base_id,
        world.GetBody(base_id).position + Vec3{0.20, 0.0, 0.0},
        5,
        0.60,
        0.8,
        0.0,
        15.0,
        18.0,
        2.0);

    Real peak_linear = 0.0;
    Real max_error = 0.0;
    constexpr Real kDt = 1.0 / 120.0;
    for (int step = 0; step < 800; ++step) {
        world.Step(kDt, 24);
        peak_linear = std::max(peak_linear, MaxLinkLinearSpeed(world, chain.linkBodies));
        peak_linear = std::max(peak_linear, Length(world.GetBody(base_id).velocity));
        max_error = std::max(max_error, MaxServoAngleError(world, chain.servoJoints));
    }

    constexpr Real kMaxLinear = 180.0;
    constexpr Real kMaxError = 0.42;
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
    auto runScenario = [](bool articulation_enabled) {
        World world({0.0, -9.81, 0.0});
        World::ArticulationConfig articulation_cfg = world.GetArticulationConfig();
        articulation_cfg.enableVelocityPreCorrection = articulation_enabled;
        articulation_cfg.enableVelocityPreCorrectionFullForwardPass = false;
        articulation_cfg.enableChainPositionSolve = articulation_enabled;
        world.SetArticulationConfig(articulation_cfg);

        Body plane;
        plane.shape = ShapeType::Plane;
        plane.isStatic = true;
        plane.planeNormal = {0.0, 1.0, 0.0};
        plane.planeOffset = 0.0;
        plane.staticFriction = 0.88;
        plane.dynamicFriction = 0.58;
        plane.restitution = 0.04;
        world.CreateBody(plane);

        Body base = MakeBoxBody({0.0, 0.18, 0.0}, {0.22, 0.06, 0.18}, 1.0, true);
        const std::uint32_t base_id = world.CreateBody(base);

        Body arm = MakeBoxBody({0.29, 0.24, 0.0}, {0.14, 0.05, 0.06}, 0.55, false);
        arm.restitution = 0.05;
        const std::uint32_t arm_id = world.CreateBody(arm);

        const std::uint32_t joint_id = world.CreateServoJoint(
            base_id,
            arm_id,
            {0.15, 0.24, 0.0},
            {0.0, 0.0, 1.0},
            0.45,
            2.0,
            1.0,
            0.4,
            0.0,
            0.5,
            0.2,
            0.2);

        ScenarioMetrics metrics;
        Real final_error = 0.0;
        constexpr Real kDt = 1.0 / 120.0;
        for (int step = 0; step < 360; ++step) {
            world.Step(kDt, 24);
            const Body& current_arm = world.GetBody(arm_id);
            metrics.peakLinear = std::max(metrics.peakLinear, Length(current_arm.velocity));
            metrics.peakAngular = std::max(metrics.peakAngular, Length(current_arm.angularVelocity));
            const ServoJoint& joint = world.GetServoJoint(joint_id);
            final_error = std::abs(WrapAngle(world.GetServoJointAngle(joint_id) - joint.targetAngle));
            metrics.maxError = std::max(metrics.maxError, final_error);
            metrics.finite = metrics.finite && BodyStateFinite(world.GetBody(base_id)) && BodyStateFinite(current_arm);
        }
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
        if (!world.GetSolverTelemetry().articulationChains.empty()) {
            const auto& sample = world.GetSolverTelemetry().articulationChains.front();
            metrics.passCShortChainSkipped = sample.passCSkippedShortChain;
            metrics.chainPositionShortChainSkipped = sample.chainPositionSkippedShortChain;
        }
#endif
        return std::pair<ScenarioMetrics, Real>{metrics, final_error};
    };

    const auto [articulation_on, final_error_on] = runScenario(true);
    const auto [articulation_off, final_error_off] = runScenario(false);

    constexpr Real kMaxLinear = 0.10;
    constexpr Real kMaxAngular = 1.0;
    constexpr Real kMaxError = 0.36;
    constexpr Real kFinalError = 0.02;
    if (!articulation_on.finite || !articulation_off.finite) {
        std::cerr << "vis_servo_arm encountered non-finite state\n";
        return false;
    }
    if (articulation_on.peakLinear > kMaxLinear || articulation_off.peakLinear > kMaxLinear) {
        std::cerr << "vis_servo_arm peak_linear on=" << articulation_on.peakLinear
                  << " off=" << articulation_off.peakLinear << " cap=" << kMaxLinear << "\n";
        return false;
    }
    if (articulation_on.peakAngular > kMaxAngular || articulation_off.peakAngular > kMaxAngular) {
        std::cerr << "vis_servo_arm peak_angular on=" << articulation_on.peakAngular
                  << " off=" << articulation_off.peakAngular << " cap=" << kMaxAngular << "\n";
        return false;
    }
    if (articulation_on.maxError > kMaxError || articulation_off.maxError > kMaxError) {
        std::cerr << "vis_servo_arm max_error on=" << articulation_on.maxError
                  << " off=" << articulation_off.maxError << " cap=" << kMaxError << "\n";
        return false;
    }
    if (final_error_on > kFinalError || final_error_off > kFinalError) {
        std::cerr << "vis_servo_arm final_error on=" << final_error_on
                  << " off=" << final_error_off << " cap=" << kFinalError << "\n";
        return false;
    }
    if (articulation_on.peakLinear > articulation_off.peakLinear + 0.02) {
        std::cerr << "vis_servo_arm articulation_on peak_linear=" << articulation_on.peakLinear
                  << " articulation_off=" << articulation_off.peakLinear << "\n";
        return false;
    }
#if MINPHYS3D_SOLVER_TELEMETRY_ENABLED
    if (!articulation_on.passCShortChainSkipped) {
        std::cerr << "vis_servo_arm expected Pass C short-chain fallback to stay active\n";
        return false;
    }
#endif
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
