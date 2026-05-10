// Links hexapod-server gravity feedforward helper to ensure the analytical model builds
// in the physics-sim tree and satisfies basic scaling / monotonicity checks.

#include "control/joint_angle_gravity_feedforward.hpp"
#include "hexapod_dynamics_constants.hpp"
#include "kinematics/types.hpp"

#include <cmath>
#include <iostream>

namespace {

bool ok(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "FAIL: " << msg << '\n';
    }
    return cond;
}

} // namespace

int main() {
    LegGeometry leg{};
    leg.coxaLength = LengthM{hexapod_dynamics::kCoxaLengthM};
    leg.femurLength = LengthM{hexapod_dynamics::kFemurLengthM};
    leg.tibiaLength = LengthM{hexapod_dynamics::kTibiaLinkLengthM};
    leg.mountAngle = AngleRad{0.35};

    control_config::GravityFeedforwardConfig cfg{};
    cfg.scale_coxa = 0.0;
    cfg.scale_femur = 1.0;
    cfg.scale_tibia = 1.0;
    cfg.include_foot_reaction = true;
    cfg.include_self_weight = false;

    const Vec3 g_down{0.0, 0.0, -1.0};
    const double F = hexapod_dynamics::kBodyMassKg * hexapod_dynamics::kStandardGravityMps2 / 6.0;

    const LegGravityCompensation half =
        computeLegGravityCompensation(leg, 0.0, 0.5, -0.2, g_down, 0.5 * F, cfg);
    const LegGravityCompensation full = computeLegGravityCompensation(leg, 0.0, 0.5, -0.2, g_down, F, cfg);
    if (!ok(std::abs(half.delta_femur_rad * 2.0 - full.delta_femur_rad) < 5e-7 &&
                std::abs(half.delta_tibia_rad * 2.0 - full.delta_tibia_rad) < 5e-7,
            "foot reaction scaling should double sag (unsaturated)")) {
        return 1;
    }

    LegGeometry leg_mirror = leg;
    leg_mirror.mountAngle = AngleRad{-leg.mountAngle.value};
    const LegGravityCompensation p = computeLegGravityCompensation(leg, 0.1, 0.6, -0.3, g_down, F, cfg);
    const LegGravityCompensation m = computeLegGravityCompensation(leg_mirror, 0.1, 0.6, -0.3, g_down, F, cfg);
    if (!ok(std::abs(std::abs(p.delta_femur_rad) - std::abs(m.delta_femur_rad)) < 1e-8,
            "mirrored mount should match femur sag magnitude")) {
        return 1;
    }

    const LegGravityCompensation ext = computeLegGravityCompensation(leg, 0.0, 0.2, 0.0, g_down, F, cfg);
    const LegGravityCompensation flex = computeLegGravityCompensation(leg, 0.0, 1.1, -0.6, g_down, F, cfg);
    if (!ok(std::abs(flex.delta_femur_rad - ext.delta_femur_rad) > 1e-7,
            "pose change should alter femur sag prediction")) {
        return 1;
    }

    control_config::GravityFeedforwardConfig cfg_soft = cfg;
    cfg_soft.stiffness_gain_scale = 0.5;
    control_config::GravityFeedforwardConfig cfg_stiff = cfg;
    cfg_stiff.stiffness_gain_scale = 1.0;
    const LegGravityCompensation sag_soft =
        computeLegGravityCompensation(leg, 0.0, 0.55, -0.25, g_down, F, cfg_soft);
    const LegGravityCompensation sag_stiff =
        computeLegGravityCompensation(leg, 0.0, 0.55, -0.25, g_down, F, cfg_stiff);
    if (!ok(std::abs(std::abs(sag_soft.delta_femur_rad) - 2.0 * std::abs(sag_stiff.delta_femur_rad)) < 1e-6,
            "halving stiffness_gain_scale should double femur sag magnitude (same τ)")) {
        return 1;
    }

    return 0;
}
