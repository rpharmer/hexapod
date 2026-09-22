#pragma once

#include "control_config.hpp"
#include "locomotion_feasibility.hpp"
#include "types.hpp"

/** Per-leg bounded compensation plus unscaled torque/stiffness audit fields. */
struct LegGravityCompensation {
    double delta_coxa_rad{0.0};
    double delta_femur_rad{0.0};
    double delta_tibia_rad{0.0};
    // Unscaled holding torque and estimated PD stiffness, for model audits.
    double torque_femur_nm{0.0};
    double torque_tibia_nm{0.0};
    double stiffness_femur_nm_per_rad{0.0};
    double stiffness_tibia_nm_per_rad{0.0};
};

/**
 * Quasi-static gravity compensation from FK, equal-share stance reaction, optional link
 * self-weight, and PD stiffness (ωₙ² I) mapping. Pure for unit tests.
 *
 * @param g_down_body_unit  Unit gravity direction in **body** frame (same convention as IMU path in apply()).
 * @param foot_reaction_n   Magnitude of upward support per leg: m_body·g / N_stance when foot reaction enabled.
 * @param actuator_stiffness Optional positive per-joint Nm/rad calibration. When
 * supplied, replaces the inertia proxy (including its stiffness_gain_scale).
 * Invalid explicit gains produce no correction for that joint; absent gains
 * retain the analytical fallback. Angle/torque/compensation-scale limits remain.
 */
LegGravityCompensation computeLegGravityCompensation(const LegGeometry& leg,
                                                     double q1,
                                                     double q2,
                                                     double q3,
                                                     const Vec3& g_down_body_unit,
                                                     double foot_reaction_n,
                                                     const control_config::GravityFeedforwardConfig& cfg,
                                                     const std::array<double, kJointsPerLeg>* actuator_stiffness = nullptr);

/** Adds joint-angle biases after IK. Reaction requires stance/contact;
 * self-weight also applies during swing. Existing feedback/IMU gates remain. */
void applyJointAngleGravityFeedforward(const control_config::GravityFeedforwardConfig& cfg,
                                       const HexapodGeometry& geometry,
                                       const RobotState& est,
                                       const GaitState& gait,
                                       JointTargets& in_out,
                                       const std::array<LegContactDecision, kNumLegs>* contact_modes = nullptr,
                                       double control_dt_s = 0.004);

/** Clears internal Δq low-pass state (call from ControlPipeline::reset). */
void resetJointAngleGravityFeedforwardState();
