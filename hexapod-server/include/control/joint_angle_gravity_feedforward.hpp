#pragma once

#include "control_config.hpp"
#include "locomotion_feasibility.hpp"
#include "types.hpp"

/** Per-leg gravity compensation (radians), before global max-delta clamps in apply(). */
struct LegGravityCompensation {
    double delta_coxa_rad{0.0};
    double delta_femur_rad{0.0};
    double delta_tibia_rad{0.0};
};

/**
 * Quasi-static gravity compensation from FK, equal-share stance reaction, optional link
 * self-weight, and PD stiffness (ωₙ² I) mapping. Pure for unit tests.
 *
 * @param g_down_body_unit  Unit gravity direction in **body** frame (same convention as IMU path in apply()).
 * @param foot_reaction_n   Magnitude of upward support per leg: m_body·g / N_stance when foot reaction enabled.
 */
LegGravityCompensation computeLegGravityCompensation(const LegGeometry& leg,
                                                     double q1,
                                                     double q2,
                                                     double q3,
                                                     const Vec3& g_down_body_unit,
                                                     double foot_reaction_n,
                                                     const control_config::GravityFeedforwardConfig& cfg);

/** Adds joint-angle biases after IK from IMU + stance/contact gates (see computeLegGravityCompensation). */
void applyJointAngleGravityFeedforward(const control_config::GravityFeedforwardConfig& cfg,
                                       const HexapodGeometry& geometry,
                                       const RobotState& est,
                                       const GaitState& gait,
                                       JointTargets& in_out,
                                       const std::array<LegContactDecision, kNumLegs>* contact_modes = nullptr);

/** Clears internal Δq low-pass state (call from ControlPipeline::reset). */
void resetJointAngleGravityFeedforwardState();
