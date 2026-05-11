#include "joint_angle_gravity_feedforward.hpp"

#include "hexapod_dynamics_constants.hpp"
#include "math_types.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kStandardGravity = hexapod_dynamics::kStandardGravityMps2;

double gyroMagnitude(const ImuSample& imu) {
    return std::sqrt(imu.gyro_radps.x * imu.gyro_radps.x + imu.gyro_radps.y * imu.gyro_radps.y +
                     imu.gyro_radps.z * imu.gyro_radps.z);
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/** Squared perpendicular distance from point `p` to axis through `axis_origin` with unit direction `axis_u`. */
double distSqFromAxis(const Vec3& p, const Vec3& axis_origin, const Vec3& axis_u) {
    const Vec3 r{p.x - axis_origin.x, p.y - axis_origin.y, p.z - axis_origin.z};
    const double along = dot(r, axis_u);
    const Vec3 perp{r.x - axis_u.x * along, r.y - axis_u.y * along, r.z - axis_u.z * along};
    return dot(perp, perp);
}

double torqueAboutAxis(const Vec3& axis_origin, const Vec3& axis_u, const Vec3& r_from, const Vec3& force) {
    const Vec3 r{r_from.x - axis_origin.x, r_from.y - axis_origin.y, r_from.z - axis_origin.z};
    const Vec3 t = cross(r, force);
    return dot(t, axis_u);
}

/** Clamp torque magnitude for linear sag model (actuator saturation). */
double clampTorqueNm(double tau) {
    return std::clamp(tau, -hexapod_dynamics::kServoMaxTorqueNm, hexapod_dynamics::kServoMaxTorqueNm);
}

double sagDeltaRad(double tau_nm,
                   double inertia_kg_m2,
                   double scale,
                   double max_delta_rad,
                   double stiffness_gain_scale) {
    const double gscale = std::clamp(stiffness_gain_scale, 0.05, 20.0);
    const double omega_sq = hexapod_dynamics::kServoOmegaNSq() * gscale;
    const double I = std::max(inertia_kg_m2, 1.0e-12);
    const double tau_c = clampTorqueNm(tau_nm);
    const double dq = -scale * tau_c / (omega_sq * I);
    return std::clamp(dq, -max_delta_rad, max_delta_rad);
}

struct LegFfLpf {
    double femur_y{};
    double tibia_y{};
    bool femur_init{};
    bool tibia_init{};
};

std::array<LegFfLpf, kNumLegs> g_ff_lpf{};

double lpfDeltaRad(double raw, double tau_s, double& y, bool& init) {
    if (tau_s <= 0.0) {
        return raw;
    }
    constexpr double kDt = 0.004;
    const double a = kDt / (tau_s + kDt);
    if (!init) {
        y = raw;
        init = true;
        return y;
    }
    y += a * (raw - y);
    return y;
}

bool gravityFeedforwardCanUseJointState(const JointStateQuality& quality) {
    if (!quality.position_valid) {
        return false;
    }
    return quality.source == JointStateSource::Measured ||
           quality.source == JointStateSource::Simulated ||
           (quality.source == JointStateSource::ObserverEstimate && quality.confidence >= 0.45);
}

} // namespace

LegGravityCompensation computeLegGravityCompensation(const LegGeometry& leg,
                                                     const double q1,
                                                     const double q2,
                                                     const double q3,
                                                     const Vec3& g_down_body_unit,
                                                     const double foot_reaction_n,
                                                     const control_config::GravityFeedforwardConfig& cfg) {
    LegGravityCompensation out{};
    const double L1 = leg.coxaLength.value;
    const double L2 = leg.femurLength.value;
    const double L3 = leg.tibiaLength.value;

    // Leg frame: same convention as leg_fk.cpp (foot at r·(cos q1, sin q1, 0) + z·ẑ).
    const double rho_mid = L2 * std::cos(q2);
    const double z_mid = L2 * std::sin(q2);
    const double r_foot = L1 + L2 * std::cos(q2) + L3 * std::cos(q2 + q3);
    const double z_foot = L2 * std::sin(q2) + L3 * std::sin(q2 + q3);

    const Vec3 P_f{L1 * std::cos(q1), L1 * std::sin(q1), 0.0};
    const Vec3 P_t{(L1 + rho_mid) * std::cos(q1), (L1 + rho_mid) * std::sin(q1), z_mid};
    const Vec3 P_F{r_foot * std::cos(q1), r_foot * std::sin(q1), z_foot};

    const Vec3 e_pitch{-std::sin(q1), std::cos(q1), 0.0};

    const Mat3 r_leg = Mat3::rotZ(-leg.mountAngle.value);
    const Vec3 g_leg = r_leg * g_down_body_unit;

    const Vec3 W_dir{
        g_leg.x * kStandardGravity,
        g_leg.y * kStandardGravity,
        g_leg.z * kStandardGravity,
    };

    const Vec3 C_coxa{
        hexapod_dynamics::kCoxaComFrac * P_f.x,
        hexapod_dynamics::kCoxaComFrac * P_f.y,
        hexapod_dynamics::kCoxaComFrac * P_f.z,
    };
    const Vec3 C_femur{
        P_f.x + hexapod_dynamics::kFemurComFrac * (P_t.x - P_f.x),
        P_f.y + hexapod_dynamics::kFemurComFrac * (P_t.y - P_f.y),
        P_f.z + hexapod_dynamics::kFemurComFrac * (P_t.z - P_f.z),
    };
    const Vec3 C_tibia{
        P_t.x + hexapod_dynamics::kTibiaPlusFootComFrac * (P_F.x - P_t.x),
        P_t.y + hexapod_dynamics::kTibiaPlusFootComFrac * (P_F.y - P_t.y),
        P_t.z + hexapod_dynamics::kTibiaPlusFootComFrac * (P_F.z - P_t.z),
    };

    // Upward support in body frame is −ĝ_down; map to leg frame for moment arms at the foot.
    const Vec3 F_body_support{
        -g_down_body_unit.x * foot_reaction_n,
        -g_down_body_unit.y * foot_reaction_n,
        -g_down_body_unit.z * foot_reaction_n,
    };
    const Vec3 F_foot_leg = r_leg * F_body_support;

    double tau_femur = 0.0;
    double tau_tibia = 0.0;

    if (cfg.include_foot_reaction && foot_reaction_n > 0.0) {
        tau_femur += torqueAboutAxis(P_f, e_pitch, P_F, F_foot_leg);
        tau_tibia += torqueAboutAxis(P_t, e_pitch, P_F, F_foot_leg);
    }

    if (cfg.include_self_weight) {
        const Vec3 W_coxa{W_dir.x * hexapod_dynamics::kCoxaMassKg, W_dir.y * hexapod_dynamics::kCoxaMassKg,
                          W_dir.z * hexapod_dynamics::kCoxaMassKg};
        const Vec3 W_femur{W_dir.x * hexapod_dynamics::kFemurMassKg, W_dir.y * hexapod_dynamics::kFemurMassKg,
                           W_dir.z * hexapod_dynamics::kFemurMassKg};
        const Vec3 W_tibia{W_dir.x * hexapod_dynamics::kTibiaMassKg, W_dir.y * hexapod_dynamics::kTibiaMassKg,
                           W_dir.z * hexapod_dynamics::kTibiaMassKg};
        const Vec3 W_foot{W_dir.x * hexapod_dynamics::kFootMassKg, W_dir.y * hexapod_dynamics::kFootMassKg,
                          W_dir.z * hexapod_dynamics::kFootMassKg};

        tau_femur += torqueAboutAxis(P_f, e_pitch, C_coxa, W_coxa);
        tau_femur += torqueAboutAxis(P_f, e_pitch, C_femur, W_femur);
        tau_femur += torqueAboutAxis(P_f, e_pitch, C_tibia, W_tibia);
        tau_femur += torqueAboutAxis(P_f, e_pitch, P_F, W_foot);

        tau_tibia += torqueAboutAxis(P_t, e_pitch, C_tibia, W_tibia);
        tau_tibia += torqueAboutAxis(P_t, e_pitch, P_F, W_foot);
    }

    // Inertia proxy: distal point masses about each joint axis (no foot reaction in I).
    const double I_femur =
        hexapod_dynamics::kFemurMassKg * distSqFromAxis(C_femur, P_f, e_pitch) +
        hexapod_dynamics::kTibiaMassKg * distSqFromAxis(C_tibia, P_f, e_pitch) +
        hexapod_dynamics::kFootMassKg * distSqFromAxis(P_F, P_f, e_pitch);

    const double I_tibia =
        hexapod_dynamics::kTibiaMassKg * distSqFromAxis(C_tibia, P_t, e_pitch) +
        hexapod_dynamics::kFootMassKg * distSqFromAxis(P_F, P_t, e_pitch);

    out.delta_femur_rad =
        sagDeltaRad(tau_femur, I_femur, cfg.scale_femur, cfg.max_delta_femur_rad, cfg.stiffness_gain_scale);
    out.delta_tibia_rad =
        sagDeltaRad(tau_tibia, I_tibia, cfg.scale_tibia, cfg.max_delta_tibia_rad, cfg.stiffness_gain_scale);
    // Coxa yaw sag is not modeled in this quasi-static pitch-only path; keep Δq_coxa = 0.
    out.delta_coxa_rad = 0.0;
    (void)cfg.scale_coxa;
    return out;
}

void applyJointAngleGravityFeedforward(const control_config::GravityFeedforwardConfig& cfg,
                                       const HexapodGeometry& geometry,
                                       const RobotState& est,
                                       const GaitState& gait,
                                       JointTargets& in_out,
                                       const std::array<LegContactDecision, kNumLegs>* contact_modes) {
    if (!cfg.enabled || cfg.mode == control_config::GravityFeedforwardMode::Off) {
        g_ff_lpf.fill({});
        return;
    }
    if (cfg.scale_coxa == 0.0 && cfg.scale_femur == 0.0 && cfg.scale_tibia == 0.0) {
        g_ff_lpf.fill({});
        return;
    }
    if (!est.has_imu || !est.imu.valid) {
        return;
    }
    if (gyroMagnitude(est.imu) > cfg.max_gyro_radps) {
        return;
    }

    const Vec3 accel{est.imu.accel_mps2.x, est.imu.accel_mps2.y, est.imu.accel_mps2.z};
    const double accel_norm = vecNorm(accel);
    if (accel_norm < 1e-6) {
        return;
    }
    if (cfg.accel_norm_margin_mps2 > 0.0 &&
        std::abs(accel_norm - kStandardGravity) > cfg.accel_norm_margin_mps2) {
        return;
    }

    const Vec3 up{accel.x / accel_norm, accel.y / accel_norm, accel.z / accel_norm};
    const Vec3 g_down{-up.x, -up.y, -up.z};

    int n_stance = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        const bool feedforward_stance =
            contact_modes != nullptr
                ? ((*contact_modes)[idx].use_stance_kinematics && (*contact_modes)[idx].raw_contact)
                : (gait.in_stance[idx] && est.foot_contacts[idx]);
        if (feedforward_stance) {
            ++n_stance;
        }
    }
    if (n_stance <= 0) {
        return;
    }

    const double foot_reaction_n =
        (cfg.include_foot_reaction ? (hexapod_dynamics::kBodyMassKg * kStandardGravity /
                                      static_cast<double>(n_stance))
                                   : 0.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        const bool feedforward_stance =
            contact_modes != nullptr
                ? ((*contact_modes)[idx].use_stance_kinematics && (*contact_modes)[idx].raw_contact)
                : (gait.in_stance[idx] && est.foot_contacts[idx]);
        if (!feedforward_stance) {
            g_ff_lpf[static_cast<std::size_t>(leg)] = {};
        }
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        const bool feedforward_stance =
            contact_modes != nullptr
                ? ((*contact_modes)[idx].use_stance_kinematics && (*contact_modes)[idx].raw_contact)
                : (gait.in_stance[idx] && est.foot_contacts[idx]);
        if (!feedforward_stance) {
            continue;
        }
        if (cfg.mode == control_config::GravityFeedforwardMode::Bounded &&
            !gravityFeedforwardCanUseJointState(est.joint_state_quality[static_cast<std::size_t>(leg)])) {
            g_ff_lpf[static_cast<std::size_t>(leg)] = {};
            continue;
        }

        const double q1 = in_out.leg_states[leg].joint_state[COXA].pos_rad.value;
        const double q2 = in_out.leg_states[leg].joint_state[FEMUR].pos_rad.value;
        const double q3 = in_out.leg_states[leg].joint_state[TIBIA].pos_rad.value;

        const LegGravityCompensation d =
            computeLegGravityCompensation(geometry.legGeometry[leg], q1, q2, q3, g_down, foot_reaction_n, cfg);

        LegFfLpf& s = g_ff_lpf[static_cast<std::size_t>(leg)];
        const double df = lpfDeltaRad(d.delta_femur_rad, cfg.delta_lpf_tau_s, s.femur_y, s.femur_init);
        const double dt = lpfDeltaRad(d.delta_tibia_rad, cfg.delta_lpf_tau_s, s.tibia_y, s.tibia_init);

        in_out.leg_states[leg].joint_state[COXA].pos_rad.value += d.delta_coxa_rad;
        in_out.leg_states[leg].joint_state[FEMUR].pos_rad.value += df;
        in_out.leg_states[leg].joint_state[TIBIA].pos_rad.value += dt;
    }
}

void resetJointAngleGravityFeedforwardState() {
    g_ff_lpf.fill({});
}
