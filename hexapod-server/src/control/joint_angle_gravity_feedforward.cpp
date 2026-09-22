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
                   double stiffness_nm_per_rad,
                   double scale,
                   double max_delta_rad) {
    if (!std::isfinite(stiffness_nm_per_rad) || stiffness_nm_per_rad <= 0.0) return 0.0;
    const double tau_c = clampTorqueNm(tau_nm);
    const double dq = scale * tau_c / stiffness_nm_per_rad;
    return std::clamp(dq, -max_delta_rad, max_delta_rad);
}

struct LegFfLpf {
    double femur_y{};
    double tibia_y{};
    bool femur_init{};
    bool tibia_init{};
};

std::array<LegFfLpf, kNumLegs> g_ff_lpf{};

double lpfDeltaRad(double raw, double tau_s, double dt_s, double& y, bool& init) {
    if (tau_s <= 0.0) {
        y = raw;
        init = true;
        return raw;
    }
    // Exact first-order update: constant inputs agree across control cadences.
    const double a = -std::expm1(-dt_s / tau_s);
    if (!init) {
        y = 0.0;
        init = true;
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

double normalizedServoSign(const double sign) {
    if (!std::isfinite(sign)) {
        return 1.0;
    }
    return sign >= 0.0 ? 1.0 : -1.0;
}

} // namespace

LegGravityCompensation computeLegGravityCompensation(const LegGeometry& leg,
                                                     const double q1,
                                                     const double q2,
                                                     const double q3,
                                                     const Vec3& g_down_body_unit,
                                                     const double foot_reaction_n,
                                                     const control_config::GravityFeedforwardConfig& cfg,
                                                     const std::array<double, kJointsPerLeg>* actuator_stiffness) {
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

    // Positive FK pitch rotates about -e_pitch. A load moment about e_pitch
    // is therefore the holding (opposing-load) generalized torque, not the
    // load torque itself. sagDeltaRad must not negate it a second time.
    const Vec3 e_pitch{-std::sin(q1), std::cos(q1), 0.0};

    const Mat3 r_leg = legFromBodyFrame(leg);
    const Vec3 g_leg = r_leg * g_down_body_unit;

    const Vec3 W_dir{
        g_leg.x * kStandardGravity,
        g_leg.y * kStandardGravity,
        g_leg.z * kStandardGravity,
    };

    const Vec3 C_femur{
        P_f.x + hexapod_dynamics::kFemurComFrac * (P_t.x - P_f.x),
        P_f.y + hexapod_dynamics::kFemurComFrac * (P_t.y - P_f.y),
        P_f.z + hexapod_dynamics::kFemurComFrac * (P_t.z - P_f.z),
    };
    const Vec3 C_tibia{
        P_t.x + hexapod_dynamics::kTibiaBodyComFrac * (P_F.x - P_t.x),
        P_t.y + hexapod_dynamics::kTibiaBodyComFrac * (P_F.y - P_t.y),
        P_t.z + hexapod_dynamics::kTibiaBodyComFrac * (P_F.z - P_t.z),
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
        const Vec3 W_femur{W_dir.x * hexapod_dynamics::kFemurMassKg, W_dir.y * hexapod_dynamics::kFemurMassKg,
                           W_dir.z * hexapod_dynamics::kFemurMassKg};
        constexpr double tibiaMass = hexapod_dynamics::kTibiaMassKg + hexapod_dynamics::kFootMassKg;
        const Vec3 W_tibia{W_dir.x * tibiaMass, W_dir.y * tibiaMass, W_dir.z * tibiaMass};

        tau_femur += torqueAboutAxis(P_f, e_pitch, C_femur, W_femur);
        tau_femur += torqueAboutAxis(P_f, e_pitch, C_tibia, W_tibia);

        tau_tibia += torqueAboutAxis(P_t, e_pitch, C_tibia, W_tibia);
    }

    // Inertia proxy: distal point masses about each joint axis (no foot reaction in I).
    const double I_femur =
        hexapod_dynamics::kFemurMassKg * distSqFromAxis(C_femur, P_f, e_pitch) +
        (hexapod_dynamics::kTibiaMassKg + hexapod_dynamics::kFootMassKg) * distSqFromAxis(C_tibia, P_f, e_pitch);

    const double I_tibia =
        (hexapod_dynamics::kTibiaMassKg + hexapod_dynamics::kFootMassKg) * distSqFromAxis(C_tibia, P_t, e_pitch);

    out.torque_femur_nm = tau_femur;
    out.torque_tibia_nm = tau_tibia;
    const double omegaSq = hexapod_dynamics::kServoOmegaNSq() * std::clamp(cfg.stiffness_gain_scale, .05, 20.0);
    out.stiffness_femur_nm_per_rad = omegaSq * std::max(I_femur, 1e-12);
    out.stiffness_tibia_nm_per_rad = omegaSq * std::max(I_tibia, 1e-12);
    if (actuator_stiffness != nullptr) {
        // This is an actuator contract, not another model estimate. Do not
        // multiply a reported gain by the analytical-proxy tuning scale.
        out.stiffness_femur_nm_per_rad = (*actuator_stiffness)[FEMUR];
        out.stiffness_tibia_nm_per_rad = (*actuator_stiffness)[TIBIA];
    }

    out.delta_femur_rad =
        sagDeltaRad(tau_femur, out.stiffness_femur_nm_per_rad, cfg.scale_femur, cfg.max_delta_femur_rad);
    out.delta_tibia_rad =
        sagDeltaRad(tau_tibia, out.stiffness_tibia_nm_per_rad, cfg.scale_tibia, cfg.max_delta_tibia_rad);
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
                                       const std::array<LegContactDecision, kNumLegs>* contact_modes,
                                       double control_dt_s) {
    if (!cfg.enabled || cfg.mode == control_config::GravityFeedforwardMode::Off) {
        g_ff_lpf.fill({});
        return;
    }
    if (cfg.scale_coxa == 0.0 && cfg.scale_femur == 0.0 && cfg.scale_tibia == 0.0) {
        g_ff_lpf.fill({});
        return;
    }
    if (!std::isfinite(control_dt_s) || control_dt_s <= 0.0) {
        g_ff_lpf.fill({});
        return;
    }
    const Vec3 accel{est.imu.accel_mps2.x, est.imu.accel_mps2.y, est.imu.accel_mps2.z};
    const double accel_norm = vecNorm(accel);
    const double gyro_norm = gyroMagnitude(est.imu);
    const bool imu_usable = est.has_imu && est.imu.valid
        && std::isfinite(gyro_norm) && gyro_norm <= cfg.max_gyro_radps
        && std::isfinite(accel_norm) && accel_norm >= 1e-6
        && (cfg.accel_norm_margin_mps2 <= 0.0
            || std::abs(accel_norm - kStandardGravity) <= cfg.accel_norm_margin_mps2);
    // Reject the measurement upstream of the filter. Its previous bounded
    // output decays towards zero, rather than disappearing and resuming stale.
    const Vec3 g_down = imu_usable
        ? Vec3{-accel.x / accel_norm, -accel.y / accel_norm, -accel.z / accel_norm}
        : Vec3{};

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
    const double foot_reaction_n =
        (cfg.include_foot_reaction && n_stance > 0 ? (hexapod_dynamics::kBodyMassKg * kStandardGravity /
                                      static_cast<double>(n_stance))
                                   : 0.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        const bool feedforward_stance =
            contact_modes != nullptr
                ? ((*contact_modes)[idx].use_stance_kinematics && (*contact_modes)[idx].raw_contact)
                : (gait.in_stance[idx] && est.foot_contacts[idx]);
        if ((cfg.mode == control_config::GravityFeedforwardMode::Bounded || cfg.include_self_weight) &&
            !gravityFeedforwardCanUseJointState(est.joint_state_quality[static_cast<std::size_t>(leg)])) {
            g_ff_lpf[static_cast<std::size_t>(leg)] = {};
            continue;
        }

        const ServoCalibration& calibration = geometry.legGeometry[leg].servo;
        // Self-weight depends on actual posture, not on a possibly far-away
        // desired angle. The Bounded quality guard above protects this sample.
        const LegState joint_target = calibration.toJointAngles(
            cfg.include_self_weight ? est.leg_states[leg] : in_out.leg_states[leg]);
        const double q1 = joint_target.joint_state[COXA].pos_rad.value;
        const double q2 = joint_target.joint_state[FEMUR].pos_rad.value;
        const double q3 = joint_target.joint_state[TIBIA].pos_rad.value;
        if (!std::isfinite(q1) || !std::isfinite(q2) || !std::isfinite(q3)) {
            g_ff_lpf[idx] = {};
            continue;
        }

        const LegGravityCompensation d = imu_usable && (feedforward_stance || cfg.include_self_weight)
            ? computeLegGravityCompensation(geometry.legGeometry[leg], q1, q2, q3, g_down,
                                          feedforward_stance ? foot_reaction_n : 0.0, cfg,
                                          est.joint_stiffness_valid[idx]
                                              ? &est.joint_stiffness_nm_per_rad[idx] : nullptr)
            : LegGravityCompensation{};

        LegFfLpf& s = g_ff_lpf[static_cast<std::size_t>(leg)];
        const double df = lpfDeltaRad(d.delta_femur_rad, cfg.delta_lpf_tau_s, control_dt_s, s.femur_y, s.femur_init);
        const double dt = lpfDeltaRad(d.delta_tibia_rad, cfg.delta_lpf_tau_s, control_dt_s, s.tibia_y, s.tibia_init);

        // Compensation is computed in mechanical joint space. Convert only its
        // delta into servo space so mirrored legs receive the same mechanical
        // correction while preserving the target's calibrated attachment offset.
        in_out.leg_states[leg].joint_state[COXA].pos_rad.value +=
            normalizedServoSign(calibration.coxaSign) * d.delta_coxa_rad;
        in_out.leg_states[leg].joint_state[FEMUR].pos_rad.value +=
            normalizedServoSign(calibration.femurSign) * df;
        in_out.leg_states[leg].joint_state[TIBIA].pos_rad.value +=
            normalizedServoSign(calibration.tibiaSign) * dt;
    }
}

void resetJointAngleGravityFeedforwardState() {
    g_ff_lpf.fill({});
}
