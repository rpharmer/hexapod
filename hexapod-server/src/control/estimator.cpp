#include "estimator.hpp"

#include <chrono>
#include <cmath>

#include "geometry_config.hpp"
#include "math_types.hpp"
#include "software_imu.hpp"

namespace {

Vec3 computeFootInBodyFrame(const LegState& leg_state, const LegGeometry& leg_geometry) {
    const double q1 = leg_state.joint_state[COXA].pos_rad.value;
    const double q2 = leg_state.joint_state[FEMUR].pos_rad.value;
    const double q3 = leg_state.joint_state[TIBIA].pos_rad.value;

    const double rho = leg_geometry.femurLength.value * std::cos(q2) +
                       leg_geometry.tibiaLength.value * std::cos(q2 + q3);
    const double z_leg = leg_geometry.femurLength.value * std::sin(q2) +
                         leg_geometry.tibiaLength.value * std::sin(q2 + q3);
    const double r = leg_geometry.coxaLength.value + rho;

    const Vec3 foot_leg_local{r * std::cos(q1), r * std::sin(q1), z_leg};
    const Vec3 foot_body_relative = Mat3::rotZ(leg_geometry.mountAngle.value) * foot_leg_local;
    return leg_geometry.bodyCoxaOffset + foot_body_relative;
}

LegState unwrapLegStateNear(const LegState& reference, const LegState& candidate) {
    LegState out{};
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        const double ref = reference.joint_state[joint].pos_rad.value;
        const double raw = candidate.joint_state[joint].pos_rad.value;
        out.joint_state[joint].pos_rad = AngleRad{ref + shortestAngleDeltaRad(ref, raw)};
    }
    return out;
}

bool solveGroundPlane(const std::array<Vec3, kNumLegs>& points,
                      const std::array<bool, kNumLegs>& valid,
                      double& a,
                      double& b,
                      double& c) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_xx = 0.0;
    double sum_yy = 0.0;
    double sum_xy = 0.0;
    double sum_xz = 0.0;
    double sum_yz = 0.0;
    int n = 0;

    for (int i = 0; i < kNumLegs; ++i) {
        if (!valid[i]) {
            continue;
        }
        const Vec3& p = points[i];
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        sum_xx += p.x * p.x;
        sum_yy += p.y * p.y;
        sum_xy += p.x * p.y;
        sum_xz += p.x * p.z;
        sum_yz += p.y * p.z;
        ++n;
    }

    if (n < 3) {
        return false;
    }

    const double dn = static_cast<double>(n);

    // Least squares solve for z = a*x + b*y + c.
    const double m00 = sum_xx;
    const double m01 = sum_xy;
    const double m02 = sum_x;
    const double m10 = sum_xy;
    const double m11 = sum_yy;
    const double m12 = sum_y;
    const double m20 = sum_x;
    const double m21 = sum_y;
    const double m22 = dn;

    const double det = m00 * (m11 * m22 - m12 * m21) -
                       m01 * (m10 * m22 - m12 * m20) +
                       m02 * (m10 * m21 - m11 * m20);
    if (std::abs(det) < 1e-8) {
        return false;
    }

    const double r0 = sum_xz;
    const double r1 = sum_yz;
    const double r2 = sum_z;

    const double det_a = r0 * (m11 * m22 - m12 * m21) -
                         m01 * (r1 * m22 - m12 * r2) +
                         m02 * (r1 * m21 - m11 * r2);
    const double det_b = m00 * (r1 * m22 - m12 * r2) -
                         r0 * (m10 * m22 - m12 * m20) +
                         m02 * (m10 * r2 - r1 * m20);
    const double det_c = m00 * (m11 * r2 - r1 * m21) -
                         m01 * (m10 * r2 - r1 * m20) +
                         r0 * (m10 * m21 - m11 * m20);

    a = det_a / det;
    b = det_b / det;
    c = det_c / det;
    return true;
}

double normalizedServoSign(const double sign) {
    if (!std::isfinite(sign)) {
        return 1.0;
    }
    return sign >= 0.0 ? 1.0 : -1.0;
}

double servoAngularRateFromMechanicalJointRate(const double mechanical_joint_rad_per_s,
                                               const int joint_index,
                                               const ServoCalibration& cal) {
    const double s = joint_index == COXA ? normalizedServoSign(cal.coxaSign)
                                         : joint_index == FEMUR ? normalizedServoSign(cal.femurSign)
                                                                : normalizedServoSign(cal.tibiaSign);
    return s * mechanical_joint_rad_per_s;
}

} // namespace

void SimpleEstimator::configure(const control_config::FusionConfig& config) {
    fusion_.configure(config);
    const auto hold_window_us = std::chrono::duration_cast<std::chrono::microseconds>(config.contact_hold_window);
    contact_memory_window_us_ = static_cast<uint64_t>(std::max<int64_t>(0, hold_window_us.count()));
}

void SimpleEstimator::reset() {
    fusion_.reset();
    last_contact_points_body_m_.fill(Vec3{});
    last_contact_timestamps_.fill(TimePointUs{});
    last_continuous_leg_states_.fill(LegState{});
    have_last_continuous_leg_state_.fill(false);
    last_leg_timestamp_ = TimePointUs{};
    last_twist_timestamp_ = TimePointUs{};
    last_twist_pos_rad_ = Vec3{};
    last_body_trans_m_ = Vec3{};
}

RobotState SimpleEstimator::update(const RobotState& raw) {
    RobotState est{};
    est.has_body_twist_state = false;

    est.foot_contacts = raw.foot_contacts;
    est.imu = raw.imu;
    est.has_imu = raw.has_imu;
    est.matrix_lidar = raw.matrix_lidar;
    est.has_matrix_lidar = raw.has_matrix_lidar;
    est.sample_id = raw.sample_id;
    est.timestamp_us = raw.timestamp_us;
    est.body_twist_state.body_trans_mps = Vec3{};

    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();

    const bool have_dt = !last_leg_timestamp_.isZero() && raw.timestamp_us.value > last_leg_timestamp_.value;
    const double inv_dt_s = have_dt
                                ? 1e6 / static_cast<double>(raw.timestamp_us.value - last_leg_timestamp_.value)
                                : 0.0;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const ServoCalibration& cal = geometry.legGeometry[leg].servo;
        const LegState curr_joint = cal.toJointAngles(raw.leg_states[leg]);

        LegState continuous_joint = curr_joint;
        if (have_last_continuous_leg_state_[leg]) {
            continuous_joint = unwrapLegStateNear(last_continuous_leg_states_[leg], curr_joint);
        }

        const LegState continuous_servo = cal.toServoAngles(continuous_joint);
        est.leg_states[leg] = continuous_servo;

        if (have_dt && have_last_continuous_leg_state_[leg]) {
            const LegState& prev_continuous = last_continuous_leg_states_[leg];
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double prev = prev_continuous.joint_state[joint].pos_rad.value;
                const double curr = continuous_joint.joint_state[joint].pos_rad.value;
                const double delta_theta = shortestAngleDeltaRad(prev, curr);
                const double mechanical_vel = delta_theta * inv_dt_s;
                est.leg_states[leg].joint_state[joint].vel_radps =
                    AngularRateRadPerSec{
                        servoAngularRateFromMechanicalJointRate(mechanical_vel, joint, cal)};
            }
        }

        last_continuous_leg_states_[leg] = continuous_joint;
        have_last_continuous_leg_state_[leg] = true;
    }

    last_leg_timestamp_ = raw.timestamp_us;

    std::array<Vec3, kNumLegs> support_points_body_m{};
    std::array<bool, kNumLegs> support_point_valid{};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (raw.foot_contacts[leg]) {
            last_contact_points_body_m_[leg] =
                computeFootInBodyFrame(raw.leg_states[leg], geometry.legGeometry[leg]);
            last_contact_timestamps_[leg] = raw.timestamp_us;
        }

        const DurationUs age = raw.timestamp_us - last_contact_timestamps_[leg];
        if (!last_contact_timestamps_[leg].isZero() && age.value <= contact_memory_window_us_) {
            support_points_body_m[leg] = last_contact_points_body_m_[leg];
            support_point_valid[leg] = true;
        }
    }

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    if (solveGroundPlane(support_points_body_m, support_point_valid, a, b, c)) {
        est.body_twist_state.twist_pos_rad.x = std::atan2(-b, 1.0);
        est.body_twist_state.twist_pos_rad.y = std::atan2(a, 1.0);
        est.body_twist_state.twist_pos_rad.z = 0.0;

        est.body_twist_state.body_trans_m.x = 0.0;
        est.body_twist_state.body_trans_m.y = 0.0;
        est.body_twist_state.body_trans_m.z = -c;

        est.has_body_twist_state = true;

        if (!last_twist_timestamp_.isZero()) {
            const DurationUs dt_us = raw.timestamp_us - last_twist_timestamp_;
            if (dt_us.value > 0) {
                const double dt_s = static_cast<double>(dt_us.value) * 1e-6;
                const Vec3 twist_delta =
                    static_cast<Vec3>(est.body_twist_state.twist_pos_rad) - last_twist_pos_rad_;
                const Vec3 body_trans_delta =
                    static_cast<Vec3>(est.body_twist_state.body_trans_m) - last_body_trans_m_;
                est.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{twist_delta * (1.0 / dt_s)};
                est.body_twist_state.body_trans_mps = VelocityMps3{body_trans_delta * (1.0 / dt_s)};
            }
        }

        last_twist_timestamp_ = raw.timestamp_us;
        last_twist_pos_rad_ = est.body_twist_state.twist_pos_rad;
        last_body_trans_m_ = est.body_twist_state.body_trans_m;
    }

    RobotState fused = fusion_.update(est, state_fusion::FusionSourceMode::Measured);
    fillSoftwareImuIfNoHardware(raw, fused);
    return fused;
}
