#include "estimator.hpp"

#include <cmath>

#include "geometry_config.hpp"
#include "leg_kinematics_utils.hpp"

namespace {

Vec3 computeFootInBodyFrame(const LegState& leg_state, const LegGeometry& leg_geometry) {
    return kinematics::footInBodyFrame(leg_state, leg_geometry);
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

    const double r0 = sum_xz;
    const double r1 = sum_yz;
    const double r2 = sum_z;
    return solve3x3Cramers(m00, m01, m02,
                           m10, m11, m12,
                           m20, m21, m22,
                           r0, r1, r2,
                           a, b, c);
}

} // namespace

RobotState SimpleEstimator::update(const RobotState& raw) {
    RobotState est{};

    est.foot_contacts = raw.foot_contacts;
    est.sample_id = raw.sample_id;
    est.timestamp_us = raw.timestamp_us;
    est.has_body_pose_state = raw.has_body_pose_state;
    est.body_pose_state.body_trans_mps = Vec3{};

    if (!last_leg_timestamp_.isZero()) {
        const DurationUs dt_us = raw.timestamp_us - last_leg_timestamp_;
        if (dt_us.value > 0) {
            const double inv_dt_s = 1e6 / static_cast<double>(dt_us.value);
            for (int leg = 0; leg < kNumLegs; ++leg) {
                for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                    const AngleRad current = raw.leg_states[leg].joint_state[joint].pos_rad;
                    const AngleRad previous = last_leg_states_[leg].joint_state[joint].pos_rad;
                    est.leg_states[leg].joint_state[joint].pos_rad = current;
                    est.leg_states[leg].joint_state[joint].vel_radps =
                        AngularRateRadPerSec{(current.value - previous.value) * inv_dt_s};
                }
            }
        }
    }

    if (last_leg_timestamp_.isZero() || raw.timestamp_us.value <= last_leg_timestamp_.value) {
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                est.leg_states[leg].joint_state[joint].pos_rad =
                    raw.leg_states[leg].joint_state[joint].pos_rad;
            }
        }
    }
    last_leg_states_ = raw.leg_states;
    last_leg_timestamp_ = raw.timestamp_us;

    if (raw.has_body_pose_state) {
        est.body_pose_state.orientation_rad = raw.body_pose_state.orientation_rad;
        est.body_pose_state.angular_velocity_radps = raw.body_pose_state.angular_velocity_radps;
        last_body_pose_timestamp_ = raw.timestamp_us;
        last_orientation_rad_ = est.body_pose_state.orientation_rad;
        return est;
    }

    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
    std::array<Vec3, kNumLegs> support_points_body_m{};
    std::array<bool, kNumLegs> support_point_valid{};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (raw.foot_contacts[leg]) {
            last_contact_points_body_m_[leg] =
                computeFootInBodyFrame(raw.leg_states[leg], geometry.legGeometry[leg]);
            last_contact_timestamps_[leg] = raw.timestamp_us;
        }

        const DurationUs age = raw.timestamp_us - last_contact_timestamps_[leg];
        if (!last_contact_timestamps_[leg].isZero() && age.value <= kContactMemoryWindowUs) {
            support_points_body_m[leg] = last_contact_points_body_m_[leg];
            support_point_valid[leg] = true;
        }
    }

    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    if (solveGroundPlane(support_points_body_m, support_point_valid, a, b, c)) {
        est.body_pose_state.orientation_rad.x = std::atan2(-b, 1.0);
        est.body_pose_state.orientation_rad.y = std::atan2(a, 1.0);
        est.body_pose_state.orientation_rad.z = 0.0;

        est.body_pose_state.body_trans_m.x = 0.0;
        est.body_pose_state.body_trans_m.y = 0.0;
        est.body_pose_state.body_trans_m.z = -c;

        if (!last_body_pose_timestamp_.isZero()) {
            const DurationUs dt_us = raw.timestamp_us - last_body_pose_timestamp_;
            if (dt_us.value > 0) {
                const double dt_s = static_cast<double>(dt_us.value) * 1e-6;
                const Vec3 orientation_delta =
                    static_cast<Vec3>(est.body_pose_state.orientation_rad) - last_orientation_rad_;
                const Vec3 body_trans_delta =
                    static_cast<Vec3>(est.body_pose_state.body_trans_m) - last_body_translation_m_;
                est.body_pose_state.angular_velocity_radps =
                    AngularVelocityRadPerSec3{orientation_delta * (1.0 / dt_s)};
                est.body_pose_state.body_trans_mps = VelocityMps3{body_trans_delta * (1.0 / dt_s)};
            }
        }

        last_body_pose_timestamp_ = raw.timestamp_us;
        last_orientation_rad_ = est.body_pose_state.orientation_rad;
        last_body_translation_m_ = est.body_pose_state.body_trans_m;
    }

    return est;
}
