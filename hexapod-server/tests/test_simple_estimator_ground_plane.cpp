#include "estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool allEstimatedValuesFinite(const RobotState& state) {
    for (const auto& leg : state.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value) || !std::isfinite(joint.vel_radps.value)) {
                return false;
            }
        }
    }

    const Vec3 body_velocity = state.body_pose_state.body_trans_mps;
    return std::isfinite(body_velocity.x) && std::isfinite(body_velocity.y) &&
           std::isfinite(body_velocity.z);
}

RobotState makeNeutralRaw(uint64_t sample_id, uint64_t timestamp_us) {
    RobotState raw{};
    raw.sample_id = sample_id;
    raw.timestamp_us = TimePointUs{timestamp_us};
    raw.foot_contacts = {true, true, true, true, true, true};

    for (auto& leg : raw.leg_states) {
        leg.joint_state[COXA].pos_rad = AngleRad{0.0};
        leg.joint_state[FEMUR].pos_rad = AngleRad{0.0};
        leg.joint_state[TIBIA].pos_rad = AngleRad{0.0};
    }

    return raw;
}

} // namespace

int main() {
    SimpleEstimator estimator{};

    const RobotState first = makeNeutralRaw(1, 1'000'000);
    const RobotState first_est = estimator.update(first);

    if (!expect(first_est.has_inferred_body_pose_state,
                "ground-plane estimate should mark inferred body pose as available") ||
        !expect(!first_est.has_measured_body_pose_state,
                "ground-plane-only estimate should not claim measured body pose") ||
        !expect(first_est.has_body_pose_state,
                "aggregate body pose availability should include inferred estimates") ||
        !expect(first_est.body_pose_state.body_trans_m.z > 0.0,
                "neutral stance should estimate body above the ground plane")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_recent = first;
    no_contact_recent.sample_id = 2;
    no_contact_recent.timestamp_us = TimePointUs{1'100'000};
    no_contact_recent.foot_contacts = {false, false, false, false, false, false};
    no_contact_recent.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.1};

    const RobotState recent_est = estimator.update(no_contact_recent);
    if (!expect(recent_est.body_pose_state.body_trans_m.z > 0.0,
                "recently touching feet should still support a ground estimate") ||
        !expect(std::abs(recent_est.leg_states[0].joint_state[COXA].vel_radps.value - 1.0) < 1e-6,
                "joint velocity should be estimated from position delta over dt")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_stale = no_contact_recent;
    no_contact_stale.sample_id = 3;
    no_contact_stale.timestamp_us = TimePointUs{1'500'001};

    const RobotState stale_est = estimator.update(no_contact_stale);
    if (!expect(std::abs(stale_est.body_pose_state.body_trans_m.z) < 1e-9,
                "stale no-contact window should clear ground estimate") ||
        !expect(!stale_est.has_inferred_body_pose_state,
                "without contact memory, inferred body pose should be unavailable") ||
        !expect(!stale_est.has_measured_body_pose_state,
                "without IMU sample, measured body pose should be unavailable") ||
        !expect(!stale_est.has_body_pose_state,
                "aggregate body pose should be unavailable when no source is present")) {
        return EXIT_FAILURE;
    }

    RobotState stance_motion_a = makeNeutralRaw(4, 2'000'000);
    stance_motion_a.foot_contacts = {true, false, false, false, false, false};
    const RobotState stance_motion_a_est = estimator.update(stance_motion_a);
    (void)stance_motion_a_est;

    RobotState stance_motion_b = stance_motion_a;
    stance_motion_b.sample_id = 5;
    stance_motion_b.timestamp_us = TimePointUs{2'050'000};
    stance_motion_b.foot_contacts = {true, false, false, false, false, false};
    stance_motion_b.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.05};

    const RobotState stance_motion_b_est = estimator.update(stance_motion_b);
    const double planar_speed =
        std::hypot(stance_motion_b_est.body_pose_state.body_trans_mps.x,
                   stance_motion_b_est.body_pose_state.body_trans_mps.y);
    if (!expect(planar_speed > 1e-4,
                "stance contact deltas should produce non-zero planar body velocity estimate")) {
        return EXIT_FAILURE;
    }

    // Feed non-increasing timestamps while joints continue changing to ensure
    // velocity estimation does not divide by zero or emit non-finite artifacts.
    RobotState non_increasing_a = makeNeutralRaw(6, 3'000'000);
    non_increasing_a.foot_contacts = {true, true, true, true, true, true};
    non_increasing_a.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.20};
    non_increasing_a.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.15};
    const RobotState non_increasing_a_est = estimator.update(non_increasing_a);

    RobotState equal_timestamp = non_increasing_a;
    equal_timestamp.sample_id = 7;
    equal_timestamp.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.23};
    equal_timestamp.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.10};
    const RobotState equal_timestamp_est = estimator.update(equal_timestamp);

    RobotState decreasing_timestamp = equal_timestamp;
    decreasing_timestamp.sample_id = 8;
    decreasing_timestamp.timestamp_us = TimePointUs{2'999'500};
    decreasing_timestamp.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.19};
    decreasing_timestamp.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.05};
    const RobotState decreasing_timestamp_est = estimator.update(decreasing_timestamp);

    if (!expect(std::abs(equal_timestamp_est.leg_states[0].joint_state[COXA].vel_radps.value) < 1e-12,
                "equal timestamps should not synthesize divide-by-zero joint velocity spikes") ||
        !expect(std::abs(decreasing_timestamp_est.leg_states[0].joint_state[COXA].vel_radps.value) < 1e-12,
                "decreasing timestamps should not synthesize divide-by-zero joint velocity spikes") ||
        !expect(allEstimatedValuesFinite(non_increasing_a_est) &&
                    allEstimatedValuesFinite(equal_timestamp_est) &&
                    allEstimatedValuesFinite(decreasing_timestamp_est),
                "non-increasing timestamp samples should keep estimator outputs finite")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
