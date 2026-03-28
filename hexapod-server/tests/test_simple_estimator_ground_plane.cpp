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

    RobotState no_contact_at_window = no_contact_recent;
    no_contact_at_window.sample_id = 3;
    no_contact_at_window.timestamp_us = TimePointUs{1'250'000};

    const RobotState at_window_est = estimator.update(no_contact_at_window);
    if (!expect(at_window_est.body_pose_state.body_trans_m.z > 0.0,
                "contact memory should preserve support points exactly at expiry boundary") ||
        !expect(at_window_est.has_inferred_body_pose_state,
                "inferred pose should remain available at contact memory boundary") ||
        !expect(at_window_est.has_body_pose_state,
                "aggregate body pose should remain available at contact memory boundary")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_just_over_window = no_contact_at_window;
    no_contact_just_over_window.sample_id = 4;
    no_contact_just_over_window.timestamp_us = TimePointUs{1'250'001};

    const RobotState just_over_window_est = estimator.update(no_contact_just_over_window);
    if (!expect(std::abs(just_over_window_est.body_pose_state.body_trans_m.z) < 1e-9,
                "support points should expire immediately after contact memory boundary") ||
        !expect(!just_over_window_est.has_inferred_body_pose_state,
                "expired support points should drop inferred body pose availability") ||
        !expect(!just_over_window_est.has_measured_body_pose_state,
                "without IMU sample, measured body pose should be unavailable") ||
        !expect(!just_over_window_est.has_body_pose_state,
                "aggregate body pose should degrade predictably after support expiry")) {
        return EXIT_FAILURE;
    }

    RobotState stance_motion_a = makeNeutralRaw(5, 2'000'000);
    stance_motion_a.foot_contacts = {true, false, false, false, false, false};
    const RobotState stance_motion_a_est = estimator.update(stance_motion_a);
    (void)stance_motion_a_est;

    RobotState stance_motion_b = stance_motion_a;
    stance_motion_b.sample_id = 6;
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

    return EXIT_SUCCESS;
}
