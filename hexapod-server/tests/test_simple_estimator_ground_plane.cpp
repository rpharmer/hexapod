#include "estimator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool almostEqual(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) < eps;
}
bool allFiniteBodyPose(const BodyPoseKinematics& pose) {
    return std::isfinite(pose.orientation_rad.x) &&
           std::isfinite(pose.orientation_rad.y) &&
           std::isfinite(pose.orientation_rad.z) &&
           std::isfinite(pose.angular_velocity_radps.x) &&
           std::isfinite(pose.angular_velocity_radps.y) &&
           std::isfinite(pose.angular_velocity_radps.z) &&
           std::isfinite(pose.body_trans_m.x) &&
           std::isfinite(pose.body_trans_m.y) &&
           std::isfinite(pose.body_trans_m.z) &&
           std::isfinite(pose.body_trans_mps.x) &&
           std::isfinite(pose.body_trans_mps.y) &&
           std::isfinite(pose.body_trans_mps.z);
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
double planarMagnitude(const Vec3& v) {
    return std::hypot(v.x, v.y);
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
    // Subcase 1: IMU-present raw pose => availability true (orientation source: measured/IMU).
    SimpleEstimator imu_estimator{};
    RobotState imu_raw = makeNeutralRaw(1, 1'000'000);
    imu_raw.has_measured_body_pose_state = true;
    imu_raw.has_body_pose_state = true;
    imu_raw.body_pose_state.orientation_rad = EulerAnglesRad3{0.11, -0.07, 0.42};
    const RobotState imu_est = imu_estimator.update(imu_raw);
    if (!expect(imu_est.has_body_pose_state,
                "IMU-present update should report aggregate body pose availability") ||
        !expect(imu_est.has_measured_body_pose_state,
                "IMU-present update should report measured body pose availability") ||
        !expect(!imu_est.has_inferred_body_pose_state,
                "IMU-present update should keep orientation source as measured, not inferred") ||
        !expect(almostEqual(imu_est.body_pose_state.orientation_rad.x, imu_raw.body_pose_state.orientation_rad.x),
                "IMU-present update should preserve IMU roll") ||
        !expect(almostEqual(imu_est.body_pose_state.orientation_rad.y, imu_raw.body_pose_state.orientation_rad.y),
                "IMU-present update should preserve IMU pitch") ||
        !expect(almostEqual(imu_est.body_pose_state.orientation_rad.z, imu_raw.body_pose_state.orientation_rad.z),
                "IMU-present update should preserve IMU yaw")) {
        return EXIT_FAILURE;
    }

    // Subcase 2: IMU-absent but solvable ground plane => availability true (orientation source: inferred).
    SimpleEstimator inferred_estimator{};
    const RobotState first = makeNeutralRaw(1, 1'000'000);
    const RobotState first_est = inferred_estimator.update(first);

    if (!expect(first_est.has_inferred_body_pose_state,
                "ground-plane estimate should mark inferred body pose as available") ||
        !expect(!first_est.has_measured_body_pose_state,
                "ground-plane-only estimate should not claim measured body pose") ||
        !expect(first_est.has_body_pose_state,
                "aggregate body pose availability should include inferred estimates") ||
        !expect(almostEqual(first_est.body_pose_state.orientation_rad.z, 0.0),
                "inferred orientation source should set yaw to zero") ||
        !expect(first_est.body_pose_state.body_trans_m.z > 0.0,
                "neutral stance should estimate body above the ground plane")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_recent = first;
    no_contact_recent.sample_id = 2;
    no_contact_recent.timestamp_us = TimePointUs{1'100'000};
    no_contact_recent.foot_contacts = {false, false, false, false, false, false};
    no_contact_recent.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.1};

    const RobotState recent_est = inferred_estimator.update(no_contact_recent);
    if (!expect(recent_est.body_pose_state.body_trans_m.z > 0.0,
                "recently touching feet should still support a ground estimate") ||
        !expect(recent_est.has_inferred_body_pose_state && !recent_est.has_measured_body_pose_state,
                "without IMU input, orientation source should remain inferred while memory is valid") ||
        !expect(std::abs(recent_est.leg_states[0].joint_state[COXA].vel_radps.value - 1.0) < 1e-6,
                "joint velocity should be estimated from position delta over dt")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_at_window = no_contact_recent;
    no_contact_at_window.sample_id = 3;
    no_contact_at_window.timestamp_us = TimePointUs{1'250'000};

    RobotState no_contact_stale = no_contact_at_window;
    no_contact_stale.sample_id = 4;
    no_contact_stale.timestamp_us = TimePointUs{1'250'001};

    const RobotState stale_est = inferred_estimator.update(no_contact_stale);
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

    // Subcase 3: IMU-absent and unsolved ground plane => availability false (orientation source: none).
    SimpleEstimator unsolved_estimator{};
    RobotState unsolved_raw = makeNeutralRaw(1, 1'000'000);
    unsolved_raw.foot_contacts = {false, false, false, false, false, false};
    const RobotState unsolved_est = unsolved_estimator.update(unsolved_raw);
    if (!expect(!unsolved_est.has_measured_body_pose_state,
                "unsolved case should not have measured orientation source") ||
        !expect(!unsolved_est.has_inferred_body_pose_state,
                "unsolved case should not have inferred orientation source") ||
        !expect(!unsolved_est.has_body_pose_state,
                "unsolved case should report body pose unavailable")) {
        return EXIT_FAILURE;
    }

    const RobotState at_window_est = inferred_estimator.update(no_contact_at_window);
    if (!expect(at_window_est.body_pose_state.body_trans_m.z > 0.0,
                "contact memory should preserve support points exactly at expiry boundary") ||
        !expect(at_window_est.has_inferred_body_pose_state,
                "inferred pose should remain available at contact memory boundary") ||
        !expect(at_window_est.has_body_pose_state,
                "aggregate body pose should remain available at contact memory boundary")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_just_over_window = no_contact_at_window;
    no_contact_just_over_window.sample_id = 5;
    no_contact_just_over_window.timestamp_us = TimePointUs{1'250'001};

    const RobotState just_over_window_est = inferred_estimator.update(no_contact_just_over_window);
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
    const RobotState stance_motion_a_est = inferred_estimator.update(stance_motion_a);
    (void)stance_motion_a_est;

    RobotState stance_motion_b = stance_motion_a;
    stance_motion_b.sample_id = 6;
    stance_motion_b.timestamp_us = TimePointUs{2'050'000};
    stance_motion_b.foot_contacts = {true, false, false, false, false, false};
    stance_motion_b.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.05};

    const RobotState stance_motion_b_est = inferred_estimator.update(stance_motion_b);
    const double planar_speed =
        std::hypot(stance_motion_b_est.body_pose_state.body_trans_mps.x,
                   stance_motion_b_est.body_pose_state.body_trans_mps.y);
    if (!expect(planar_speed > 1e-4,
                "stance contact deltas should produce non-zero planar body velocity estimate")) {
        return EXIT_FAILURE;
    }

    RobotState non_finite_imu = makeNeutralRaw(6, 2'100'000);
    non_finite_imu.has_measured_body_pose_state = true;
    non_finite_imu.body_pose_state.orientation_rad = Vec3{0.01, -0.01, 0.0};
    non_finite_imu.body_pose_state.angular_velocity_radps = Vec3{
        std::numeric_limits<double>::quiet_NaN(), 0.2, 0.3};
    non_finite_imu.body_pose_state.body_trans_mps = Vec3{
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    const RobotState non_finite_imu_est = inferred_estimator.update(non_finite_imu);
    if (!expect(allFiniteBodyPose(non_finite_imu_est.body_pose_state),
                "non-finite measured body velocity inputs should not produce non-finite estimator body pose outputs")) {
        return EXIT_FAILURE;
    }

    RobotState non_finite_joint = makeNeutralRaw(7, 2'120'000);
    non_finite_joint.leg_states[0].joint_state[FEMUR].pos_rad =
        AngleRad{std::numeric_limits<double>::infinity()};
    const RobotState non_finite_joint_est = inferred_estimator.update(non_finite_joint);
    if (!expect(std::isfinite(non_finite_joint_est.leg_states[0].joint_state[FEMUR].vel_radps.value),
                "non-finite joint telemetry should be safely rejected for velocity estimation")) {
        return EXIT_FAILURE;
    }

    // Feed non-increasing timestamps while joints continue changing to ensure
    // velocity estimation does not divide by zero or emit non-finite artifacts.
    RobotState non_increasing_a = makeNeutralRaw(6, 3'000'000);
    non_increasing_a.foot_contacts = {true, true, true, true, true, true};
    non_increasing_a.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.20};
    non_increasing_a.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.15};
    const RobotState non_increasing_a_est = inferred_estimator.update(non_increasing_a);

    RobotState equal_timestamp = non_increasing_a;
    equal_timestamp.sample_id = 7;
    equal_timestamp.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.23};
    equal_timestamp.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.10};
    const RobotState equal_timestamp_est = inferred_estimator.update(equal_timestamp);

    RobotState decreasing_timestamp = equal_timestamp;
    decreasing_timestamp.sample_id = 8;
    decreasing_timestamp.timestamp_us = TimePointUs{2'999'500};
    decreasing_timestamp.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.19};
    decreasing_timestamp.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{-0.05};
    const RobotState decreasing_timestamp_est = inferred_estimator.update(decreasing_timestamp);

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

    // Subcase 4: alternating contact around memory window should keep support only within window,
    // and reacquisition after expiry should avoid pose discontinuity under slight joint drift.
    SimpleEstimator alternating_contact_estimator{};
    RobotState alternating_contact_true = makeNeutralRaw(10, 3'000'000);
    const RobotState alternating_contact_true_est =
        alternating_contact_estimator.update(alternating_contact_true);
    if (!expect(alternating_contact_true_est.has_inferred_body_pose_state,
                "initial contact sample should seed inferred support points")) {
        return EXIT_FAILURE;
    }

    RobotState alternating_contact_false_within = alternating_contact_true;
    alternating_contact_false_within.sample_id = 11;
    alternating_contact_false_within.timestamp_us = TimePointUs{3'249'999}; // 249,999us age < 250,000us window.
    alternating_contact_false_within.foot_contacts = {false, false, false, false, false, false};
    alternating_contact_false_within.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.015};
    const RobotState alternating_contact_false_within_est =
        alternating_contact_estimator.update(alternating_contact_false_within);
    if (!expect(alternating_contact_false_within_est.has_inferred_body_pose_state,
                "support points should persist while no-contact age remains within memory window") ||
        !expect(std::abs(alternating_contact_false_within_est.body_pose_state.body_trans_m.z -
                         alternating_contact_true_est.body_pose_state.body_trans_m.z) < 1e-9,
                "joint drift without contact should not perturb support-point-derived body height")) {
        return EXIT_FAILURE;
    }

    RobotState alternating_contact_false_stale = alternating_contact_false_within;
    alternating_contact_false_stale.sample_id = 12;
    alternating_contact_false_stale.timestamp_us = TimePointUs{3'250'001}; // 250,001us age > 250,000us window.
    const RobotState alternating_contact_false_stale_est =
        alternating_contact_estimator.update(alternating_contact_false_stale);
    if (!expect(!alternating_contact_false_stale_est.has_inferred_body_pose_state,
                "support points should expire once no-contact age exceeds memory window")) {
        return EXIT_FAILURE;
    }

    RobotState alternating_contact_reacquired = alternating_contact_false_stale;
    alternating_contact_reacquired.sample_id = 13;
    alternating_contact_reacquired.timestamp_us = TimePointUs{3'300'000};
    alternating_contact_reacquired.foot_contacts = {true, true, true, true, true, true};
    alternating_contact_reacquired.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.02};
    const RobotState alternating_contact_reacquired_est =
        alternating_contact_estimator.update(alternating_contact_reacquired);
    if (!expect(alternating_contact_reacquired_est.has_inferred_body_pose_state,
                "contact reacquisition should restore inferred pose availability") ||
        !expect(std::abs(alternating_contact_reacquired_est.body_pose_state.body_trans_m.z -
                         alternating_contact_false_within_est.body_pose_state.body_trans_m.z) < 0.02,
                "contact reacquisition with slight drift should not introduce body-height discontinuity") ||
        !expect(std::abs(alternating_contact_reacquired_est.body_pose_state.orientation_rad.x -
                         alternating_contact_false_within_est.body_pose_state.orientation_rad.x) < 0.05,
                "contact reacquisition with slight drift should keep roll continuity") ||
        !expect(std::abs(alternating_contact_reacquired_est.body_pose_state.orientation_rad.y -
                         alternating_contact_false_within_est.body_pose_state.orientation_rad.y) < 0.05,
                "contact reacquisition with slight drift should keep pitch continuity")) {
        return EXIT_FAILURE;
    }

    // Subcase 5: sustained realistic topology transitions (tripod alternation with transient slip
    // and re-contact) should keep estimated orientation/velocity finite without unbounded derivative
    // artifacts over many cycles.
    SimpleEstimator topology_transition_estimator{};
    constexpr uint64_t kDtUs = 20'000; // 50 Hz estimator cadence.
    constexpr int kCycles = 150;

    auto setTripodContacts = [](RobotState& state, bool tripod_a) {
        // Alternating tripods: A={LF, LM, RR}, B={RF, RM, LR}.
        // Leg indices: 0..5 in fixed hardware order.
        state.foot_contacts = {tripod_a, !tripod_a, !tripod_a, tripod_a, !tripod_a, tripod_a};
    };

    RobotState topology_raw = makeNeutralRaw(100, 4'000'000);
    setTripodContacts(topology_raw, true);
    RobotState prev_est = topology_transition_estimator.update(topology_raw);
    if (!expect(prev_est.has_inferred_body_pose_state,
                "initial tripod contact should produce inferred body pose")) {
        return EXIT_FAILURE;
    }

    double max_roll_rate_radps = 0.0;
    double max_pitch_rate_radps = 0.0;
    double max_planar_accel_mps2 = 0.0;
    double max_roll_jerk_radps2 = 0.0;
    double max_pitch_jerk_radps2 = 0.0;
    double max_planar_jerk_mps3 = 0.0;
    bool has_prev_derivative = false;
    double prev_roll_rate_radps = 0.0;
    double prev_pitch_rate_radps = 0.0;
    double prev_planar_accel_mps2 = 0.0;

    for (int cycle = 1; cycle <= kCycles; ++cycle) {
        const bool tripod_a = (cycle % 2) == 0;
        topology_raw.sample_id += 1;
        topology_raw.timestamp_us = TimePointUs{topology_raw.timestamp_us.value + kDtUs};
        setTripodContacts(topology_raw, tripod_a);

        // Inject transient slip for one support leg every 6th cycle then re-contact next cycle.
        if ((cycle % 6) == 0) {
            topology_raw.foot_contacts[tripod_a ? 0 : 1] = false;
        }

        // Modulate joints smoothly to emulate gait progression and load transfer.
        for (std::size_t leg_idx = 0; leg_idx < topology_raw.leg_states.size(); ++leg_idx) {
            const double phase = (static_cast<double>(cycle) * 0.22) + static_cast<double>(leg_idx) * 0.31;
            topology_raw.leg_states[leg_idx].joint_state[COXA].pos_rad = AngleRad{0.06 * std::sin(phase)};
            topology_raw.leg_states[leg_idx].joint_state[FEMUR].pos_rad =
                AngleRad{-0.28 + 0.07 * std::cos(phase * 0.9)};
            topology_raw.leg_states[leg_idx].joint_state[TIBIA].pos_rad =
                AngleRad{0.52 + 0.08 * std::sin(phase * 1.1)};
        }

        const RobotState est = topology_transition_estimator.update(topology_raw);
        if (!expect(est.has_inferred_body_pose_state,
                    "tripod alternation with short slip should preserve inferred support estimate") ||
            !expect(allEstimatedValuesFinite(est),
                    "topology transition sequence should keep estimates finite")) {
            return EXIT_FAILURE;
        }

        const double dt_s =
            static_cast<double>(topology_raw.timestamp_us.value - prev_est.timestamp_us.value) * 1e-6;
        const double roll_rate_radps = (est.body_pose_state.orientation_rad.x -
                                        prev_est.body_pose_state.orientation_rad.x) / dt_s;
        const double pitch_rate_radps = (est.body_pose_state.orientation_rad.y -
                                         prev_est.body_pose_state.orientation_rad.y) / dt_s;
        const double planar_accel_mps2 =
            (planarMagnitude(est.body_pose_state.body_trans_mps) -
             planarMagnitude(prev_est.body_pose_state.body_trans_mps)) / dt_s;

        if (!expect(std::isfinite(roll_rate_radps) && std::isfinite(pitch_rate_radps) &&
                        std::isfinite(planar_accel_mps2),
                    "first-order orientation/velocity derivatives should remain finite")) {
            return EXIT_FAILURE;
        }

        max_roll_rate_radps = std::max(max_roll_rate_radps, std::abs(roll_rate_radps));
        max_pitch_rate_radps = std::max(max_pitch_rate_radps, std::abs(pitch_rate_radps));
        max_planar_accel_mps2 = std::max(max_planar_accel_mps2, std::abs(planar_accel_mps2));

        if (has_prev_derivative) {
            const double roll_jerk_radps2 = (roll_rate_radps - prev_roll_rate_radps) / dt_s;
            const double pitch_jerk_radps2 = (pitch_rate_radps - prev_pitch_rate_radps) / dt_s;
            const double planar_jerk_mps3 = (planar_accel_mps2 - prev_planar_accel_mps2) / dt_s;
            if (!expect(std::isfinite(roll_jerk_radps2) && std::isfinite(pitch_jerk_radps2) &&
                            std::isfinite(planar_jerk_mps3),
                        "second-order derivative checks should remain finite")) {
                return EXIT_FAILURE;
            }
            max_roll_jerk_radps2 = std::max(max_roll_jerk_radps2, std::abs(roll_jerk_radps2));
            max_pitch_jerk_radps2 = std::max(max_pitch_jerk_radps2, std::abs(pitch_jerk_radps2));
            max_planar_jerk_mps3 = std::max(max_planar_jerk_mps3, std::abs(planar_jerk_mps3));
        }

        has_prev_derivative = true;
        prev_roll_rate_radps = roll_rate_radps;
        prev_pitch_rate_radps = pitch_rate_radps;
        prev_planar_accel_mps2 = planar_accel_mps2;
        prev_est = est;
    }

    if (!expect(max_roll_rate_radps < 200.0 && max_pitch_rate_radps < 200.0,
                "tripod/slip transitions should not create unbounded orientation derivative spikes") ||
        !expect(max_planar_accel_mps2 < 500.0,
                "tripod/slip transitions should not create unbounded velocity derivative spikes") ||
        !expect(max_roll_jerk_radps2 < 20'000.0 && max_pitch_jerk_radps2 < 20'000.0,
                "tripod/slip transitions should keep orientation jerk bounded") ||
        !expect(max_planar_jerk_mps3 < 50'000.0,
                "tripod/slip transitions should keep velocity jerk bounded")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
