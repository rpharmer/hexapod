#include "estimator.hpp"
#include "math_types.hpp"

#include <chrono>
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

RobotState makeBoundaryCrossingRaw(uint64_t sample_id,
                                   uint64_t timestamp_us,
                                   double coxa_angle_rad) {
    RobotState raw = makeNeutralRaw(sample_id, timestamp_us);
    raw.leg_states[0].joint_state[COXA].pos_rad = AngleRad{coxa_angle_rad};
    return raw;
}

} // namespace

int main() {
    SimpleEstimator estimator{};

    const RobotState first = makeNeutralRaw(1, 1'000'000);
    const RobotState first_est = estimator.update(first);

    if (!expect(std::abs(first_est.body_twist_state.twist_pos_rad.x) < 1e-6,
                "neutral stance should estimate near-zero roll") ||
        !expect(std::abs(first_est.body_twist_state.twist_pos_rad.y) < 1e-6,
                "neutral stance should estimate near-zero pitch") ||
        !expect(first_est.body_twist_state.body_trans_m.z > 0.0,
                "neutral stance should estimate body above the ground plane") ||
        !expect(first_est.has_imu && first_est.imu.valid,
                "software IMU should be synthesized from the foot contact plane when raw has no IMU") ||
        !expect(first_est.has_fusion_diagnostics && first_est.foot_contacts[0] &&
                    first_est.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "fusion should confirm the initial stance contact state") ||
        !expect(std::abs(vecNorm(first_est.imu.accel_mps2) - 9.80665) < 1e-3,
                "software IMU should report ~1g specific-force when the plane is near level")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_recent = first;
    no_contact_recent.sample_id = 2;
    no_contact_recent.timestamp_us = TimePointUs{1'100'000};
    no_contact_recent.foot_contacts = {false, false, false, false, false, false};
    no_contact_recent.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.1};

    const RobotState recent_est = estimator.update(no_contact_recent);
    if (!expect(recent_est.body_twist_state.body_trans_m.z > 0.0,
                "recently touching feet should still support a ground estimate") ||
        !expect(recent_est.has_fusion_diagnostics && recent_est.foot_contacts[0] &&
                    recent_est.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "brief contact loss should stay in confirmed stance while the hold window is open") ||
        !expect(std::abs(recent_est.leg_states[0].joint_state[COXA].vel_radps.value - 1.0) < 1e-6,
                "joint velocity should be estimated from position delta over dt")) {
        return EXIT_FAILURE;
    }

    RobotState no_contact_stale = no_contact_recent;
    no_contact_stale.sample_id = 3;
    no_contact_stale.timestamp_us = TimePointUs{1'500'001};

    const RobotState stale_est = estimator.update(no_contact_stale);
    if (!expect(std::abs(stale_est.body_twist_state.body_trans_m.z) < 1e-9,
                "stale no-contact window should clear ground estimate") ||
        !expect(!stale_est.has_body_twist_state && !stale_est.has_imu,
                "without a plane estimate, software IMU should not be published") ||
        !expect(stale_est.has_fusion_diagnostics && !stale_est.foot_contacts[0],
                "stale no-contact windows should drop confirmed stance")) {
        return EXIT_FAILURE;
    }

    SimpleEstimator physics_estimator{};
    control_config::FusionConfig physics_config{};
    physics_config.touchdown_window = std::chrono::milliseconds{40};
    physics_config.contact_hold_window = std::chrono::milliseconds{120};
    physics_estimator.configure(physics_config);

    const RobotState physics_first = makeNeutralRaw(10, 2'000'000);
    (void)physics_estimator.update(physics_first);

    RobotState physics_no_contact = physics_first;
    physics_no_contact.sample_id = 11;
    physics_no_contact.timestamp_us = TimePointUs{2'150'000};
    physics_no_contact.foot_contacts = {false, false, false, false, false, false};

    const RobotState physics_no_contact_est = physics_estimator.update(physics_no_contact);
    if (!expect(!physics_no_contact_est.foot_contacts[0],
                "short physics-sim contact hold should release stance sooner") ||
        !expect(!physics_no_contact_est.has_body_twist_state,
                "short physics-sim contact hold should stop reusing the stale ground plane")) {
        return EXIT_FAILURE;
    }

    const RobotState branch_seed = makeBoundaryCrossingRaw(4, 1'700'000, 3.10);
    const RobotState branch_seed_est = estimator.update(branch_seed);
    const RobotState branch_wrap = makeBoundaryCrossingRaw(5, 1'800'000, -3.08);
    const RobotState branch_wrap_est = estimator.update(branch_wrap);
    if (!expect(branch_wrap_est.leg_states[0].joint_state[COXA].pos_rad.value >
                    branch_seed_est.leg_states[0].joint_state[COXA].pos_rad.value,
                "estimator should unwrap joint samples across the +/-pi boundary") ||
        !expect(branch_wrap_est.leg_states[0].joint_state[COXA].pos_rad.value > 3.0,
                "estimator should preserve a continuous joint value past pi")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
