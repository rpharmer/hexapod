#include "physics_sim_estimator.hpp"

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

RobotState makePhysicsSimSample(uint64_t sample_id, uint64_t timestamp_us, double body_z_m = 0.08) {
    RobotState raw{};
    raw.sample_id = sample_id;
    raw.timestamp_us = TimePointUs{timestamp_us};
    raw.valid = true;
    raw.has_valid_flag = true;
    raw.bus_ok = true;
    raw.has_body_twist_state = true;
    raw.body_twist_state.body_trans_m = PositionM3{0.0, 0.0, body_z_m};
    raw.body_twist_state.body_trans_mps = VelocityMps3{0.0, 0.0, 0.0};
    raw.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.0, 0.0, 0.0};
    raw.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.0, 0.0, 0.0};
    raw.foot_contacts = {true, true, true, true, true, true};
    return raw;
}

bool testHealthyStanceSample() {
    PhysicsSimEstimator estimator{};
    control_config::FusionConfig config{};
    estimator.configure(config);
    estimator.reset();

    const RobotState raw = makePhysicsSimSample(1, 1'000'000);
    const RobotState out = estimator.update(raw);

    return expect(out.has_fusion_diagnostics, "physics sim estimator should publish fusion diagnostics") &&
           expect(out.has_body_twist_state, "physics sim estimator should preserve body twist state") &&
           expect(out.foot_contacts[0], "raw contact should remain load-bearing after fusion") &&
           expect(out.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                  "physics sim estimator should confirm stable stance contact") &&
           expect(out.fusion.model_trust > 0.6, "stable sim stance should produce healthy model trust");
}

// Placeholder bus race: default RobotState has bus_ok=true but no body pose. Forcing
// has_body_twist_state latches fusion at z=0 and the next real sim pose trips hard_reset.
bool testPlaceholderWithoutBodyPoseDoesNotHardResetNextSample() {
    PhysicsSimEstimator estimator{};
    control_config::FusionConfig config{};
    config.hard_pose_resync_m = 0.05;
    config.soft_pose_resync_m = 0.02;
    estimator.configure(config);
    estimator.reset();

    RobotState placeholder{};
    placeholder.sample_id = 1;
    placeholder.timestamp_us = TimePointUs{1'000'000};
    placeholder.bus_ok = true;
    placeholder.valid = true;
    placeholder.has_valid_flag = true;
    placeholder.has_body_twist_state = false;
    placeholder.foot_contacts = {true, true, true, true, true, true};
    (void)estimator.update(placeholder);

    const RobotState measured = makePhysicsSimSample(2, 1'001'000, 0.14);
    const RobotState out = estimator.update(measured);

    return expect(!out.fusion.hard_reset_requested,
                  "placeholder without body pose must not latch fusion at z=0 and hard-reset on first real pose") &&
           expect(out.fusion.residuals.max_body_position_error_m < config.hard_pose_resync_m,
                  "first real sim pose residual should stay below hard_pose_resync_m");
}

} // namespace

int main() {
    if (!testHealthyStanceSample() || !testPlaceholderWithoutBodyPoseDoesNotHardResetNextSample()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
