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

RobotState makePhysicsSimSample(uint64_t sample_id, uint64_t timestamp_us) {
    RobotState raw{};
    raw.sample_id = sample_id;
    raw.timestamp_us = TimePointUs{timestamp_us};
    raw.valid = true;
    raw.has_valid_flag = true;
    raw.has_body_twist_state = true;
    raw.body_twist_state.body_trans_m = PositionM3{0.0, 0.0, 0.08};
    raw.body_twist_state.body_trans_mps = VelocityMps3{0.0, 0.0, 0.0};
    raw.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.0, 0.0, 0.0};
    raw.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.0, 0.0, 0.0};
    raw.foot_contacts = {true, true, true, true, true, true};
    return raw;
}

} // namespace

int main() {
    PhysicsSimEstimator estimator{};
    control_config::FusionConfig config{};
    estimator.configure(config);
    estimator.reset();

    const RobotState raw = makePhysicsSimSample(1, 1'000'000);
    const RobotState out = estimator.update(raw);

    if (!expect(out.has_fusion_diagnostics, "physics sim estimator should publish fusion diagnostics") ||
        !expect(out.has_body_twist_state, "physics sim estimator should preserve body twist state") ||
        !expect(out.foot_contacts[0], "raw contact should remain load-bearing after fusion") ||
        !expect(out.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "physics sim estimator should confirm stable stance contact") ||
        !expect(out.fusion.model_trust > 0.6, "stable sim stance should produce healthy model trust")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
