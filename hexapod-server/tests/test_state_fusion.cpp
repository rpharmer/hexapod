#include "state_fusion.hpp"

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

RobotState makeSample(uint64_t sample_id,
                      uint64_t timestamp_us,
                      bool contact,
                      double body_z_m,
                      double roll_rad = 0.0,
                      double pitch_rad = 0.0) {
    RobotState st{};
    st.sample_id = sample_id;
    st.timestamp_us = TimePointUs{timestamp_us};
    st.foot_contacts.fill(contact);
    st.has_body_twist_state = true;
    st.body_twist_state.body_trans_m = PositionM3{0.0, 0.0, body_z_m};
    st.body_twist_state.body_trans_mps = VelocityMps3{0.0, 0.0, 0.0};
    st.body_twist_state.twist_pos_rad = EulerAnglesRad3{roll_rad, pitch_rad, 0.0};
    st.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.0, 0.0, 0.0};
    return st;
}

} // namespace

int main() {
    control_config::FusionConfig config{};
    config.contact_debounce_samples = 2;
    config.touchdown_window = std::chrono::milliseconds{40};
    config.contact_hold_window = std::chrono::milliseconds{80};
    config.soft_pose_resync_m = 0.03;
    config.hard_pose_resync_m = 0.12;

    state_fusion::StateFusion fusion{config};

    RobotState first = makeSample(1, 1'000'000, true, 0.08);
    RobotState out1 = fusion.update(first, state_fusion::FusionSourceMode::Measured);
    if (!expect(out1.has_fusion_diagnostics, "fusion diagnostics should be attached") ||
        !expect(out1.foot_contacts[0], "confirmed contact should remain load-bearing") ||
        !expect(out1.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "first contact sample should confirm stance immediately") ||
        !expect(out1.fusion.model_trust > 0.6, "good contact confidence should produce healthy trust")) {
        return EXIT_FAILURE;
    }

    RobotState release = makeSample(2, 1'040'000, false, 0.08);
    RobotState out2 = fusion.update(release, state_fusion::FusionSourceMode::Measured);
    if (!expect(out2.foot_contacts[0], "hold window should keep stance briefly after contact loss") ||
        !expect(out2.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "contact hold should stay in confirmed stance during the debounce window")) {
        return EXIT_FAILURE;
    }

    RobotState reacquire = makeSample(3, 1'140'000, true, 0.08);
    RobotState out3 = fusion.update(reacquire, state_fusion::FusionSourceMode::Measured);
    if (!expect(out3.foot_contact_fusion[0].phase == ContactPhase::ExpectedTouchdown ||
                    out3.foot_contact_fusion[0].phase == ContactPhase::ContactCandidate ||
                    out3.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "reacquired contact should re-enter touchdown validation") ||
        !expect(out3.fusion.model_trust <= out1.fusion.model_trust,
                "reacquisition should be no more trusted than the stable initial stance")) {
        return EXIT_FAILURE;
    }

    RobotState confirmed = makeSample(4, 1'180'000, true, 0.08);
    RobotState out4 = fusion.update(confirmed, state_fusion::FusionSourceMode::Measured);
    if (!expect(out4.foot_contacts[0], "second contact sample should confirm stance again") ||
        !expect(out4.foot_contact_fusion[0].phase == ContactPhase::ConfirmedStance,
                "debounced contact should return to confirmed stance")) {
        return EXIT_FAILURE;
    }

    RobotState baseline = makeSample(5, 1'220'000, true, 0.08);
    (void)fusion.update(baseline, state_fusion::FusionSourceMode::Measured);

    RobotState divergent = makeSample(6, 1'260'000, true, 0.28, 0.85, 0.0);
    RobotState out5 = fusion.update(divergent, state_fusion::FusionSourceMode::Measured);
    if (!expect(out5.has_fusion_diagnostics, "divergent sample should still report diagnostics") ||
        !expect(out5.fusion.resync_requested, "large pose jumps should request a resync") ||
        !expect(out5.fusion.hard_reset_requested, "large pose jumps should request a hard reset") ||
        !expect(out5.fusion.model_trust < out4.fusion.model_trust,
                "large pose jumps should reduce model trust")) {
        return EXIT_FAILURE;
    }

    state_fusion::StateFusion contact_only_fusion{config};
    (void)contact_only_fusion.update(makeSample(7, 1'300'000, true, 0.28, 0.0, 0.0),
                                     state_fusion::FusionSourceMode::Measured);
    RobotState contact_churn = makeSample(8, 1'340'000, false, 0.28, 0.0, 0.0);
    RobotState out6 = contact_only_fusion.update(contact_churn, state_fusion::FusionSourceMode::Measured);
    if (!expect(out6.has_fusion_diagnostics, "contact churn should still report diagnostics") ||
        !expect(!out6.fusion.resync_requested, "contact churn alone should not request a resync") ||
        !expect(!out6.fusion.hard_reset_requested, "contact churn alone should not request a hard reset")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
