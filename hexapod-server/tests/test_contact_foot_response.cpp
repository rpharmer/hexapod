#include "contact_foot_response.hpp"

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

} // namespace

int main() {
    RobotState untrusted{};
    untrusted.foot_contacts[0] = true;
    if (!expect(!contact_foot_response::sensorsTrustedForContactResponse(untrusted),
                 "default estimator state should not trust contact-driven swing edits")) {
        return EXIT_FAILURE;
    }

    RobotState trusted{};
    trusted.valid = true;
    if (!expect(contact_foot_response::sensorsTrustedForContactResponse(trusted),
                 "valid estimator should trust contact sensors")) {
        return EXIT_FAILURE;
    }

    RobotState flagged{};
    flagged.has_valid_flag = true;
    if (!expect(contact_foot_response::sensorsTrustedForContactResponse(flagged),
                 "has_valid_flag should also enable contact response")) {
        return EXIT_FAILURE;
    }

    RobotState low_trust = trusted;
    low_trust.has_fusion_diagnostics = true;
    low_trust.fusion.model_trust = 0.10;
    low_trust.fusion.hard_reset_requested = false;
    if (!expect(!contact_foot_response::sensorsTrustedForContactResponse(low_trust),
                 "low fusion trust should disable contact-driven swing edits")) {
        return EXIT_FAILURE;
    }

    double tau = 0.0;
    double extra = 0.0;
    contact_foot_response::adjustSwingTauAndVerticalExtension(
        true, true, trusted, 0.2, tau, extra);
    if (!expect(std::abs(tau - 1.0) < 1e-12 && extra == 0.0,
                 "early ground contact during swing should snap tau to touchdown")) {
        return EXIT_FAILURE;
    }

    FootContactFusion predicted{};
    predicted.phase = ContactPhase::ExpectedTouchdown;
    predicted.confidence = 0.6f;
    tau = 0.0;
    extra = 0.0;
    contact_foot_response::adjustSwingTauAndVerticalExtension(
        true, false, trusted, 0.95, tau, extra, &predicted);
    if (!expect(extra > 0.0 && tau < 1.0,
                 "predicted touchdown should extend downward without forcing touchdown")) {
        return EXIT_FAILURE;
    }

    FootContactFusion stale_confirmed{};
    stale_confirmed.phase = ContactPhase::ConfirmedStance;
    stale_confirmed.confidence = 0.9f;
    tau = 0.95;
    extra = 0.0;
    contact_foot_response::adjustSwingTauAndVerticalExtension(
        true, false, trusted, 0.95, tau, extra, &stale_confirmed);
    if (!expect(std::abs(tau - 0.95) < 1e-12 && extra == 0.0,
                 "stale confirmed stance fusion should not override a lifted swing leg")) {
        return EXIT_FAILURE;
    }

    tau = 0.0;
    extra = 0.0;
    contact_foot_response::adjustSwingTauAndVerticalExtension(
        true, false, trusted, 0.95, tau, extra);
    if (!expect(extra > 0.0 && tau > 0.9,
                 "late swing without contact should extend foot downward")) {
        return EXIT_FAILURE;
    }

    tau = 0.0;
    extra = 0.0;
    contact_foot_response::adjustSwingTauAndVerticalExtension(
        true, false, untrusted, 0.99, tau, extra);
    if (!expect(extra == 0.0 && std::abs(tau - 0.99) < 1e-12,
                 "untrusted estimator should not apply late-swing extension")) {
        return EXIT_FAILURE;
    }

    MotionIntent intent{};
    intent.twist.twist_pos_rad = EulerAnglesRad3{0.0, 0.0, 0.0};
    RobotState tilted{};
    tilted.has_body_twist_state = true;
    tilted.body_twist_state.twist_pos_rad = EulerAnglesRad3{1.0, 1.0, 0.0};
    const double dz =
        contact_foot_response::stanceTiltLevelingDeltaZ(tilted, intent, 0.2, 0.2);
    if (!expect(std::abs(dz) > 1e-6 && std::abs(dz) <= 0.0140001,
                 "tilt error vs intent should produce bounded stance height tweak")) {
        return EXIT_FAILURE;
    }

    RobotState no_tilt{};
    no_tilt.has_body_twist_state = false;
    if (!expect(contact_foot_response::stanceTiltLevelingDeltaZ(no_tilt, intent, 0.1, 0.1) == 0.0,
                 "without body twist state, leveling delta should be zero")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
