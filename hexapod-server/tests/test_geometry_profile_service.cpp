#include "geometry_profile_service.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool restoreBaseline(const HexapodGeometry& baseline) {
    std::string error;
    if (!geometry_profile_service::preview(baseline, &error)) {
        std::cerr << "FAIL: baseline preview restore failed: " << error << '\n';
        return false;
    }
    if (!geometry_profile_service::apply(&error)) {
        std::cerr << "FAIL: baseline apply restore failed: " << error << '\n';
        return false;
    }
    return true;
}

bool testPreviewApplyRollbackTransitions() {
    const HexapodGeometry baseline = geometry_config::activeHexapodGeometry();

    HexapodGeometry candidate = baseline;
    candidate.toBottom = LengthM{baseline.toBottom.value + 0.01};
    candidate.legGeometry[0].servoDynamics[COXA].positive_direction.tau_s = 0.12;

    std::string error;
    if (!expect(geometry_profile_service::preview(candidate, &error),
                "preview should accept valid candidate")) {
        return false;
    }
    if (!expect(geometry_profile_service::previewSnapshot() != nullptr,
                "preview snapshot should be available after preview")) {
        return false;
    }

    if (!expect(geometry_profile_service::apply(&error),
                "apply should succeed after preview")) {
        return false;
    }

    const HexapodGeometry& active_after_apply = geometry_config::activeHexapodGeometry();
    if (!expect(std::abs(active_after_apply.toBottom.value - candidate.toBottom.value) < 1e-12,
                "apply should update active geometry")) {
        restoreBaseline(baseline);
        return false;
    }
    if (!expect(geometry_profile_service::previewSnapshot() == nullptr,
                "preview should be cleared after apply")) {
        restoreBaseline(baseline);
        return false;
    }

    if (!expect(geometry_profile_service::rollback(&error),
                "rollback should restore pre-apply geometry")) {
        restoreBaseline(baseline);
        return false;
    }
    const HexapodGeometry& active_after_rollback = geometry_config::activeHexapodGeometry();
    if (!expect(std::abs(active_after_rollback.toBottom.value - baseline.toBottom.value) < 1e-12,
                "rollback should restore original toBottom")) {
        restoreBaseline(baseline);
        return false;
    }

    return restoreBaseline(baseline);
}

bool testInvariantValidation() {
    const HexapodGeometry baseline = geometry_config::activeHexapodGeometry();

    HexapodGeometry invalid = baseline;
    invalid.legGeometry[2].femurLength = LengthM{-0.01};

    std::string error;
    if (!expect(!geometry_profile_service::preview(invalid, &error),
                "preview should reject negative link lengths")) {
        restoreBaseline(baseline);
        return false;
    }

    invalid = baseline;
    invalid.legGeometry[1].servoDynamics[TIBIA].positive_direction.vmax_radps =
        std::numeric_limits<double>::infinity();
    if (!expect(!geometry_profile_service::preview(invalid, &error),
                "preview should reject non-finite servo dynamics")) {
        restoreBaseline(baseline);
        return false;
    }

    return restoreBaseline(baseline);
}

bool testPerLegPropagationApply() {
    const HexapodGeometry baseline = geometry_config::activeHexapodGeometry();

    HexapodGeometry candidate = baseline;
    std::array<ServoJointDynamics, kJointsPerLeg> profile{};
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        profile[joint].positive_direction.tau_s = 0.05 + 0.01 * static_cast<double>(joint);
        profile[joint].negative_direction.tau_s = 0.07 + 0.01 * static_cast<double>(joint);
        profile[joint].positive_direction.vmax_radps = 1.0 + static_cast<double>(joint);
        profile[joint].negative_direction.vmax_radps = 2.0 + static_cast<double>(joint);
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        candidate.legGeometry[leg].servoDynamics = profile;
    }

    std::string error;
    if (!expect(geometry_profile_service::preview(candidate, &error),
                "preview should accept propagated per-leg profile")) {
        restoreBaseline(baseline);
        return false;
    }
    if (!expect(geometry_profile_service::apply(&error),
                "apply should accept propagated per-leg profile")) {
        restoreBaseline(baseline);
        return false;
    }

    const HexapodGeometry& active = geometry_config::activeHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const ServoJointDynamics& observed = active.legGeometry[leg].servoDynamics[joint];
            if (!expect(std::abs(observed.positive_direction.tau_s -
                                 profile[joint].positive_direction.tau_s) < 1e-12,
                        "positive tau should propagate to every leg")) {
                restoreBaseline(baseline);
                return false;
            }
            if (!expect(std::abs(observed.negative_direction.vmax_radps -
                                 profile[joint].negative_direction.vmax_radps) < 1e-12,
                        "negative vmax should propagate to every leg")) {
                restoreBaseline(baseline);
                return false;
            }
        }
    }

    return restoreBaseline(baseline);
}

} // namespace

int main() {
    if (!testPreviewApplyRollbackTransitions() ||
        !testInvariantValidation() ||
        !testPerLegPropagationApply()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
