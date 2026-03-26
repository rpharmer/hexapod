#include "geometry_profile_service.hpp"

#include <cmath>
#include <optional>

namespace {

std::optional<HexapodGeometry> g_preview_geometry;
std::optional<HexapodGeometry> g_rollback_geometry;

bool validateServoDirectionDynamics(const ServoDirectionDynamics& dynamics) {
    return std::isfinite(dynamics.tau_s) && std::isfinite(dynamics.vmax_radps) &&
           dynamics.tau_s >= 0.0 && dynamics.vmax_radps >= 0.0;
}

bool validateGeometry(const HexapodGeometry& geometry, std::string* error) {
    if (!std::isfinite(geometry.toBottom.value) || geometry.toBottom.value < 0.0) {
        if (error != nullptr) {
            *error = "geometry.toBottom must be finite and non-negative";
        }
        return false;
    }

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geometry = geometry.legGeometry[leg];
        if (!isFinite(leg_geometry.bodyCoxaOffset) ||
            !std::isfinite(leg_geometry.mountAngle.value)) {
            if (error != nullptr) {
                *error = "leg geometry positions/angles must be finite";
            }
            return false;
        }

        if (!std::isfinite(leg_geometry.coxaLength.value) ||
            !std::isfinite(leg_geometry.femurLength.value) ||
            !std::isfinite(leg_geometry.tibiaLength.value) ||
            leg_geometry.coxaLength.value < 0.0 ||
            leg_geometry.femurLength.value < 0.0 ||
            leg_geometry.tibiaLength.value < 0.0) {
            if (error != nullptr) {
                *error = "leg link lengths must be finite and non-negative";
            }
            return false;
        }

        const ServoCalibration& servo = leg_geometry.servo;
        if (!std::isfinite(servo.coxaOffset.value) ||
            !std::isfinite(servo.femurOffset.value) ||
            !std::isfinite(servo.tibiaOffset.value) ||
            !std::isfinite(servo.coxaSign) ||
            !std::isfinite(servo.femurSign) ||
            !std::isfinite(servo.tibiaSign)) {
            if (error != nullptr) {
                *error = "servo calibration values must be finite";
            }
            return false;
        }

        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const ServoJointDynamics& dynamics = leg_geometry.servoDynamics[joint];
            if (!validateServoDirectionDynamics(dynamics.positive_direction) ||
                !validateServoDirectionDynamics(dynamics.negative_direction)) {
                if (error != nullptr) {
                    *error = "servo dynamics limits must be finite and non-negative";
                }
                return false;
            }
        }
    }

    return true;
}

} // namespace

namespace geometry_profile_service {

bool preview(const HexapodGeometry& candidate, std::string* error) {
    if (!validateGeometry(candidate, error)) {
        return false;
    }
    g_preview_geometry = candidate;
    return true;
}

bool apply(std::string* error) {
    if (!g_preview_geometry.has_value()) {
        if (error != nullptr) {
            *error = "cannot apply geometry profile without an active preview";
        }
        return false;
    }

    g_rollback_geometry = geometry_config::kHexapodGeometry;
    geometry_config::kHexapodGeometry = *g_preview_geometry;
    g_preview_geometry.reset();
    return true;
}

bool rollback(std::string* error) {
    if (g_rollback_geometry.has_value()) {
        geometry_config::kHexapodGeometry = *g_rollback_geometry;
        g_rollback_geometry.reset();
        g_preview_geometry.reset();
        return true;
    }

    if (g_preview_geometry.has_value()) {
        g_preview_geometry.reset();
        return true;
    }

    if (error != nullptr) {
        *error = "cannot rollback geometry profile without a preview or applied snapshot";
    }
    return false;
}

bool persist(std::string* error) {
    if (error != nullptr) {
        *error = "persist not implemented: storage design pending";
    }
    return false;
}

const HexapodGeometry* previewSnapshot() {
    if (!g_preview_geometry.has_value()) {
        return nullptr;
    }
    return &(*g_preview_geometry);
}

} // namespace geometry_profile_service
