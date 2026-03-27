#include "autonomy/modules/localization.hpp"

namespace autonomy {

LocalizationModuleShell::LocalizationModuleShell()
    : AutonomyModuleStub("localization") {}

LocalizationEstimate LocalizationModuleShell::update(const NavigationUpdate& navigation_update,
                                                     uint64_t timestamp_ms) {
    estimate_.timestamp_ms = timestamp_ms;
    if (navigation_update.has_intent) {
        estimate_.valid = true;
        estimate_.x_m = navigation_update.intent.target.x_m;
        estimate_.y_m = navigation_update.intent.target.y_m;
        estimate_.yaw_rad = navigation_update.intent.target.yaw_rad;
    }
    return estimate_;
}

LocalizationEstimate LocalizationModuleShell::estimate() const {
    return estimate_;
}

} // namespace autonomy
