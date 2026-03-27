#include "autonomy/modules/localization.hpp"

#include <utility>

namespace autonomy {

namespace {

uint64_t toMilliseconds(TimePointUs timestamp_us) {
    return timestamp_us.value / 1000;
}

} // namespace

LocalizationSourceObservation localizationObservationFromEstimator(const RobotState& estimator_state,
                                                                   const std::string& frame_id) {
    LocalizationSourceObservation observation{};
    observation.valid = estimator_state.valid && estimator_state.has_body_pose_state;
    observation.frame_id = frame_id;
    observation.x_m = estimator_state.body_pose_state.body_trans_m.x;
    observation.y_m = estimator_state.body_pose_state.body_trans_m.y;
    observation.yaw_rad = estimator_state.body_pose_state.orientation_rad.z;
    observation.timestamp_ms = toMilliseconds(estimator_state.timestamp_us);
    return observation;
}

LocalizationSourceObservation localizationObservationFromOdometry(double x_m,
                                                                  double y_m,
                                                                  double yaw_rad,
                                                                  uint64_t timestamp_ms,
                                                                  const std::string& frame_id) {
    return LocalizationSourceObservation{
        .valid = true,
        .frame_id = frame_id,
        .x_m = x_m,
        .y_m = y_m,
        .yaw_rad = yaw_rad,
        .timestamp_ms = timestamp_ms,
    };
}

LocalizationModuleShell::LocalizationModuleShell(std::string expected_frame,
                                                 uint64_t stale_threshold_ms)
    : AutonomyModuleStub("localization"),
      expected_frame_(std::move(expected_frame)),
      stale_threshold_ms_(stale_threshold_ms) {}

LocalizationEstimate LocalizationModuleShell::update(const LocalizationSourceObservation& observation,
                                                     uint64_t now_ms) {
    estimate_.valid = false;
    estimate_.frame_id = expected_frame_;

    if (!observation.valid || observation.frame_id != expected_frame_ || !isObservationFresh(observation, now_ms)) {
        estimate_.timestamp_ms = now_ms;
        return estimate_;
    }

    estimate_.valid = true;
    estimate_.frame_id = observation.frame_id;
    estimate_.x_m = observation.x_m;
    estimate_.y_m = observation.y_m;
    estimate_.yaw_rad = observation.yaw_rad;
    estimate_.timestamp_ms = observation.timestamp_ms;
    return estimate_;
}

LocalizationEstimate LocalizationModuleShell::estimate() const {
    return estimate_;
}

bool LocalizationModuleShell::isObservationFresh(const LocalizationSourceObservation& observation,
                                                 uint64_t now_ms) const {
    if (observation.timestamp_ms == 0 || observation.timestamp_ms > now_ms) {
        return false;
    }
    return (now_ms - observation.timestamp_ms) <= stale_threshold_ms_;
}

} // namespace autonomy
