#include "autonomy/modules/localization.hpp"

#include <utility>

namespace autonomy {

namespace {

TimestampMs toMilliseconds(TimePointUs timestamp_us) {
    return TimestampMs{timestamp_us.value / 1000};
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
                                                                  TimestampMs timestamp_ms,
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

LocalizationModuleShell::LocalizationModuleShell(LocalizationValidationPolicy policy)
    : AutonomyModuleStub("localization"),
      policy_(std::move(policy)) {}

LocalizationEstimate LocalizationModuleShell::update(const LocalizationSourceObservation& observation,
                                                     uint64_t now_ms,
                                                     ContractEnvelope envelope) {
    (void)envelope;
    estimate_.valid = false;
    estimate_.frame_id = policy_.expected_frame_id;

    if (!observation.valid || !isFrameValid(observation) || !isObservationFresh(observation, now_ms)) {
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

bool LocalizationModuleShell::isFrameValid(const LocalizationSourceObservation& observation) const {
    if (!policy_.require_frame_match) {
        return true;
    }
    return observation.frame_id == policy_.expected_frame_id;
}

bool LocalizationModuleShell::isObservationFresh(const LocalizationSourceObservation& observation,
                                                 uint64_t now_ms) const {
    if (policy_.reject_zero_timestamp && observation.timestamp_ms == TimestampMs{}) {
        return false;
    }
    if (policy_.reject_future_timestamps && observation.timestamp_ms > now_ms) {
        return false;
    }
    return (now_ms - observation.timestamp_ms).value <= policy_.stale_threshold_ms;
}

} // namespace autonomy
