#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

#include "core/types.hpp"

#include <cstdint>
#include <string>

namespace autonomy {

struct LocalizationValidationPolicy {
    std::string expected_frame_id{"map"};
    uint64_t stale_threshold_ms{250};
    bool require_frame_match{true};
    bool reject_future_timestamps{true};
    bool reject_zero_timestamp{true};
};

struct LocalizationSourceObservation {
    bool valid{false};
    std::string frame_id{"map"};
    double x_m{0.0};
    double y_m{0.0};
    double yaw_rad{0.0};
    TimestampMs timestamp_ms{};

    PositionM3 position_m() const {
        return PositionM3{x_m, y_m, 0.0};
    }

    AngleRad yaw() const {
        return AngleRad{yaw_rad};
    }
};

LocalizationSourceObservation localizationObservationFromEstimator(const RobotState& estimator_state,
                                                                   const std::string& frame_id = "map");
LocalizationSourceObservation localizationObservationFromOdometry(double x_m,
                                                                  double y_m,
                                                                  double yaw_rad,
                                                                  TimestampMs timestamp_ms,
                                                                  const std::string& frame_id = "odom");

class LocalizationModuleShell : public AutonomyModuleStub {
public:
    explicit LocalizationModuleShell(LocalizationValidationPolicy policy = {});

    LocalizationEstimate update(const LocalizationSourceObservation& observation,
                                uint64_t now_ms,
                                ContractEnvelope envelope = {});
    [[nodiscard]] LocalizationEstimate estimate() const;

private:
    bool isFrameValid(const LocalizationSourceObservation& observation) const;
    bool isObservationFresh(const LocalizationSourceObservation& observation,
                            uint64_t now_ms) const;

    LocalizationEstimate estimate_{};
    LocalizationValidationPolicy policy_{};
};

} // namespace autonomy
