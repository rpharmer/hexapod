#ifndef MOTOR_CALIBRATION_VALIDATOR_HPP
#define MOTOR_CALIBRATION_VALIDATOR_HPP

#include <string>
#include <tuple>
#include <vector>
#include <memory>

#include "logger.hpp"

namespace motor_calibration_validator {

using CalibrationRow = std::tuple<std::string, int, int>;

bool validateAndNormalize(const std::vector<CalibrationRow>& raw_calibrations,
                          std::vector<float>& out_min_max_pulses,
                          const std::string& section_context,
                          std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace motor_calibration_validator

#endif // MOTOR_CALIBRATION_VALIDATOR_HPP
