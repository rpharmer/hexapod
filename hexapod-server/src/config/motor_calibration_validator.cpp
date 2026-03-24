#include "motor_calibration_validator.hpp"

#include <algorithm>
#include <array>
#include <set>

#include "hexapod-server.hpp"
#include "logger.hpp"

using namespace logging;

namespace {

constexpr int kExpectedJointCount = static_cast<int>(kProtocolJointCount);
constexpr int kMinServoPulse = 500;
constexpr int kMaxServoPulse = 2500;
const std::array<std::string, kExpectedJointCount> kExpectedJointOrder = {
    "R31", "R32", "R33", "L31", "L32", "L33",
    "R21", "R22", "R23", "L21", "L22", "L23",
    "R11", "R12", "R13", "L11", "L12", "L13"};

bool isCalibrationKeyValid(const std::string& key)
{
  return key.size() == 3 && (key[0] == 'R' || key[0] == 'L') && (key[1] >= '1' && key[1] <= '3') &&
         (key[2] >= '1' && key[2] <= '3');
}

} // namespace

namespace motor_calibration_validator {

bool validateAndNormalize(const std::vector<CalibrationRow>& raw_calibrations,
                          std::vector<float>& out_min_max_pulses,
                          const std::string& section_context,
                          std::shared_ptr<logging::AsyncLogger> logger)
{
  if (raw_calibrations.size() != kExpectedJointCount) {
    if (logger) {
      LOG_ERROR(logger, "[", section_context,
                "] invalid number of MotorCalibrations, expected ", kExpectedJointCount,
                ", got ", raw_calibrations.size());
    }
    return false;
  }

  std::set<std::string> seen_keys;
  std::set<std::string> expected_keys(kExpectedJointOrder.begin(), kExpectedJointOrder.end());
  std::vector<CalibrationRow> sorted = raw_calibrations;

  for (const auto& calib : sorted) {
    const auto& key = std::get<0>(calib);
    const int min_pulse = std::get<1>(calib);
    const int max_pulse = std::get<2>(calib);

    if (!isCalibrationKeyValid(key)) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context,
                  "] invalid motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (!expected_keys.contains(key)) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context,
                  "] unexpected motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (!seen_keys.insert(key).second) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context,
                  "] duplicate motor key '", key, "' in MotorCalibrations");
      }
      return false;
    }
    if (min_pulse >= max_pulse) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context,
                  "] invalid pulse bounds for '", key, "': min must be < max");
      }
      return false;
    }
    if (min_pulse < kMinServoPulse || max_pulse > kMaxServoPulse) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context, "] invalid pulse bounds for '", key,
                  "': must be within [", kMinServoPulse, ", ", kMaxServoPulse, "]");
      }
      return false;
    }
  }

  for (const auto& expected_key : expected_keys) {
    if (!seen_keys.contains(expected_key)) {
      if (logger) {
        LOG_ERROR(logger, "[", section_context,
                  "] missing motor key '", expected_key, "' in MotorCalibrations");
      }
      return false;
    }
  }

  std::sort(sorted.begin(), sorted.end(), [](const CalibrationRow& a, const CalibrationRow& b) -> bool {
    const std::string& key_a = std::get<0>(a);
    const std::string& key_b = std::get<0>(b);

    const std::array<int, 3> sort_key_a = {key_a[1] - '0', key_a[0] == 'R' ? 1 : 0,
                                           3 - (key_a[2] - '0')};
    const std::array<int, 3> sort_key_b = {key_b[1] - '0', key_b[0] == 'R' ? 1 : 0,
                                           3 - (key_b[2] - '0')};

    return sort_key_a > sort_key_b;
  });

  std::vector<float> normalized;
  normalized.reserve(kProtocolJointCount * kProtocolCalibrationPairsPerJoint);
  for (const auto& calib : sorted) {
    normalized.push_back(static_cast<float>(std::get<1>(calib)));
    normalized.push_back(static_cast<float>(std::get<2>(calib)));
  }

  out_min_max_pulses = std::move(normalized);
  return true;
}

} // namespace motor_calibration_validator
