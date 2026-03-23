#include "config_validation.hpp"

#include <array>

#include "logger.hpp"

using namespace logging;

namespace config_validation {

int parseIntWithFallback(const toml::value& root,
                         const std::string& key,
                         int default_value,
                         int min_value,
                         int max_value,
                         const std::string& section)
{
  const int value = toml::find_or<int>(root, key, default_value);
  if (value < min_value || value > max_value) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "invalid ", section, " value for ", key, "=", value, ", using default ",
               default_value);
    }
    return default_value;
  }
  return value;
}

uint64_t parseU64WithFallback(const toml::value& root,
                              const std::string& key,
                              uint64_t default_value,
                              uint64_t min_value,
                              uint64_t max_value,
                              const std::string& section)
{
  const int64_t raw_value = toml::find_or<int64_t>(root, key, static_cast<int64_t>(default_value));
  if (raw_value < static_cast<int64_t>(min_value) ||
      raw_value > static_cast<int64_t>(max_value)) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "invalid ", section, " value for ", key, "=", raw_value,
               ", using default ", default_value);
    }
    return default_value;
  }
  return static_cast<uint64_t>(raw_value);
}

double parseDoubleWithFallback(const toml::value& root,
                               const std::string& key,
                               double default_value,
                               double min_value,
                               double max_value,
                               const std::string& section)
{
  const double value = toml::find_or<double>(root, key, default_value);
  if (value < min_value || value > max_value) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "invalid ", section, " value for ", key, "=", value, ", using default ",
               default_value);
    }
    return default_value;
  }
  return value;
}

bool parseBoolWithFallback(const toml::value& root,
                           const std::string& key,
                           bool default_value)
{
  return toml::find_or<bool>(root, key, default_value);
}

std::vector<double> parseDoubleListWithFallback(const toml::value& root,
                                                const std::string& key,
                                                const std::vector<double>& defaults,
                                                std::size_t expected_size,
                                                double min_value,
                                                double max_value,
                                                const std::string& section)
{
  const std::vector<double> values = toml::find_or<std::vector<double>>(root, key, defaults);
  if (values.size() != expected_size) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "invalid ", section, " list size for ", key, ": expected ", expected_size,
               ", got ", values.size(), ", using defaults");
    }
    return defaults;
  }

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] < min_value || values[i] > max_value) {
      if (auto logger = GetDefaultLogger()) {
        LOG_WARN(logger, "invalid ", section, " value for ", key, "[", i, "] = ", values[i],
                 ", using defaults");
      }
      return defaults;
    }
  }
  return values;
}

std::vector<Vec3> parseVec3ListWithFallback(const toml::value& root,
                                            const std::string& key,
                                            const std::vector<Vec3>& defaults,
                                            std::size_t expected_size,
                                            double min_value,
                                            double max_value,
                                            const std::string& section)
{
  const std::vector<std::array<double, 3>> raw =
      toml::find_or<std::vector<std::array<double, 3>>>(root, key, {});

  if (raw.empty()) {
    return defaults;
  }

  if (raw.size() != expected_size) {
    if (auto logger = GetDefaultLogger()) {
      LOG_WARN(logger, "invalid ", section, " list size for ", key, ": expected ", expected_size,
               ", got ", raw.size(), ", using defaults");
    }
    return defaults;
  }

  std::vector<Vec3> parsed;
  parsed.reserve(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto& row = raw[i];
    for (std::size_t axis = 0; axis < row.size(); ++axis) {
      if (row[axis] < min_value || row[axis] > max_value) {
        if (auto logger = GetDefaultLogger()) {
          LOG_WARN(logger, "invalid ", section, " value for ", key, "[", i, "][", axis,
                   "] = ", row[axis], ", using defaults");
        }
        return defaults;
      }
    }
    parsed.push_back(Vec3{row[0], row[1], row[2]});
  }
  return parsed;
}

} // namespace config_validation
