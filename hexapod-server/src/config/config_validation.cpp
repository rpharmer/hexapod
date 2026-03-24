#include "config_validation.hpp"

#include <array>
#include <sstream>
#include <string_view>

#include "logger.hpp"

using namespace logging;

namespace config_validation {

namespace {

std::vector<std::string> splitDottedPath(const std::string& dotted_key)
{
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= dotted_key.size()) {
    const std::size_t dot = dotted_key.find('.', start);
    if (dot == std::string::npos) {
      parts.push_back(dotted_key.substr(start));
      break;
    }
    parts.push_back(dotted_key.substr(start, dot - start));
    start = dot + 1;
  }
  return parts;
}

template <typename T>
T findOrByPath(const toml::value& root, const std::string& dotted_key, const T& default_value)
{
  const toml::value* current = &root;
  for (const auto& part : splitDottedPath(dotted_key)) {
    if (!current->is_table()) {
      return default_value;
    }
    const auto& table = current->as_table();
    const auto it = table.find(part);
    if (it == table.end()) {
      return default_value;
    }
    current = &(it->second);
  }
  try {
    return toml::get<T>(*current);
  } catch (const std::exception&) {
    return default_value;
  }
}

template <typename T>
T parseBoundedScalarWithFallback(const toml::value& root,
                                 const std::string& key,
                                 T default_value,
                                 T min_value,
                                 T max_value,
                                 const std::string& section,
                                 std::shared_ptr<logging::AsyncLogger> logger,
                                 std::vector<ConfigDiagnostic>* diagnostics)
{
  const T value = findOrByPath<T>(root, key, default_value);
  if (value < min_value || value > max_value) {
    std::ostringstream message;
    message << "invalid value for " << key << "=" << value << ", using default " << default_value;
    if (logger) {
      LOG_WARN(logger, "invalid ", section, " value for ", key, "=", value, ", using default ",
               default_value);
    }
    emitDiagnostic(diagnostics, section, key, "out_of_range", message.str());
    return default_value;
  }
  return value;
}

template <typename T>
std::vector<T> parseBoundedVectorWithFallback(const std::vector<T>& values,
                                              const std::string& key,
                                              const std::vector<T>& defaults,
                                              std::size_t expected_size,
                                              T min_value,
                                              T max_value,
                                              const std::string& section,
                                              std::shared_ptr<logging::AsyncLogger> logger,
                                              std::vector<ConfigDiagnostic>* diagnostics)
{
  if (values.size() != expected_size) {
    if (logger) {
      LOG_WARN(logger, "invalid ", section, " list size for ", key, ": expected ", expected_size,
               ", got ", values.size(), ", using defaults");
    }
    std::ostringstream message;
    message << "invalid list size: expected " << expected_size << ", got " << values.size()
            << ", using defaults";
    emitDiagnostic(diagnostics, section, key, "invalid_size", message.str());
    return defaults;
  }

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] < min_value || values[i] > max_value) {
      if (logger) {
        LOG_WARN(logger, "invalid ", section, " value for ", key, "[", i, "] = ", values[i],
                 ", using defaults");
      }
      std::ostringstream message;
      message << "invalid value for " << key << "[" << i << "]=" << values[i]
              << ", using defaults";
      emitDiagnostic(diagnostics, section, key, "out_of_range", message.str());
      return defaults;
    }
  }
  return values;
}

} // namespace

void emitDiagnostic(std::vector<ConfigDiagnostic>* diagnostics,
                    const std::string& section,
                    const std::string& key,
                    const std::string& error_code,
                    const std::string& message)
{
  if (!diagnostics) {
    return;
  }
  diagnostics->push_back(ConfigDiagnostic{section, key, error_code, message});
}

int parseIntWithFallback(const toml::value& root,
                         const std::string& key,
                         int default_value,
                         int min_value,
                         int max_value,
                         const std::string& section,
                         std::shared_ptr<logging::AsyncLogger> logger,
                         std::vector<ConfigDiagnostic>* diagnostics)
{
  return parseBoundedScalarWithFallback<int>(root, key, default_value, min_value, max_value,
                                             section, logger, diagnostics);
}

uint64_t parseU64WithFallback(const toml::value& root,
                              const std::string& key,
                              uint64_t default_value,
                              uint64_t min_value,
                              uint64_t max_value,
                              const std::string& section,
                              std::shared_ptr<logging::AsyncLogger> logger,
                              std::vector<ConfigDiagnostic>* diagnostics)
{
  const int64_t raw_value =
      findOrByPath<int64_t>(root, key, static_cast<int64_t>(default_value));
  if (raw_value < static_cast<int64_t>(min_value) ||
      raw_value > static_cast<int64_t>(max_value)) {
    if (logger) {
      LOG_WARN(logger, "invalid ", section, " value for ", key, "=", raw_value,
               ", using default ", default_value);
    }
    std::ostringstream message;
    message << "invalid value for " << key << "=" << raw_value << ", using default "
            << default_value;
    emitDiagnostic(diagnostics, section, key, "out_of_range", message.str());
    return default_value;
  }
  return static_cast<uint64_t>(raw_value);
}

double parseDoubleWithFallback(const toml::value& root,
                               const std::string& key,
                               double default_value,
                               double min_value,
                               double max_value,
                               const std::string& section,
                               std::shared_ptr<logging::AsyncLogger> logger,
                               std::vector<ConfigDiagnostic>* diagnostics)
{
  return parseBoundedScalarWithFallback<double>(root, key, default_value, min_value, max_value,
                                                section, logger, diagnostics);
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
                                                const std::string& section,
                                                std::shared_ptr<logging::AsyncLogger> logger,
                                                std::vector<ConfigDiagnostic>* diagnostics)
{
  const std::vector<double> parsed_values =
      findOrByPath<std::vector<double>>(root, key, defaults);
  return parseBoundedVectorWithFallback<double>(parsed_values, key, defaults, expected_size, min_value,
                                                max_value, section, logger, diagnostics);
}

std::vector<Vec3> parseVec3ListWithFallback(const toml::value& root,
                                            const std::string& key,
                                            const std::vector<Vec3>& defaults,
                                            std::size_t expected_size,
                                            double min_value,
                                            double max_value,
                                            const std::string& section,
                                            std::shared_ptr<logging::AsyncLogger> logger,
                                            std::vector<ConfigDiagnostic>* diagnostics)
{
  const std::vector<std::array<double, 3>> raw =
      findOrByPath<std::vector<std::array<double, 3>>>(root, key, {});

  if (raw.empty()) {
    return defaults;
  }

  if (raw.size() != expected_size) {
    if (logger) {
      LOG_WARN(logger, "invalid ", section, " list size for ", key, ": expected ", expected_size,
               ", got ", raw.size(), ", using defaults");
    }
    std::ostringstream message;
    message << "invalid list size: expected " << expected_size << ", got " << raw.size()
            << ", using defaults";
    emitDiagnostic(diagnostics, section, key, "invalid_size", message.str());
    return defaults;
  }

  std::vector<Vec3> parsed;
  parsed.reserve(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto& row = raw[i];
    for (std::size_t axis = 0; axis < row.size(); ++axis) {
      if (row[axis] < min_value || row[axis] > max_value) {
        if (logger) {
          LOG_WARN(logger, "invalid ", section, " value for ", key, "[", i, "][", axis,
                   "] = ", row[axis], ", using defaults");
        }
        std::ostringstream message;
        message << "invalid value for " << key << "[" << i << "][" << axis << "]=" << row[axis]
                << ", using defaults";
        emitDiagnostic(diagnostics, section, key, "out_of_range", message.str());
        return defaults;
      }
    }
    parsed.push_back(Vec3{row[0], row[1], row[2]});
  }
  return parsed;
}

} // namespace config_validation
