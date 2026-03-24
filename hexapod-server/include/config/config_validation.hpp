#ifndef CONFIG_VALIDATION_HPP
#define CONFIG_VALIDATION_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <toml.hpp>

#include "logger.hpp"
#include "types.hpp"

namespace config_validation {

struct ConfigDiagnostic
{
  std::string section;
  std::string key;
  std::string errorCode;
  std::string message;
};

void emitDiagnostic(std::vector<ConfigDiagnostic>* diagnostics,
                    const std::string& section,
                    const std::string& key,
                    const std::string& error_code,
                    const std::string& message);

int parseIntWithFallback(const toml::value& root,
                         const std::string& key,
                         int default_value,
                         int min_value,
                         int max_value,
                         const std::string& section,
                         std::shared_ptr<logging::AsyncLogger> logger = nullptr,
                         std::vector<ConfigDiagnostic>* diagnostics = nullptr);

uint64_t parseU64WithFallback(const toml::value& root,
                              const std::string& key,
                              uint64_t default_value,
                              uint64_t min_value,
                              uint64_t max_value,
                              const std::string& section,
                              std::shared_ptr<logging::AsyncLogger> logger = nullptr,
                              std::vector<ConfigDiagnostic>* diagnostics = nullptr);

double parseDoubleWithFallback(const toml::value& root,
                               const std::string& key,
                               double default_value,
                               double min_value,
                               double max_value,
                               const std::string& section,
                               std::shared_ptr<logging::AsyncLogger> logger = nullptr,
                               std::vector<ConfigDiagnostic>* diagnostics = nullptr);

bool parseBoolWithFallback(const toml::value& root,
                           const std::string& key,
                           bool default_value);

std::vector<double> parseDoubleListWithFallback(const toml::value& root,
                                                const std::string& key,
                                                const std::vector<double>& defaults,
                                                std::size_t expected_size,
                                                double min_value,
                                                double max_value,
                                                const std::string& section,
                                                std::shared_ptr<logging::AsyncLogger> logger = nullptr,
                                                std::vector<ConfigDiagnostic>* diagnostics = nullptr);

std::vector<Vec3> parseVec3ListWithFallback(const toml::value& root,
                                            const std::string& key,
                                            const std::vector<Vec3>& defaults,
                                            std::size_t expected_size,
                                            double min_value,
                                            double max_value,
                                            const std::string& section,
                                            std::shared_ptr<logging::AsyncLogger> logger = nullptr,
                                            std::vector<ConfigDiagnostic>* diagnostics = nullptr);

} // namespace config_validation

#endif // CONFIG_VALIDATION_HPP
