#ifndef CONFIG_VALIDATION_HPP
#define CONFIG_VALIDATION_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <memory>

#include <toml.hpp>

#include "types.hpp"
#include "logger.hpp"

namespace config_validation {

int parseIntWithFallback(const toml::value& root,
                         const std::string& key,
                         int default_value,
                         int min_value,
                         int max_value,
                         const std::string& section,
                         std::shared_ptr<logging::AsyncLogger> logger = nullptr);

uint64_t parseU64WithFallback(const toml::value& root,
                              const std::string& key,
                              uint64_t default_value,
                              uint64_t min_value,
                              uint64_t max_value,
                              const std::string& section,
                              std::shared_ptr<logging::AsyncLogger> logger = nullptr);

double parseDoubleWithFallback(const toml::value& root,
                               const std::string& key,
                               double default_value,
                               double min_value,
                               double max_value,
                               const std::string& section,
                               std::shared_ptr<logging::AsyncLogger> logger = nullptr);

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
                                                std::shared_ptr<logging::AsyncLogger> logger = nullptr);

std::vector<Vec3> parseVec3ListWithFallback(const toml::value& root,
                                            const std::string& key,
                                            const std::vector<Vec3>& defaults,
                                            std::size_t expected_size,
                                            double min_value,
                                            double max_value,
                                            const std::string& section,
                                            std::shared_ptr<logging::AsyncLogger> logger = nullptr);

} // namespace config_validation

#endif // CONFIG_VALIDATION_HPP
