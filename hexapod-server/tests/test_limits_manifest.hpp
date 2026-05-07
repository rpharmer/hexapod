#pragma once

#include <cstddef>
#include <optional>
#include <string>

namespace test_limits {

/** argv (--limits-manifest), then HEXAPOD_TEST_LIMITS_MANIFEST, then default file path if it exists. */
bool init(int argc, char** argv, std::string& error_out);

bool loaded();

/** When `profile` is non-empty, merge `suites[suite][name][profile]`; otherwise merge primitive keys from `suites[suite][name]`. */
double getDouble(const std::string& suite,
                 const std::string& name,
                 const std::string& profile,
                 const std::string& key,
                 double fallback);

bool getBool(const std::string& suite,
             const std::string& name,
             const std::string& profile,
             const std::string& key,
             bool fallback);

std::size_t getSizeT(const std::string& suite,
                     const std::string& name,
                     const std::string& profile,
                     const std::string& key,
                     std::size_t fallback);

} // namespace test_limits
