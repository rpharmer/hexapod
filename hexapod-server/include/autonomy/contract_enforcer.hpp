#pragma once

#include "autonomy/common_types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace autonomy {

struct ContractValidationConfig {
    uint64_t max_age_ms{500};
    bool require_frame_id{true};
    bool require_correlation_id{true};
    bool require_nonzero_sample_id{true};
    bool enforce_monotonic_sample_id{true};
};

struct ContractValidationResult {
    bool valid{false};
    bool invalid_version{false};
    bool missing_frame_id{false};
    bool missing_correlation_id{false};
    bool missing_sample_id{false};
    bool non_monotonic_sample_id{false};
    bool stale_timestamp{false};
    uint64_t age_ms{0};
    std::string message{};
};

class ContractEnforcer {
public:
    explicit ContractEnforcer(std::string expected_major = "v1");

    ContractValidationResult validate(const ContractEnvelope& envelope,
                                      uint64_t now_ms,
                                      const ContractValidationConfig& config,
                                      std::optional<uint64_t> last_sample_id) const;

    bool isCompatibleMajorVersion(const std::string& contract_version) const;

private:
    std::string expected_major_;
};

} // namespace autonomy
