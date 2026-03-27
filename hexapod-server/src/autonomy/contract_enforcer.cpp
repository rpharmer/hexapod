#include "autonomy/contract_enforcer.hpp"

#include <cctype>
#include <sstream>
#include <string_view>

namespace autonomy {

namespace {

bool isExactMajorOrMinor(std::string_view actual, std::string_view expected_major) {
    if (actual == expected_major) {
        return true;
    }
    if (actual.size() <= expected_major.size() || actual.substr(0, expected_major.size()) != expected_major) {
        return false;
    }
    if (actual[expected_major.size()] != '.') {
        return false;
    }

    const auto minor_part = actual.substr(expected_major.size() + 1);
    if (minor_part.empty()) {
        return false;
    }
    for (const char c : minor_part) {
        if (!std::isdigit(static_cast<unsigned char>(c))) {
            return false;
        }
    }
    return true;
}

} // namespace

ContractEnforcer::ContractEnforcer(std::string expected_major)
    : expected_major_(std::move(expected_major)) {}

ContractValidationResult ContractEnforcer::validate(const ContractEnvelope& envelope,
                                                    uint64_t now_ms,
                                                    const ContractValidationConfig& config,
                                                    std::optional<uint64_t> last_sample_id) const {
    ContractValidationResult result{};
    result.valid = true;

    if (!isCompatibleMajorVersion(envelope.contract_version)) {
        result.valid = false;
        result.invalid_version = true;
    }

    if (config.require_frame_id && envelope.frame_id.empty()) {
        result.valid = false;
        result.missing_frame_id = true;
    }

    if (config.require_correlation_id && envelope.correlation_id.empty()) {
        result.valid = false;
        result.missing_correlation_id = true;
    }

    if (config.require_nonzero_sample_id && envelope.sample_id == 0) {
        result.valid = false;
        result.missing_sample_id = true;
    }

    if (config.enforce_monotonic_sample_id && last_sample_id.has_value() && envelope.sample_id <= *last_sample_id) {
        result.valid = false;
        result.non_monotonic_sample_id = true;
    }

    if (envelope.timestamp_ms > now_ms) {
        result.valid = false;
        result.stale_timestamp = true;
        result.age_ms = 0;
    } else {
        result.age_ms = now_ms - envelope.timestamp_ms;
        if (result.age_ms > config.max_age_ms) {
            result.valid = false;
            result.stale_timestamp = true;
        }
    }

    if (!result.valid) {
        std::ostringstream oss;
        oss << "contract validation failed";
        if (result.invalid_version) {
            oss << " invalid_version";
        }
        if (result.missing_frame_id) {
            oss << " missing_frame_id";
        }
        if (result.missing_correlation_id) {
            oss << " missing_correlation_id";
        }
        if (result.missing_sample_id) {
            oss << " missing_sample_id";
        }
        if (result.non_monotonic_sample_id) {
            oss << " non_monotonic_sample_id";
        }
        if (result.stale_timestamp) {
            oss << " stale_timestamp";
        }
        result.message = oss.str();
    }

    return result;
}

bool ContractEnforcer::isCompatibleMajorVersion(const std::string& contract_version) const {
    return isExactMajorOrMinor(contract_version, expected_major_);
}

} // namespace autonomy
