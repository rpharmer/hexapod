#include "autonomy/contract_enforcer.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testAcceptsCompatibleVersionAndFreshPayload() {
    autonomy::ContractEnforcer enforcer("v1");
    autonomy::ContractEnvelope envelope{};
    envelope.contract_version = "v1.2";
    envelope.frame_id = "map";
    envelope.correlation_id = "corr-1";
    envelope.sample_id = 7;
    envelope.timestamp_ms = 900;

    autonomy::ContractValidationConfig config{};
    config.max_age_ms = 200;

    const auto result = enforcer.validate(envelope, 1000, config, std::optional<uint64_t>{6});
    return expect(result.valid, "compatible version and fresh envelope should pass validation");
}

bool testRejectsVersionAndStaleAge() {
    autonomy::ContractEnforcer enforcer("v1");
    autonomy::ContractEnvelope envelope{};
    envelope.contract_version = "v2.0";
    envelope.frame_id = "map";
    envelope.correlation_id = "corr-2";
    envelope.sample_id = 10;
    envelope.timestamp_ms = 0;

    autonomy::ContractValidationConfig config{};
    config.max_age_ms = 100;

    const auto result = enforcer.validate(envelope, 500, config, std::nullopt);
    return expect(!result.valid, "invalid payload should fail validation") &&
           expect(result.invalid_version, "new major version should be rejected") &&
           expect(result.stale_timestamp, "stale payload should be flagged");
}

bool testRejectsNonMonotonicSampleId() {
    autonomy::ContractEnforcer enforcer("v1");
    autonomy::ContractEnvelope envelope{};
    envelope.contract_version = "v1";
    envelope.frame_id = "odom";
    envelope.correlation_id = "corr-3";
    envelope.sample_id = 3;
    envelope.timestamp_ms = 100;

    const auto result = enforcer.validate(envelope, 150, autonomy::ContractValidationConfig{}, std::optional<uint64_t>{3});
    return expect(!result.valid, "equal sample id should fail when monotonic required") &&
           expect(result.non_monotonic_sample_id, "non monotonic flag should be set");
}

bool testRejectsVersionThatOnlySharesPrefix() {
    autonomy::ContractEnforcer enforcer("v1");
    autonomy::ContractEnvelope envelope{};
    envelope.contract_version = "v10.1";
    envelope.frame_id = "map";
    envelope.correlation_id = "corr-4";
    envelope.sample_id = 11;
    envelope.timestamp_ms = 100;

    const auto result = enforcer.validate(envelope, 150, autonomy::ContractValidationConfig{}, std::nullopt);
    return expect(!result.valid, "v10 should not match expected major v1") &&
           expect(result.invalid_version, "prefix-sharing major should still be rejected");
}

} // namespace

int main() {
    if (!testAcceptsCompatibleVersionAndFreshPayload() ||
        !testRejectsVersionAndStaleAge() ||
        !testRejectsNonMonotonicSampleId() ||
        !testRejectsVersionThatOnlySharesPrefix()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
