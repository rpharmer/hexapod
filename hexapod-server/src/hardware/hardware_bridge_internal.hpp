#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "hardware_bridge.hpp"
#include "protocol_codec.hpp"

namespace logging {
class AsyncLogger;
}

namespace hardware_bridge::detail {

inline constexpr uint8_t kRequestedCapabilities = CAPABILITY_ANGULAR_FEEDBACK;

const char* bridge_error_to_text(BridgeError error);
const char* bridge_phase_to_text(BridgeFailurePhase phase);
const char* bridge_domain_to_text(BridgeFailureDomain domain);

class BridgeResultMetadataBuilder {
public:
    explicit BridgeResultMetadataBuilder(uint8_t requested_capabilities);

    BridgeCommandResultMetadata build(BridgeError error,
                                      BridgeFailurePhase phase,
                                      BridgeFailureDomain domain,
                                      bool retryable,
                                      uint8_t command_code,
                                      uint8_t negotiated_capabilities) const;

private:
    uint8_t requested_capabilities_;
};

bool decode_scalar_float_payload(const std::vector<uint8_t>& payload, protocol::ScalarFloat& decoded);
std::shared_ptr<logging::AsyncLogger> resolve_logger(const std::shared_ptr<logging::AsyncLogger>& logger);

}  // namespace hardware_bridge::detail
