#pragma once

#include "core/types.hpp"

#include <cstdint>
#include <string>

namespace autonomy {

enum class QosClass {
    Critical,
    Control,
    Status,
    Bulk,
};

enum class StaleDataBehavior {
    Hold,
    SafeStop,
    Degrade,
};

enum class FaultSeverity {
    Info,
    Warn,
    Error,
    Fatal,
};

struct ContractEnvelope {
    std::string contract_version{"v1"};
    std::string frame_id{};
    std::string correlation_id{};
    std::string stream_id{};
    uint64_t sample_id{0};
    TimestampMs timestamp_ms{};
};

struct FaultEvent {
    std::string fault_code{};
    FaultSeverity severity{FaultSeverity::Info};
    TimestampMs timestamp_ms{};
    std::string origin_module{};
    std::string details{};
    std::string correlation_id{};
};

} // namespace autonomy
