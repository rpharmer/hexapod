#pragma once

#include <cstdint>
#include <string>

namespace autonomy {

enum class RecoveryAction {
    None,
    Hold,
    Retry,
    Replan,
    Abort,
};

struct RecoveryDecision {
    RecoveryAction action{RecoveryAction::None};
    bool recovery_active{false};
    bool mission_should_abort{false};
    std::string reason{};
};

class RecoveryManager {
public:
    explicit RecoveryManager(uint64_t retry_budget);

    void reset();
    RecoveryDecision onNoProgress();

private:
    uint64_t retry_budget_{0};
    uint64_t attempts_{0};
};

} // namespace autonomy
