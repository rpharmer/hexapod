#pragma once

#include <cstdint>

struct RuntimeStageToggles {
    bool limiter{true};
    bool gait{true};
    bool body{true};
    bool ik{true};
};

struct RuntimeStageApprovalState {
    RuntimeStageToggles toggles{};
    uint8_t current_enabled_stage_index{3};
    uint8_t max_stage_index{3};
    bool approval_required_for_next_stage{false};
};
