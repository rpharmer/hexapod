#pragma once

#include "autonomy/mission_types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace autonomy {

enum class NavigationStatus {
    Idle,
    Active,
    Complete,
    Blocked,
};

struct NavigationIntent {
    std::string mission_id{};
    uint64_t waypoint_index{0};
    Waypoint target{};
};

struct NavigationUpdate {
    bool has_intent{false};
    NavigationIntent intent{};
    NavigationStatus status{NavigationStatus::Idle};
    std::string reason{};
};

} // namespace autonomy
