#pragma once

#include <array>

#include "gait_policy_planner.hpp"
#include "types.hpp"

struct ContactManagerOutput {
    GaitState managed_gait{};
    RuntimeGaitPolicy managed_policy{};
    std::array<double, kNumLegs> stance_confidence{};
    std::array<bool, kNumLegs> touchdown_detected{};
    std::array<bool, kNumLegs> slip_detected{};
    bool derating_requested{false};
};

class ContactManager {
public:
    ContactManager() = default;

    ContactManagerOutput update(const RobotState& estimated,
                                const GaitState& scheduled_gait,
                                const RuntimeGaitPolicy& policy);

private:
    struct LegContactState {
        bool previous_contact{false};
        bool previous_scheduled_stance{true};
        bool touchdown_latched{false};
        bool degraded{false};
        double stance_confidence{1.0};
        int missed_touchdown_count{0};
        int slip_count{0};
        int stable_contact_cycles{0};
    };

    std::array<LegContactState, kNumLegs> leg_states_{};
};
