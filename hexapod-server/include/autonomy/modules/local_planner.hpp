#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

#include <cstdint>

namespace autonomy {

struct LocalPlannerPolicyConfig {
    uint64_t stale_plan_timeout_ms{1000};
    double execution_horizon_m{1.0};
    double max_feasible_step_m{1.25};
    double fallback_blend_gain{0.5};
};

class LocalPlannerModuleShell : public AutonomyModuleStub {
public:
    explicit LocalPlannerModuleShell(LocalPlannerPolicyConfig policy = {});

    LocalPlan plan(const GlobalPlan& global_plan,
                   bool blocked,
                   uint64_t now_ms,
                   ContractEnvelope envelope = {});
    [[nodiscard]] LocalPlan currentPlan() const;

private:
    [[nodiscard]] Waypoint selectShortHorizonTarget(const GlobalPlan& global_plan) const;
    [[nodiscard]] bool trajectoryFeasible(const GlobalPlan& global_plan) const;
    [[nodiscard]] Waypoint blendFallbackTarget(const Waypoint& stale_target,
                                               const Waypoint& fallback_target) const;

    LocalPlannerPolicyConfig policy_{};
    LocalPlan plan_{};
    Waypoint last_executable_target_{};
    bool last_executable_target_valid_{false};
};

} // namespace autonomy
