#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

#include <cstdint>

namespace autonomy {

class LocalPlannerModuleShell : public AutonomyModuleStub {
public:
    LocalPlannerModuleShell();

    LocalPlan plan(const GlobalPlan& global_plan,
                   bool blocked,
                   uint64_t now_ms,
                   ContractEnvelope envelope = {});
    [[nodiscard]] LocalPlan currentPlan() const;

private:
    LocalPlan plan_{};
};

} // namespace autonomy
