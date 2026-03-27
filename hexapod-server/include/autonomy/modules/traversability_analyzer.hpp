#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

class TraversabilityAnalyzerModuleShell : public AutonomyModuleStub {
public:
    TraversabilityAnalyzerModuleShell();

    TraversabilityReport analyze(const WorldModelSnapshot& world_model,
                                 uint64_t timestamp_ms);
    [[nodiscard]] TraversabilityReport report() const;

private:
    TraversabilityReport report_{};
};

} // namespace autonomy
