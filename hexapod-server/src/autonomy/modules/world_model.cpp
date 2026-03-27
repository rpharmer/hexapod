#include "autonomy/modules/world_model.hpp"

namespace autonomy {

WorldModelModuleShell::WorldModelModuleShell()
    : AutonomyModuleStub("world_model") {}

WorldModelSnapshot WorldModelModuleShell::update(const LocalizationEstimate& localization,
                                                 bool blocked,
                                                 uint64_t timestamp_ms,
                                                 ContractEnvelope envelope) {
    (void)envelope;
    snapshot_.has_map = localization.valid;
    snapshot_.obstacle_risk = blocked ? 1.0 : 0.1;
    snapshot_.timestamp_ms = timestamp_ms;
    return snapshot_;
}

WorldModelSnapshot WorldModelModuleShell::snapshot() const {
    return snapshot_;
}

} // namespace autonomy
