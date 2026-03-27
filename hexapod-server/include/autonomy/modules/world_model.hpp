#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

class WorldModelModuleShell : public AutonomyModuleStub {
public:
    WorldModelModuleShell();

    WorldModelSnapshot update(const LocalizationEstimate& localization,
                              const MapSliceInput& map_slice_input,
                              bool blocked,
                              TimestampMs timestamp_ms,
                                ContractEnvelope envelope = {});
    [[nodiscard]] WorldModelSnapshot snapshot() const;

private:
    WorldModelSnapshot snapshot_{};
};

} // namespace autonomy
