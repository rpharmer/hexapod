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
    void rebuildPlannerBands();

    std::string map_frame_expected_{"map"};
    std::string localization_map_frame_{"map"};
    std::string localization_odom_frame_{"odom"};
    uint64_t stale_map_threshold_ms_{500};
    TimestampMs last_map_timestamp_ms_{};
    WorldModelSnapshot snapshot_{};
};

} // namespace autonomy
