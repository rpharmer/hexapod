#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/modules/module_data.hpp"

namespace autonomy {

class WorldModelModuleShell : public AutonomyModuleStub {
public:
    WorldModelModuleShell();

    WorldModelSnapshot update(const LocalizationEstimate& localization,
                              bool blocked,
                              uint64_t timestamp_ms);
    [[nodiscard]] WorldModelSnapshot snapshot() const;

private:
    WorldModelSnapshot snapshot_{};
};

} // namespace autonomy
