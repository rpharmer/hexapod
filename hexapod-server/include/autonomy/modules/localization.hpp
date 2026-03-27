#pragma once

#include "autonomy/module_stubs.hpp"
#include "autonomy/common_types.hpp"
#include "autonomy/modules/module_data.hpp"
#include "autonomy/navigation_types.hpp"

namespace autonomy {

class LocalizationModuleShell : public AutonomyModuleStub {
public:
    LocalizationModuleShell();

    LocalizationEstimate update(const NavigationUpdate& navigation_update,
                                uint64_t timestamp_ms,
                                ContractEnvelope envelope = {});
    [[nodiscard]] LocalizationEstimate estimate() const;

private:
    LocalizationEstimate estimate_{};
};

} // namespace autonomy
