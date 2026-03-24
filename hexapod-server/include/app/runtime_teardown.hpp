#pragma once

#include <functional>
#include <memory>

#include "logger.hpp"

namespace app {

bool runRuntimeTeardown(const std::shared_ptr<logging::AsyncLogger>& logger,
                        const std::function<void()>& stop_fn,
                        int runner_rc);

} // namespace app
