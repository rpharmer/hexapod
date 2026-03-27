#include "autonomy/module_stubs.hpp"

#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool testLifecycleTransitions() {
    autonomy::MissionExecutiveModule module;

    if (!expect(module.state() == autonomy::ModuleRunState::Stopped,
                "module should start in stopped state")) {
        return false;
    }
    if (!expect(module.init(), "init should succeed from stopped")) {
        return false;
    }
    if (!expect(module.start(), "start should succeed from initialized")) {
        return false;
    }
    if (!expect(module.step(321), "step should succeed while running")) {
        return false;
    }

    const auto health = module.health();
    if (!expect(health.heartbeat_timestamp_ms == TimestampMs{321},
                "step should update heartbeat timestamp")) {
        return false;
    }

    module.stop();
    return expect(module.state() == autonomy::ModuleRunState::Stopped,
                  "stop should transition module to stopped");
}

bool testStepFailsWhenNotRunning() {
    autonomy::NavigationManagerModule module;
    const bool stepped = module.step(10);
    const auto health = module.health();

    return expect(!stepped, "step should fail when module is not running") &&
           expect(!health.healthy, "health should be unhealthy after invalid step call");
}

} // namespace

int main() {
    if (!testLifecycleTransitions() ||
        !testStepFailsWhenNotRunning()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
