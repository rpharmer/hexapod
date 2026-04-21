#pragma once

#include "demo/demo_run_control.hpp"
#include "process_resource_monitoring.hpp"

#include <chrono>
#include <cstdio>

namespace minphys3d::demo {

struct ProcessResourceDiagnosticsState {
    resource_monitoring::ProcessResourceSampler sampler{};
    DemoSteadyClock::time_point next_emit_at{};
    std::chrono::milliseconds emit_interval{std::chrono::milliseconds{1000}};
};

inline void MaybeLogProcessResourceSnapshot(std::FILE* out,
                                            ProcessResourceDiagnosticsState& state,
                                            const char* prefix = "[resource]") {
    if (out == nullptr) {
        return;
    }

    const DemoSteadyClock::time_point now = DemoSteadyClock::now();
    if (state.next_emit_at.time_since_epoch().count() != 0 && now < state.next_emit_at) {
        return;
    }

    const resource_monitoring::ProcessResourceSnapshot snapshot = state.sampler.sample();
    std::fprintf(out, "%s process_resource=%s\n", prefix, resource_monitoring::formatProcessResourceSnapshot(snapshot).c_str());
    std::fflush(out);
    state.next_emit_at = now + state.emit_interval;
}

} // namespace minphys3d::demo
