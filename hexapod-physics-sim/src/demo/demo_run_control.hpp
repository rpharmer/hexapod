#pragma once

#include <atomic>
#include <chrono>
#include <thread>

namespace minphys3d::demo {

using DemoSteadyClock = std::chrono::steady_clock;

/// Optional cooperative control for long demo runs (interactive mode runs the sim on a worker
/// thread so stdin can still accept commands). When null, demos use the `realtime_playback` bool
/// only. When non-null, pacing and cancellation consult this object each frame.
struct DemoRunControl {
    std::atomic<bool> cancel_requested{false};
    std::atomic<bool> pause_requested{false};
    /// Wall-clock pacing (~60 Hz outer frame); may be toggled while a worker thread is running.
    std::atomic<bool> realtime_pacing{false};
};

inline bool PaceRealtimeEnabled(bool param_realtime, DemoRunControl* control) {
    if (control != nullptr) {
        return control->realtime_pacing.load(std::memory_order_relaxed);
    }
    return param_realtime;
}

inline void CooperativeYield(DemoRunControl* control) {
    if (control == nullptr) {
        return;
    }
    while (control->pause_requested.load(std::memory_order_acquire)
           && !control->cancel_requested.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}

inline bool CooperativeCancelled(DemoRunControl* control) {
    return control != nullptr && control->cancel_requested.load(std::memory_order_relaxed);
}

inline void PaceRealtimeOuterFrame(bool pace, DemoSteadyClock::time_point t0, int frame_index, float dt) {
    if (!pace) {
        return;
    }
    using namespace std::chrono;
    const auto offset = duration_cast<DemoSteadyClock::duration>(
        duration<double>(static_cast<double>(dt) * static_cast<double>(frame_index + 1)));
    std::this_thread::sleep_until(t0 + offset);
}

} // namespace minphys3d::demo
