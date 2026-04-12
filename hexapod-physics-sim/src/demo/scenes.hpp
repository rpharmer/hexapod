#pragma once

#include "demo/demo_run_control.hpp"
#include "demo/frame_sink.hpp"
#include "minphys3d/math/vec3.hpp"

#include <string>

namespace minphys3d::demo {

enum class SceneModel {
    Default,
    Hexapod,
};

int RunDefaultScene(SinkKind sink_kind = SinkKind::Dummy, SceneModel model = SceneModel::Default);
int RunRealTimeDefaultScene(SinkKind sink_kind = SinkKind::Dummy, SceneModel model = SceneModel::Default);

/// Runs the interactive demo for `frame_count` frames. When pacing is on (see below), waits until the
/// wall-clock deadline for each outer frame so UDP/visualizers can follow ~60 Hz sim time; otherwise
/// steps as fast as possible. Use `gravity = {0,0,0}` for a static preview (no weight) while inspecting
/// poses with UDP, etc.
///
/// When `run_control` is non-null, pacing uses `run_control->realtime_pacing` each frame (so it can be
/// toggled during cooperative runs) and the loop honors `cancel_requested` / `pause_requested`.
/// When `run_control` is null, pacing follows `realtime_playback` only.
int RunPhysicsDemo(
    SinkKind sink_kind = SinkKind::Dummy,
    SceneModel model = SceneModel::Default,
    int frame_count = 1200,
    bool realtime_playback = false,
    minphys3d::Vec3 gravity = minphys3d::Vec3{0.0f, -9.81f, 0.0f},
    const std::string& udp_host = "127.0.0.1",
    int udp_port = 9870,
    DemoRunControl* run_control = nullptr);
} // namespace minphys3d::demo
