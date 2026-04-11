#pragma once

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

/// Runs the interactive demo for `frame_count` frames. When `realtime_playback` is true, waits ~one
/// frame interval after each frame so UDP/visualizers can follow wall clock; otherwise steps as fast as possible.
/// Use `gravity = {0,0,0}` for a static preview (no weight) while inspecting poses with UDP, etc.
int RunPhysicsDemo(
    SinkKind sink_kind = SinkKind::Dummy,
    SceneModel model = SceneModel::Default,
    int frame_count = 1200,
    bool realtime_playback = false,
    minphys3d::Vec3 gravity = minphys3d::Vec3{0.0f, -9.81f, 0.0f},
    const std::string& udp_host = "127.0.0.1",
    int udp_port = 9870);
} // namespace minphys3d::demo
