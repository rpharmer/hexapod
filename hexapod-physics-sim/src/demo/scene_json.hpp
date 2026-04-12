#pragma once

#include "demo/demo_run_control.hpp"
#include "demo/frame_sink.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/math/vec3.hpp"

#include <string>

namespace minphys3d::demo {

/// Loads `bodies` and optional `joints` from minphys scene JSON (see `assets/scenes/examples/`).
/// `world_out` must be empty. `joints_loaded_out`, when non-null, receives the number of joints created.
/// Schema 1: bodies only. Schema 2: same + documented `joints` array.
bool LoadWorldFromMinphysSceneJson(
    const std::string& json_text,
    minphys3d::World& world_out,
    int& solver_iterations_out,
    std::string& error_out,
    int* joints_loaded_out = nullptr);

/// Runs a JSON-defined scene for `frame_count` steps at 60 Hz, emitting all bodies each frame.
int RunPhysicsDemoFromJsonFile(
    const std::string& scene_path,
    SinkKind sink_kind,
    int frame_count,
    bool realtime_playback,
    minphys3d::Vec3 gravity,
    const std::string& udp_host = "127.0.0.1",
    int udp_port = 9870,
    DemoRunControl* run_control = nullptr);

} // namespace minphys3d::demo
