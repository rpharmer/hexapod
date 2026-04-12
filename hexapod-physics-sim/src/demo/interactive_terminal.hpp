#pragma once

#include "demo/scenes.hpp"
#include "minphys3d/math/vec3.hpp"

#include <string>

namespace minphys3d::demo {

/// Starting point for an interactive session (CLI flags can pre-fill before the REPL runs).
struct InteractiveSessionSeed {
    SinkKind sink_kind = SinkKind::Dummy;
    SceneModel scene_model = SceneModel::Default;
    int frame_count = 1200;
    bool realtime_playback = false;
    minphys3d::Vec3 gravity = minphys3d::Vec3{0.0f, -9.81f, 0.0f};
    /// When non-empty, `r` runs `RunPhysicsDemoFromJsonFile` instead of built-in demos.
    std::string scene_json_path;
    std::string udp_host = "127.0.0.1";
    int udp_port = 9870;
    /// After each full (non-cancelled) background `r` completes, advance to the next row in the preset catalog.
    bool autonext_preset = false;
    /// After autonext applies the next preset, start another background `r` automatically (implies cycling when on).
    bool autonext_auto_run = false;
};

/**
 * Terminal REPL: pick built-in scenario (1/2) or JSON file (`l path`), UDP target (`u host port`),
 * gravity / realtime / frames, optional `preset` bundles, then `r` to run once.
 */
int RunInteractiveTerminalSession(const InteractiveSessionSeed& seed = {});

} // namespace minphys3d::demo
