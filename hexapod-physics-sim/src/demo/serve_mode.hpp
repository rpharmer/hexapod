#pragma once

#include "demo/frame_sink.hpp"

#include <cstdint>
#include <string>

namespace minphys3d::demo {

/// UDP serve loop: hexapod scene, step on StepCommand, reply with StateResponse.
/// When `preview_sink` is `Udp`, each stepped frame is also sent as minphys scene UDP (OpenGL visualiser).
/// Blocks until fatal socket error; returns non-zero on failure.
int RunPhysicsServeMode(std::uint16_t listen_port,
                        SinkKind preview_sink = SinkKind::Dummy,
                        const std::string& preview_udp_host = "127.0.0.1",
                        int preview_udp_port = 9870,
                        const std::string& scene_file = "",
                        int preview_emit_stride = 1);

} // namespace minphys3d::demo
