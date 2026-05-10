#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "minphys3d/core/body.hpp"
#include "demo/terrain_patch.hpp"

namespace minphys3d::demo {

enum class SinkKind {
    Dummy,
    Udp,
};

struct TerrainPatchFrameSnapshot {
    bool valid{false};
    int schema_version{1};
    TerrainPatchConfig config{};
    Vec3 center_world{};
    Vec3 grid_origin_world{};
    Real base_height_m{0.0};
    Vec3 last_normal{0.0, 1.0, 0.0};
    Real last_plane_height_m{0.0};
    std::vector<float> heights{};
    std::vector<float> confidences{};
    std::vector<float> collision_heights{};
};

TerrainPatchFrameSnapshot CaptureTerrainPatchFrameSnapshot(const TerrainPatch& terrain_patch);

class FrameSink {
public:
    virtual ~FrameSink() = default;

    virtual void begin_frame(int frame_index, Real sim_time_s) = 0;
    virtual void emit_body(std::uint32_t body_id, const Body& body) = 0;
    void emit_terrain_patch(const TerrainPatch& terrain_patch) {
        emit_terrain_patch_snapshot(CaptureTerrainPatchFrameSnapshot(terrain_patch));
    }
    virtual void emit_terrain_patch_snapshot(const TerrainPatchFrameSnapshot& terrain_patch) = 0;
    virtual void end_frame() = 0;
};

std::unique_ptr<FrameSink> MakeDummySink();
std::unique_ptr<FrameSink> MakeUdpSink(const std::string& host = "127.0.0.1", int port = 9870);

/// Builds dummy or UDP sink; `host` / `port` are ignored when `kind == SinkKind::Dummy`.
std::unique_ptr<FrameSink> MakeFrameSink(SinkKind kind, const std::string& udp_host, int udp_port);

} // namespace minphys3d::demo
