#pragma once

// Binary UDP scene preview between hexapod-physics-sim (UdpSink) and hexapod-opengl-visualiser.
// Little-endian wire layout; trivially copyable sub-structs. Not the same as physics_sim_protocol.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

namespace minphys_viz {

inline constexpr char kMagic0 = 'M';
inline constexpr char kMagic1 = 'P';
inline constexpr char kMagic2 = 'V';
inline constexpr char kMagic3 = '1';
inline constexpr std::uint16_t kWireVersion = 1;

/// Matches minphys3d::ShapeType enum numeric order (Sphere=0 .. Compound=6).
enum class VizShapeType : std::uint8_t {
    Sphere = 0,
    Box = 1,
    Plane = 2,
    Capsule = 3,
    Cylinder = 4,
    HalfCylinder = 5,
    Compound = 6,
};

enum class VizMessageKind : std::uint8_t {
    SceneClear = 0,
    EntityStatic = 1,
    EntityFrame = 2,
    TerrainPatchMeta = 3,
    TerrainPatchFloats = 4,
};

inline constexpr std::uint32_t kTerrainFlagHasCollisionHeights = 1u << 0;

#pragma pack(push, 1)

struct VizWireHeader {
    char magic[4]{kMagic0, kMagic1, kMagic2, kMagic3};
    std::uint16_t wire_version{kWireVersion};
    std::uint8_t message_kind{0};
    std::uint8_t reserved0{0};
};

struct VizSceneClearBody {
    std::uint32_t frame_index{0};
};

struct VizVec3f {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct VizQuatf {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct VizCompoundChildWire {
    VizShapeType shape{VizShapeType::Box};
    std::uint8_t pad0{0};
    std::uint8_t pad1{0};
    std::uint8_t pad2{0};
    VizVec3f local_position{};
    VizQuatf local_orientation{};
    float radius{0.5f};
    float half_height{0.5f};
    VizVec3f half_extents{0.5f, 0.5f, 0.5f};
};

struct VizEntityStaticBody {
    std::uint32_t entity_id{0};
    VizShapeType shape{VizShapeType::Sphere};
    std::uint8_t pad0{0};
    std::uint8_t pad1{0};
    std::uint8_t pad2{0};
    float static_friction{0.0f};
    float dynamic_friction{0.0f};
    float restitution{0.0f};
    float radius{0.0f};
    float half_height{0.0f};
    VizVec3f half_extents{};
    VizVec3f plane_normal{0.0f, 1.0f, 0.0f};
    float plane_offset{0.0f};
    std::uint32_t compound_child_count{0};
    // Followed by compound_child_count * VizCompoundChildWire (variable-length tail).
};

struct VizEntityFrameBody {
    std::uint32_t entity_id{0};
    std::int32_t frame{0};
    float sim_time_s{0.0f};
    VizVec3f position{};
    VizQuatf rotation{};
};

struct VizTerrainPatchMetaBody {
    std::uint32_t terrain_seq{0};
    std::int32_t frame{0};
    float sim_time_s{0.0f};
    std::int32_t rows{0};
    std::int32_t cols{0};
    std::uint32_t flags{0};
    float cell_size_m{0.05f};
    float base_margin_m{0.08f};
    float min_cell_thickness_m{0.012f};
    float influence_sigma_m{0.12f};
    float plane_confidence{0.12f};
    float confidence_half_life_s{1.5f};
    float base_update_blend{0.35f};
    float decay_update_boost{0.45f};
    std::uint8_t use_sample_binning{0};
    std::uint8_t use_conservative_collision{0};
    std::uint8_t scroll_world_fixed{0};
    std::uint8_t lidar_fusion_enable{0};
    float sample_bin_size_m{0.25f};
    std::int32_t lidar_sample_stride{4};
    float lidar_sample_weight{0.18f};
    float lidar_min_surface_confidence{0.10f};
    float lidar_contact_arbitration_radius_m{0.10f};
    float lidar_contact_disagreement_m{0.05f};
    VizVec3f center_world{};
    float grid_origin_x{0.0f};
    float grid_origin_z{0.0f};
    float base_height_m{0.0f};
    float plane_height_m{0.0f};
    VizVec3f plane_normal{0.0f, 1.0f, 0.0f};
    /// Total bytes receiver should accumulate for TerrainPatchFloats payloads:
    /// rows*cols*4*(2 + collision_layer?1:0)
    std::uint32_t expected_float_blob_bytes{0};
};

struct VizTerrainPatchFloatsHeader {
    std::uint32_t terrain_seq{0};
    std::uint32_t byte_offset{0};
    std::uint16_t chunk_bytes{0};
    std::uint16_t reserved0{0};
    // Followed by chunk_bytes of raw data (multiple of 4).
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<VizWireHeader>);
static_assert(std::is_trivially_copyable_v<VizSceneClearBody>);
static_assert(std::is_trivially_copyable_v<VizEntityFrameBody>);
static_assert(std::is_trivially_copyable_v<VizTerrainPatchMetaBody>);
static_assert(std::is_trivially_copyable_v<VizTerrainPatchFloatsHeader>);
static_assert(std::is_trivially_copyable_v<VizCompoundChildWire>);
static_assert(std::is_trivially_copyable_v<VizEntityStaticBody>);

inline constexpr std::size_t kMaxVizUdpPayload = 60000;

inline bool IsVizBinaryPayload(const void* data, std::size_t len) {
    if (len < sizeof(VizWireHeader)) {
        return false;
    }
    const auto* h = static_cast<const VizWireHeader*>(data);
    return h->magic[0] == kMagic0 && h->magic[1] == kMagic1 && h->magic[2] == kMagic2 && h->magic[3] == kMagic3
        && h->wire_version == kWireVersion;
}

inline void AppendHeader(std::vector<std::uint8_t>& out, VizMessageKind kind) {
    VizWireHeader hdr{};
    hdr.magic[0] = kMagic0;
    hdr.magic[1] = kMagic1;
    hdr.magic[2] = kMagic2;
    hdr.magic[3] = kMagic3;
    hdr.wire_version = kWireVersion;
    hdr.message_kind = static_cast<std::uint8_t>(kind);
    hdr.reserved0 = 0;
    const auto* p = reinterpret_cast<const std::uint8_t*>(&hdr);
    out.insert(out.end(), p, p + sizeof(hdr));
}

inline void AppendBytes(std::vector<std::uint8_t>& out, const void* src, std::size_t n) {
    const auto* p = static_cast<const std::uint8_t*>(src);
    out.insert(out.end(), p, p + n);
}

inline void EncodeSceneClear(std::vector<std::uint8_t>& out, std::uint32_t frame_index) {
    out.clear();
    AppendHeader(out, VizMessageKind::SceneClear);
    VizSceneClearBody body{};
    body.frame_index = frame_index;
    AppendBytes(out, &body, sizeof(body));
}

inline void EncodeEntityFrame(
    std::vector<std::uint8_t>& out,
    std::uint32_t entity_id,
    std::int32_t frame,
    float sim_time_s,
    float px,
    float py,
    float pz,
    float qw,
    float qx,
    float qy,
    float qz) {
    out.clear();
    AppendHeader(out, VizMessageKind::EntityFrame);
    VizEntityFrameBody body{};
    body.entity_id = entity_id;
    body.frame = frame;
    body.sim_time_s = sim_time_s;
    body.position = {px, py, pz};
    body.rotation = {qw, qx, qy, qz};
    AppendBytes(out, &body, sizeof(body));
}

inline void EncodeEntityStatic(std::vector<std::uint8_t>& out,
                               const VizEntityStaticBody& fixed,
                               const VizCompoundChildWire* children,
                               std::size_t child_count) {
    out.clear();
    AppendHeader(out, VizMessageKind::EntityStatic);
    AppendBytes(out, &fixed, sizeof(fixed));
    if (child_count > 0 && children != nullptr) {
        AppendBytes(out, children, child_count * sizeof(VizCompoundChildWire));
    }
}

inline void EncodeTerrainMeta(std::vector<std::uint8_t>& out, const VizTerrainPatchMetaBody& meta) {
    out.clear();
    AppendHeader(out, VizMessageKind::TerrainPatchMeta);
    AppendBytes(out, &meta, sizeof(meta));
}

/// Max float bytes per TerrainPatchFloats datagram (after headers); stable for reserve hints.
inline std::uint32_t TerrainFloatChunkMaxDataBytes() {
    const std::uint32_t header_and_max_data =
        static_cast<std::uint32_t>(sizeof(VizWireHeader) + sizeof(VizTerrainPatchFloatsHeader));
    std::uint32_t room = kMaxVizUdpPayload > header_and_max_data
        ? static_cast<std::uint32_t>(kMaxVizUdpPayload) - header_and_max_data
        : 0u;
    room -= room % 4u;
    return room;
}

/// Upper bound on chunk count for `total_bytes` (aligned float blob).
inline std::size_t TerrainFloatChunkCountUpperBound(std::uint32_t total_bytes) {
    const std::uint32_t room = TerrainFloatChunkMaxDataBytes();
    if (room < 4u || total_bytes == 0 || (total_bytes % 4u) != 0u) {
        return 0;
    }
    return static_cast<std::size_t>((static_cast<std::uint64_t>(total_bytes) + room - 1) / room);
}

/// Split contiguous float blob into multiple datagrams; each outs[i] is one UDP payload for i in [0, return).
/// When shrink_tail is true (default), clears outs first and resizes to the chunk count.
/// When shrink_tail is false, reuses existing outs[i] capacity from prior calls; only outs[0..return) are valid.
inline std::size_t EncodeTerrainFloatChunks(std::vector<std::vector<std::uint8_t>>& outs,
                                            std::uint32_t terrain_seq,
                                            const void* blob,
                                            std::uint32_t total_bytes,
                                            bool shrink_tail = true) {
    if (shrink_tail) {
        outs.clear();
    }
    if (total_bytes == 0 || (total_bytes % 4u) != 0u) {
        if (shrink_tail) {
            outs.clear();
        }
        return 0;
    }
    const auto* base = static_cast<const std::uint8_t*>(blob);
    std::uint32_t offset = 0;
    std::size_t idx = 0;
    while (offset < total_bytes) {
        const std::uint32_t room = TerrainFloatChunkMaxDataBytes();
        if (room < 4u) {
            break;
        }
        const std::uint32_t remain = total_bytes - offset;
        const std::uint16_t chunk = static_cast<std::uint16_t>(std::min<std::uint32_t>(remain, room));
        if (chunk == 0) {
            break;
        }
        if (idx >= outs.size()) {
            outs.emplace_back();
        }
        std::vector<std::uint8_t>& packet = outs[idx];
        packet.clear();
        AppendHeader(packet, VizMessageKind::TerrainPatchFloats);
        VizTerrainPatchFloatsHeader fh{};
        fh.terrain_seq = terrain_seq;
        fh.byte_offset = offset;
        fh.chunk_bytes = chunk;
        fh.reserved0 = 0;
        AppendBytes(packet, &fh, sizeof(fh));
        AppendBytes(packet, base + offset, chunk);
        offset += chunk;
        ++idx;
    }
    if (shrink_tail) {
        outs.resize(idx);
    }
    return idx;
}

} // namespace minphys_viz
