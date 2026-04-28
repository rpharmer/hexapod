#include "demo/frame_sink.hpp"

#include "minphys_viz_protocol.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>
#endif

#include <cstring>

namespace minphys3d::demo {
namespace {

minphys_viz::VizShapeType ToVizShapeType(ShapeType shape) {
    return static_cast<minphys_viz::VizShapeType>(static_cast<std::uint8_t>(shape));
}

minphys_viz::VizCompoundChildWire PackCompoundChild(const CompoundChild& c) {
    minphys_viz::VizCompoundChildWire w{};
    w.shape = ToVizShapeType(c.shape);
    w.local_position = {c.localPosition.x, c.localPosition.y, c.localPosition.z};
    w.local_orientation = {c.localOrientation.w, c.localOrientation.x, c.localOrientation.y, c.localOrientation.z};
    w.radius = c.radius;
    w.half_height = c.halfHeight;
    w.half_extents = {c.halfExtents.x, c.halfExtents.y, c.halfExtents.z};
    return w;
}

class DummySink final : public FrameSink {
public:
    void begin_frame(int, float) override {}
    void emit_body(std::uint32_t, const Body&) override {}
    void emit_terrain_patch(const TerrainPatch&) override {}
    void end_frame() override {}
};

#ifndef _WIN32
class UdpSink final : public FrameSink {
public:
    explicit UdpSink(std::string host, int port) : host_(std::move(host)), port_(port) {
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "[udp_sink] failed to create UDP socket; sink disabled\n";
            return;
        }

        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(static_cast<std::uint16_t>(port_));
        if (::inet_pton(AF_INET, host_.c_str(), &dest_addr_.sin_addr) != 1) {
            std::cerr << "[udp_sink] invalid host " << host_ << "; sink disabled\n";
            return;
        }

        valid_ = true;
    }

    ~UdpSink() override {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
        }
    }

    void begin_frame(int frame_index, float sim_time_s) override {
        frame_index_ = frame_index;
        sim_time_s_ = sim_time_s;
        bodies_.clear();
        if (frame_index == 0 && valid_) {
            scene_clear_packet_.clear();
            minphys_viz::EncodeSceneClear(scene_clear_packet_, static_cast<std::uint32_t>(frame_index));
            send_bytes(scene_clear_packet_);
        }
    }

    void emit_body(std::uint32_t body_id, const Body& body) override {
        bodies_.push_back({body_id, body});
    }

    void emit_terrain_patch(const TerrainPatch& terrain_patch) override {
        viz_terrain_snapshot_.valid = terrain_patch.initialized();
        if (!viz_terrain_snapshot_.valid) {
            viz_terrain_snapshot_ = {};
            return;
        }

        viz_terrain_snapshot_.config = terrain_patch.config();
        viz_terrain_snapshot_.center_world = terrain_patch.center_world();
        viz_terrain_snapshot_.grid_origin_world = terrain_patch.grid_origin_world();
        viz_terrain_snapshot_.base_height_m = terrain_patch.base_height_m();
        viz_terrain_snapshot_.last_normal = terrain_patch.last_normal();
        viz_terrain_snapshot_.last_plane_height_m = terrain_patch.last_plane_height_m();
        viz_terrain_snapshot_.heights = terrain_patch.surface_heights_m();
        viz_terrain_snapshot_.confidences = terrain_patch.confidences();
        if (terrain_patch.has_collision_layer()) {
            viz_terrain_snapshot_.collision_heights = terrain_patch.collision_heights_m();
            viz_terrain_snapshot_.schema_version = 2;
        } else {
            viz_terrain_snapshot_.collision_heights.clear();
            viz_terrain_snapshot_.schema_version = 1;
        }
    }

    void end_frame() override {
        if (!valid_) {
            return;
        }

        reset_packet_pool();

        std::size_t terrain_chunk_ub = 0;
        if (viz_terrain_snapshot_.valid) {
            const auto& snap = viz_terrain_snapshot_;
            const int rows = snap.config.rows;
            const int cols = snap.config.cols;
            const std::size_t ncell = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
            if (rows > 0 && cols > 0 && snap.heights.size() == ncell && snap.confidences.size() == ncell) {
                const bool has_collision =
                    snap.schema_version >= 2 && !snap.collision_heights.empty() && snap.collision_heights.size() == ncell;
                const std::uint32_t layers = has_collision ? 3u : 2u;
                const std::uint32_t expected_bytes =
                    static_cast<std::uint32_t>(ncell * sizeof(float) * layers);
                if (expected_bytes != 0u) {
                    terrain_chunk_ub = minphys_viz::TerrainFloatChunkCountUpperBound(expected_bytes);
                }
            }
        }

        const std::size_t packet_upper_bound = bodies_.size() * 2 + 1 + terrain_chunk_ub;
        packet_send_ptrs_.reserve(packet_upper_bound);
        // packet_send_ptrs_ stores pointers into packet_pool_. Ensure pool growth does not
        // reallocate mid-frame and invalidate earlier pointers.
        if (packet_pool_.capacity() < packet_upper_bound) {
            packet_pool_.reserve(packet_upper_bound);
        }

        for (const auto& snapshot : bodies_) {
            const BodyStaticDescriptor descriptor = MakeStaticDescriptor(snapshot.body);
            const auto found = last_static_descriptors_.find(snapshot.id);
            if (found == last_static_descriptors_.end() || !AreDescriptorsEqual(found->second, descriptor)) {
                append_entity_static(snapshot.id, descriptor);
                last_static_descriptors_[snapshot.id] = descriptor;
            }

            append_entity_frame(snapshot.id, snapshot.body);
        }

        if (viz_terrain_snapshot_.valid) {
            AppendTerrainPackets(viz_terrain_snapshot_);
        }

        flush_frame_payloads();
    }

private:
    struct BodyStaticDescriptor {
        ShapeType shape = ShapeType::Sphere;
        float radius = 0.0f;
        float half_height = 0.0f;
        minphys3d::Vec3 half_extents{};
        std::vector<CompoundChild> compound_children{};
        minphys3d::Vec3 plane_normal{};
        float plane_offset = 0.0f;
        float static_friction = 0.0f;
        float dynamic_friction = 0.0f;
        float restitution = 0.0f;
    };

    struct BodySnapshot {
        std::uint32_t id = 0;
        Body body{};
    };

    struct VizTerrainSnapshot {
        bool valid = false;
        int schema_version = 1;
        TerrainPatchConfig config{};
        Vec3 center_world{};
        Vec3 grid_origin_world{};
        float base_height_m = 0.0f;
        Vec3 last_normal{0.0f, 1.0f, 0.0f};
        float last_plane_height_m = 0.0f;
        std::vector<float> heights{};
        std::vector<float> confidences{};
        std::vector<float> collision_heights{};
    };

    void send_bytes(const std::vector<std::uint8_t>& payload) const {
        if (payload.empty()) {
            return;
        }
        (void)::sendto(
            socket_fd_,
            payload.data(),
            payload.size(),
            0,
            reinterpret_cast<const sockaddr*>(&dest_addr_),
            sizeof(dest_addr_));
    }

    void flush_frame_payloads() const {
        if (packet_send_ptrs_.empty()) {
            return;
        }
#if defined(__linux__)
        const std::size_t count = packet_send_ptrs_.size();
        iovec_scratch_.resize(count);
        msg_scratch_.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            const std::vector<std::uint8_t>& payload = *packet_send_ptrs_[i];
            iovec_scratch_[i].iov_base = const_cast<std::uint8_t*>(payload.data());
            iovec_scratch_[i].iov_len = payload.size();
            std::memset(&msg_scratch_[i], 0, sizeof(mmsghdr));
            msg_scratch_[i].msg_hdr.msg_name =
                const_cast<void*>(static_cast<const void*>(&dest_addr_));
            msg_scratch_[i].msg_hdr.msg_namelen = sizeof(dest_addr_);
            msg_scratch_[i].msg_hdr.msg_iov = &iovec_scratch_[i];
            msg_scratch_[i].msg_hdr.msg_iovlen = 1;
        }
        std::size_t sent = 0;
        while (sent < count) {
            const int rc = ::sendmmsg(
                socket_fd_,
                msg_scratch_.data() + sent,
                static_cast<unsigned int>(count - sent),
                0);
            if (rc < 0) {
                break;
            }
            sent += static_cast<std::size_t>(rc);
        }
#else
        for (const std::vector<std::uint8_t>* payload_ptr : packet_send_ptrs_) {
            send_bytes(*payload_ptr);
        }
#endif
    }

    void reset_packet_pool() {
        pool_next_ = 0;
        packet_send_ptrs_.clear();
    }

    std::vector<std::uint8_t>& acquire_packet() {
        if (pool_next_ >= packet_pool_.size()) {
            packet_pool_.emplace_back();
        }
        std::vector<std::uint8_t>& packet = packet_pool_[pool_next_];
        packet.clear();
        ++pool_next_;
        return packet;
    }

    void append_entity_frame(std::uint32_t entity_id, const Body& body) {
        const minphys3d::Vec3 p = minphys3d::BodyWorldShapeOrigin(body);
        std::vector<std::uint8_t>& out = acquire_packet();
        minphys_viz::EncodeEntityFrame(
            out,
            entity_id,
            frame_index_,
            sim_time_s_,
            p.x,
            p.y,
            p.z,
            body.orientation.w,
            body.orientation.x,
            body.orientation.y,
            body.orientation.z);
        packet_send_ptrs_.push_back(&out);
    }

    void append_entity_static(std::uint32_t entity_id, const BodyStaticDescriptor& d) {
        minphys_viz::VizEntityStaticBody fixed{};
        fixed.entity_id = entity_id;
        fixed.shape = ToVizShapeType(d.shape);
        fixed.static_friction = d.static_friction;
        fixed.dynamic_friction = d.dynamic_friction;
        fixed.restitution = d.restitution;
        fixed.radius = d.radius;
        fixed.half_height = d.half_height;
        fixed.half_extents = {d.half_extents.x, d.half_extents.y, d.half_extents.z};
        fixed.plane_normal = {d.plane_normal.x, d.plane_normal.y, d.plane_normal.z};
        fixed.plane_offset = d.plane_offset;
        fixed.compound_child_count = static_cast<std::uint32_t>(d.compound_children.size());

        compound_wire_scratch_.clear();
        compound_wire_scratch_.reserve(d.compound_children.size());
        for (const CompoundChild& c : d.compound_children) {
            compound_wire_scratch_.push_back(PackCompoundChild(c));
        }

        std::vector<std::uint8_t>& out = acquire_packet();
        minphys_viz::EncodeEntityStatic(out, fixed, compound_wire_scratch_.data(), compound_wire_scratch_.size());
        packet_send_ptrs_.push_back(&out);
    }

    void AppendTerrainPackets(const VizTerrainSnapshot& snap) {
        const int rows = snap.config.rows;
        const int cols = snap.config.cols;
        const std::size_t ncell = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
        if (rows <= 0 || cols <= 0 || snap.heights.size() != ncell || snap.confidences.size() != ncell) {
            return;
        }
        const bool has_collision = snap.schema_version >= 2 && !snap.collision_heights.empty()
            && snap.collision_heights.size() == ncell;
        const std::uint32_t layers = has_collision ? 3u : 2u;
        const std::uint32_t expected_bytes =
            static_cast<std::uint32_t>(ncell * sizeof(float) * layers);
        if (expected_bytes == 0u) {
            return;
        }

        ++terrain_seq_;
        if (terrain_float_blob_.size() < expected_bytes) {
            terrain_float_blob_.resize(expected_bytes);
        }
        std::memcpy(terrain_float_blob_.data(), snap.heights.data(), ncell * sizeof(float));
        std::memcpy(terrain_float_blob_.data() + ncell * sizeof(float), snap.confidences.data(), ncell * sizeof(float));
        if (has_collision) {
            std::memcpy(
                terrain_float_blob_.data() + 2u * ncell * sizeof(float),
                snap.collision_heights.data(),
                ncell * sizeof(float));
        }

        minphys_viz::VizTerrainPatchMetaBody meta{};
        meta.terrain_seq = terrain_seq_;
        meta.frame = frame_index_;
        meta.sim_time_s = sim_time_s_;
        meta.rows = rows;
        meta.cols = cols;
        meta.flags = has_collision ? minphys_viz::kTerrainFlagHasCollisionHeights : 0u;
        meta.cell_size_m = snap.config.cell_size_m;
        meta.base_margin_m = snap.config.base_margin_m;
        meta.min_cell_thickness_m = snap.config.min_cell_thickness_m;
        meta.influence_sigma_m = snap.config.influence_sigma_m;
        meta.plane_confidence = snap.config.plane_confidence;
        meta.confidence_half_life_s = snap.config.confidence_half_life_s;
        meta.base_update_blend = snap.config.base_update_blend;
        meta.decay_update_boost = snap.config.decay_update_boost;
        meta.use_sample_binning = snap.config.use_sample_binning ? 1u : 0u;
        meta.use_conservative_collision = snap.config.use_conservative_collision ? 1u : 0u;
        meta.scroll_world_fixed = snap.config.scroll_world_fixed ? 1u : 0u;
        meta.lidar_fusion_enable = snap.config.lidar_fusion_enable ? 1u : 0u;
        meta.sample_bin_size_m = snap.config.sample_bin_size_m;
        meta.lidar_sample_stride = snap.config.lidar_sample_stride;
        meta.lidar_sample_weight = snap.config.lidar_sample_weight;
        meta.lidar_min_surface_confidence = snap.config.lidar_min_surface_confidence;
        meta.lidar_contact_arbitration_radius_m = snap.config.lidar_contact_arbitration_radius_m;
        meta.lidar_contact_disagreement_m = snap.config.lidar_contact_disagreement_m;
        meta.center_world = {snap.center_world.x, snap.center_world.y, snap.center_world.z};
        meta.grid_origin_x = snap.grid_origin_world.x;
        meta.grid_origin_z = snap.grid_origin_world.z;
        meta.base_height_m = snap.base_height_m;
        meta.plane_height_m = snap.last_plane_height_m;
        meta.plane_normal = {snap.last_normal.x, snap.last_normal.y, snap.last_normal.z};
        meta.expected_float_blob_bytes = expected_bytes;

        std::vector<std::uint8_t>& meta_packet = acquire_packet();
        minphys_viz::EncodeTerrainMeta(meta_packet, meta);
        packet_send_ptrs_.push_back(&meta_packet);

        const std::size_t chunk_count = minphys_viz::EncodeTerrainFloatChunks(
            terrain_chunk_storage_,
            terrain_seq_,
            terrain_float_blob_.data(),
            expected_bytes,
            false);
        for (std::size_t ci = 0; ci < chunk_count; ++ci) {
            packet_send_ptrs_.push_back(&terrain_chunk_storage_[ci]);
        }
    }

    static bool AlmostEqual(float lhs, float rhs) {
        constexpr float kTolerance = 1e-6f;
        return std::fabs(lhs - rhs) <= kTolerance;
    }

    static bool AreDescriptorsEqual(const BodyStaticDescriptor& lhs, const BodyStaticDescriptor& rhs) {
        const auto childrenEqual = [&](const std::vector<CompoundChild>& a, const std::vector<CompoundChild>& b) {
            if (a.size() != b.size()) {
                return false;
            }
            for (std::size_t index = 0; index < a.size(); ++index) {
                const CompoundChild& childA = a[index];
                const CompoundChild& childB = b[index];
                if (childA.shape != childB.shape) {
                    return false;
                }
                if (!AlmostEqual(childA.localPosition.x, childB.localPosition.x)
                    || !AlmostEqual(childA.localPosition.y, childB.localPosition.y)
                    || !AlmostEqual(childA.localPosition.z, childB.localPosition.z)) {
                    return false;
                }
                if (!AlmostEqual(childA.localOrientation.w, childB.localOrientation.w)
                    || !AlmostEqual(childA.localOrientation.x, childB.localOrientation.x)
                    || !AlmostEqual(childA.localOrientation.y, childB.localOrientation.y)
                    || !AlmostEqual(childA.localOrientation.z, childB.localOrientation.z)) {
                    return false;
                }
                if (!AlmostEqual(childA.radius, childB.radius)
                    || !AlmostEqual(childA.halfHeight, childB.halfHeight)
                    || !AlmostEqual(childA.halfExtents.x, childB.halfExtents.x)
                    || !AlmostEqual(childA.halfExtents.y, childB.halfExtents.y)
                    || !AlmostEqual(childA.halfExtents.z, childB.halfExtents.z)) {
                    return false;
                }
            }
            return true;
        };
        return lhs.shape == rhs.shape
            && AlmostEqual(lhs.radius, rhs.radius)
            && AlmostEqual(lhs.half_height, rhs.half_height)
            && AlmostEqual(lhs.half_extents.x, rhs.half_extents.x)
            && AlmostEqual(lhs.half_extents.y, rhs.half_extents.y)
            && AlmostEqual(lhs.half_extents.z, rhs.half_extents.z)
            && childrenEqual(lhs.compound_children, rhs.compound_children)
            && AlmostEqual(lhs.plane_normal.x, rhs.plane_normal.x)
            && AlmostEqual(lhs.plane_normal.y, rhs.plane_normal.y)
            && AlmostEqual(lhs.plane_normal.z, rhs.plane_normal.z)
            && AlmostEqual(lhs.plane_offset, rhs.plane_offset)
            && AlmostEqual(lhs.static_friction, rhs.static_friction)
            && AlmostEqual(lhs.dynamic_friction, rhs.dynamic_friction)
            && AlmostEqual(lhs.restitution, rhs.restitution);
    }

    static BodyStaticDescriptor MakeStaticDescriptor(const Body& body) {
        return {
            body.shape,
            body.radius,
            body.halfHeight,
            ShapeBoundsHalfExtents(body),
            body.compoundChildren,
            body.planeNormal,
            body.planeOffset,
            body.staticFriction,
            body.dynamicFriction,
            body.restitution,
        };
    }

    std::string host_;
    int port_ = 0;
    int socket_fd_ = -1;
    bool valid_ = false;
    sockaddr_in dest_addr_{};

    int frame_index_ = 0;
    float sim_time_s_ = 0.0f;
    std::uint32_t terrain_seq_{0};
    std::vector<BodySnapshot> bodies_{};
    VizTerrainSnapshot viz_terrain_snapshot_{};
    std::map<std::uint32_t, BodyStaticDescriptor> last_static_descriptors_{};
    /// Pool of UDP payloads; acquire_packet() advances pool_next_; reset_packet_pool() starts each frame.
    mutable std::vector<std::vector<std::uint8_t>> packet_pool_{};
    mutable std::size_t pool_next_{0};
    mutable std::vector<const std::vector<std::uint8_t>*> packet_send_ptrs_{};
    mutable std::vector<std::uint8_t> scene_clear_packet_{};
    mutable std::vector<std::uint8_t> terrain_float_blob_{};
    mutable std::vector<std::vector<std::uint8_t>> terrain_chunk_storage_{};
    mutable std::vector<minphys_viz::VizCompoundChildWire> compound_wire_scratch_{};
#if defined(__linux__)
    mutable std::vector<iovec> iovec_scratch_{};
    mutable std::vector<mmsghdr> msg_scratch_{};
#endif
};
#endif

} // namespace

std::unique_ptr<FrameSink> MakeDummySink() {
    return std::make_unique<DummySink>();
}

std::unique_ptr<FrameSink> MakeUdpSink(const std::string& host, int port) {
#ifndef _WIN32
    return std::make_unique<UdpSink>(host, port);
#else
    (void)host;
    (void)port;
    std::cerr << "[udp_sink] UDP sink is not implemented on Windows in this build; using dummy sink\n";
    return MakeDummySink();
#endif
}

std::unique_ptr<FrameSink> MakeFrameSink(SinkKind kind, const std::string& udp_host, int udp_port) {
    if (kind == SinkKind::Udp) {
        return MakeUdpSink(udp_host, udp_port);
    }
    return MakeDummySink();
}

} // namespace minphys3d::demo
