#include "demo/frame_sink.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace minphys3d::demo {
namespace {

class DummySink final : public FrameSink {
public:
    void begin_frame(int, float) override {}
    void emit_body(std::uint32_t, const Body&) override {}
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
    }

    void emit_body(std::uint32_t body_id, const Body& body) override {
        bodies_.push_back({body_id, body});
    }

    void end_frame() override {
        if (!valid_) {
            return;
        }

        for (const auto& snapshot : bodies_) {
            const BodyStaticDescriptor descriptor = MakeStaticDescriptor(snapshot.body);
            const auto found = last_static_descriptors_.find(snapshot.id);
            if (found == last_static_descriptors_.end() || !AreDescriptorsEqual(found->second, descriptor)) {
                send_payload(BuildStaticMessage(snapshot.id, descriptor));
                last_static_descriptors_[snapshot.id] = descriptor;
            }

            send_payload(BuildFrameMessage(snapshot.id, snapshot.body));
        }
    }

private:
    void send_payload(const std::string& payload) const {
        (void)::sendto(
            socket_fd_,
            payload.data(),
            payload.size(),
            0,
            reinterpret_cast<const sockaddr*>(&dest_addr_),
            sizeof(dest_addr_));
    }

    static const char* ShapeName(ShapeType shape) {
        switch (shape) {
            case ShapeType::Sphere:
                return "sphere";
            case ShapeType::Box:
                return "box";
            case ShapeType::Plane:
                return "plane";
            case ShapeType::Capsule:
                return "capsule";
            case ShapeType::Cylinder:
                return "cylinder";
            case ShapeType::HalfCylinder:
                return "half_cylinder";
            case ShapeType::Compound:
                return "compound";
            default:
                return "unknown";
        }
    }

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

    void AppendCompoundChildrenJson(std::ostringstream& out, const std::vector<CompoundChild>& children) const {
        out << ",\"compound_children\":[";
        for (std::size_t index = 0; index < children.size(); ++index) {
            const CompoundChild& child = children[index];
            if (index > 0) {
                out << ",";
            }
            out << "{\"shape_type\":\"" << ShapeName(child.shape) << "\""
                << ",\"local_position\":[" << child.localPosition.x << "," << child.localPosition.y << "," << child.localPosition.z << "]"
                << ",\"local_rotation\":[" << child.localOrientation.w << "," << child.localOrientation.x << ","
                << child.localOrientation.y << "," << child.localOrientation.z << "]"
                << ",\"radius\":" << child.radius
                << ",\"half_height\":" << child.halfHeight
                << ",\"half_extents\":[" << child.halfExtents.x << "," << child.halfExtents.y << "," << child.halfExtents.z << "]"
                << "}";
        }
        out << "]";
    }

    std::string BuildStaticMessage(std::uint32_t entity_id, const BodyStaticDescriptor& descriptor) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(6);
        out << "{\"schema_version\":1,\"message_type\":\"entity_static\""
            << ",\"entity_id\":" << entity_id
            << ",\"shape_type\":\"" << ShapeName(descriptor.shape) << "\"";

        out << ",\"material\":{\"static_friction\":" << descriptor.static_friction
            << ",\"dynamic_friction\":" << descriptor.dynamic_friction
            << ",\"restitution\":" << descriptor.restitution << "}";

        out << ",\"mesh_key\":\"" << ShapeName(descriptor.shape) << "\"";

        if (descriptor.shape == ShapeType::Sphere) {
            out << ",\"dimensions\":{\"radius\":" << descriptor.radius << "}";
        } else if (descriptor.shape == ShapeType::Box) {
            out << ",\"dimensions\":{\"half_extents\":[" << descriptor.half_extents.x << "," << descriptor.half_extents.y
                << "," << descriptor.half_extents.z << "]}";
        } else if (descriptor.shape == ShapeType::Capsule || descriptor.shape == ShapeType::Cylinder
                   || descriptor.shape == ShapeType::HalfCylinder) {
            out << ",\"dimensions\":{\"radius\":" << descriptor.radius
                << ",\"half_height\":" << descriptor.half_height << "}";
        } else if (descriptor.shape == ShapeType::Compound) {
            out << ",\"dimensions\":{\"half_extents\":[" << descriptor.half_extents.x << "," << descriptor.half_extents.y
                << "," << descriptor.half_extents.z << "]}";
        } else if (descriptor.shape == ShapeType::Plane) {
            out << ",\"dimensions\":{\"plane_normal\":[" << descriptor.plane_normal.x << "," << descriptor.plane_normal.y
                << "," << descriptor.plane_normal.z << "],\"plane_offset\":" << descriptor.plane_offset << "}";
        }

        if (descriptor.shape == ShapeType::Compound && !descriptor.compound_children.empty()) {
            AppendCompoundChildrenJson(out, descriptor.compound_children);
        }

        out << "}";
        return out.str();
    }

    std::string BuildFrameMessage(std::uint32_t entity_id, const Body& body) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(6);
        out << "{\"schema_version\":1,\"message_type\":\"entity_frame\""
            << ",\"frame\":" << frame_index_
            << ",\"sim_time_s\":" << sim_time_s_
            << ",\"entity_id\":" << entity_id
            << ",\"position\":[" << body.position.x << "," << body.position.y << "," << body.position.z << "]"
            << ",\"rotation\":[" << body.orientation.w << "," << body.orientation.x << "," << body.orientation.y << ","
            << body.orientation.z << "]"
            << "}";
        return out.str();
    }

    struct BodySnapshot {
        std::uint32_t id = 0;
        Body body{};
    };

    std::string host_;
    int port_ = 0;
    int socket_fd_ = -1;
    bool valid_ = false;
    sockaddr_in dest_addr_{};

    int frame_index_ = 0;
    float sim_time_s_ = 0.0f;
    std::vector<BodySnapshot> bodies_{};
    std::map<std::uint32_t, BodyStaticDescriptor> last_static_descriptors_{};
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
