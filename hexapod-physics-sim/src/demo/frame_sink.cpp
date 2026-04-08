#include "demo/frame_sink.hpp"

#include <array>
#include <iomanip>
#include <iostream>
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

        std::ostringstream out;
        out << std::fixed << std::setprecision(6);
        out << "{\"schema_version\":1,\"type\":\"minphys_frame\",\"frame\":" << frame_index_
            << ",\"sim_time_s\":" << sim_time_s_ << ",\"bodies\":[";

        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            const auto& snapshot = bodies_[i];
            const Body& body = snapshot.body;
            out << (i == 0 ? "" : ",")
                << "{\"id\":" << snapshot.id << ",\"shape\":\"" << ShapeName(body.shape) << "\""
                << ",\"position\":[" << body.position.x << "," << body.position.y << "," << body.position.z << "]"
                << ",\"orientation\":[" << body.orientation.w << "," << body.orientation.x << ","
                << body.orientation.y << "," << body.orientation.z << "]";
            if (body.shape == ShapeType::Sphere) {
                out << ",\"radius\":" << body.radius;
            }
            if (body.shape == ShapeType::Box) {
                out << ",\"half_extents\":[" << body.halfExtents.x << "," << body.halfExtents.y << ","
                    << body.halfExtents.z << "]";
            }
            if (body.shape == ShapeType::Plane) {
                out << ",\"plane_normal\":[" << body.planeNormal.x << "," << body.planeNormal.y << ","
                    << body.planeNormal.z << "]"
                    << ",\"plane_offset\":" << body.planeOffset;
            }
            out << "}";
        }

        out << "]}";
        const std::string payload = out.str();
        (void)::sendto(
            socket_fd_,
            payload.data(),
            payload.size(),
            0,
            reinterpret_cast<const sockaddr*>(&dest_addr_),
            sizeof(dest_addr_));
    }

private:
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
            default:
                return "unknown";
        }
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

} // namespace minphys3d::demo
