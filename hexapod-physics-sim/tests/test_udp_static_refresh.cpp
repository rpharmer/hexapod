#include "demo/frame_sink.hpp"
#include "minphys_viz_protocol.hpp"

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>

int main() {
#ifdef _WIN32
    return 0;
#else
    const int listener = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (listener < 0) return 1;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (::bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(listener);
        return 2;
    }
    socklen_t address_size = sizeof(address);
    if (::getsockname(listener, reinterpret_cast<sockaddr*>(&address), &address_size) != 0) {
        ::close(listener);
        return 3;
    }
    auto sink = minphys3d::demo::MakeUdpSink("127.0.0.1", ntohs(address.sin_port));
    minphys3d::Body body{};
    body.shape = minphys3d::ShapeType::Box;
    body.halfExtents = {0.1, 0.1, 0.1};

    auto check_frame = [&](int frame, int expected_static) {
        sink->begin_frame(frame, frame / 30.0);
        sink->emit_body(7, body);
        sink->end_frame();
        int static_count = 0;
        int pose_count = 0;
        std::array<std::uint8_t, 4096> packet{};
        for (;;) {
            const ssize_t size = ::recvfrom(listener, packet.data(), packet.size(), MSG_DONTWAIT,
                                            nullptr, nullptr);
            if (size < static_cast<ssize_t>(sizeof(minphys_viz::VizWireHeader))) break;
            minphys_viz::VizWireHeader header{};
            std::memcpy(&header, packet.data(), sizeof(header));
            if (header.message_kind ==
                static_cast<std::uint8_t>(minphys_viz::VizMessageKind::EntityStatic)) {
                ++static_count;
            } else if (header.message_kind ==
                       static_cast<std::uint8_t>(minphys_viz::VizMessageKind::EntityFrame)) {
                ++pose_count;
            }
        }
        if (static_count != expected_static || pose_count != 1) {
            std::cerr << "frame " << frame << ": static=" << static_count
                      << " pose=" << pose_count << '\n';
            return false;
        }
        return true;
    };

    // Skipping frame 30 simulates a dropped asynchronous preview. The next
    // delivered frame must still refresh the shape rather than missing forever.
    const bool ok = check_frame(0, 1) && check_frame(1, 0) &&
                    check_frame(31, 1) && check_frame(32, 0);
    ::close(listener);
    return ok ? 0 : 4;
#endif
}
