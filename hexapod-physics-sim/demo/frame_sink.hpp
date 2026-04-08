#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "minphys3d/core/body.hpp"

namespace minphys3d::demo {

class FrameSink {
public:
    virtual ~FrameSink() = default;

    virtual void begin_frame(int frame_index, float sim_time_s) = 0;
    virtual void emit_body(std::uint32_t body_id, const Body& body) = 0;
    virtual void end_frame() = 0;
};

std::unique_ptr<FrameSink> MakeDummySink();
std::unique_ptr<FrameSink> MakeUdpSink(const std::string& host = "127.0.0.1", int port = 9870);

} // namespace minphys3d::demo
