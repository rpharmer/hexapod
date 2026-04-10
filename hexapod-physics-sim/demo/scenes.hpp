#pragma once

namespace minphys3d::demo {

enum class SinkKind {
    Dummy,
    Udp,
};

int RunDefaultScene(SinkKind sink_kind = SinkKind::Dummy);
int RunRealTimeDefaultScene(SinkKind sink_kind = SinkKind::Dummy);
} // namespace minphys3d::demo
