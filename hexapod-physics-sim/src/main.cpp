#include "../demo/scenes.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    minphys3d::demo::SinkKind sink_kind = minphys3d::demo::SinkKind::Dummy;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--sink") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --sink (expected dummy|udp)\n";
                return 1;
            }
            const std::string value = argv[++i];
            if (value == "dummy") {
                sink_kind = minphys3d::demo::SinkKind::Dummy;
            } else if (value == "udp") {
                sink_kind = minphys3d::demo::SinkKind::Udp;
            } else {
                std::cerr << "Unsupported sink: " << value << " (expected dummy|udp)\n";
                return 1;
            }
            continue;
        }

        std::cerr << "Unknown argument: " << arg << "\n";
        return 1;
    }

    return minphys3d::demo::RunDefaultScene(sink_kind);
}
