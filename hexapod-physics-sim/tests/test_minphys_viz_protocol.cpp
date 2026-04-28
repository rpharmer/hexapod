#include "minphys_viz_protocol.hpp"

#include <cstdlib>
#include <vector>

int main() {
    static_assert(sizeof(minphys_viz::VizWireHeader) == 8);
    static_assert(sizeof(minphys_viz::VizEntityFrameBody) == 40);

    std::vector<std::uint8_t> frame;
    minphys_viz::EncodeEntityFrame(frame, 42u, 9, 0.125f, 1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    if (!minphys_viz::IsVizBinaryPayload(frame.data(), frame.size())) {
        return 1;
    }
    if (frame.size() != sizeof(minphys_viz::VizWireHeader) + sizeof(minphys_viz::VizEntityFrameBody)) {
        return 2;
    }

    std::vector<std::vector<std::uint8_t>> chunks;
    float blob[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    minphys_viz::EncodeTerrainFloatChunks(chunks, 1u, blob, sizeof(blob));
    if (chunks.empty()) {
        return 3;
    }
    return 0;
}
