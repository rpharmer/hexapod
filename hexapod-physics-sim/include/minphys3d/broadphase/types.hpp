#pragma once

#include <cstdint>

#include "minphys3d/collision/shapes.hpp"

namespace minphys3d {

struct Pair {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
};

struct TreeNode {
    AABB box{};
    std::int32_t left = -1;
    std::int32_t right = -1;
    std::int32_t parent = -1;
    std::int32_t bodyId = -1;
    std::int32_t height = 0;
    std::int32_t next = -1;
    bool IsLeaf() const { return bodyId >= 0; }
};

struct BroadphaseProxy {
    AABB fatBox{};
    std::int32_t leaf = -1;
    bool valid = false;
};

} // namespace minphys3d
