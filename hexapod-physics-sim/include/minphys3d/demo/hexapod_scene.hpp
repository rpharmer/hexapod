#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "minphys3d/core/world.hpp"

namespace minphys3d::demo {

struct LegLinkIds {
    std::uint32_t coxa = 0;
    std::uint32_t femur = 0;
    std::uint32_t tibia = 0;
    std::uint32_t bodyToCoxaJoint = World::kInvalidJointId;
    std::uint32_t coxaToFemurJoint = World::kInvalidJointId;
    std::uint32_t femurToTibiaJoint = World::kInvalidJointId;
};

struct HexapodSceneObjects {
    std::uint32_t plane = 0;
    std::uint32_t body = 0;
    std::array<LegLinkIds, 6> legs{};
    std::vector<std::uint32_t> body_ids{};
};

HexapodSceneObjects BuildHexapodScene(World& world);
void RelaxBuiltInHexapodServos(World& world, const HexapodSceneObjects& scene);
void RelaxZeroGravityHexapodServos(World& world, const HexapodSceneObjects& scene);

} // namespace minphys3d::demo
