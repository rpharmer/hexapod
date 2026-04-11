#pragma once

#include "minphys3d/core/world.hpp"

namespace minphys3d::core_internal {

struct SleepSystemContext {
    std::vector<Body>& bodies;
    const std::vector<Manifold>& manifolds;
    const std::vector<DistanceJoint>& joints;
    const std::vector<HingeJoint>& hingeJoints;
    const std::vector<BallSocketJoint>& ballSocketJoints;
    const std::vector<FixedJoint>& fixedJoints;
    const std::vector<PrismaticJoint>& prismaticJoints;
    const std::vector<ServoJoint>& servoJoints;
    float linearThreshold;
    float angularThreshold;
    std::uint32_t framesThreshold;
};

class SleepSystem {
public:
    void UpdateSleeping(const SleepSystemContext& context) const;
};

} // namespace minphys3d::core_internal
