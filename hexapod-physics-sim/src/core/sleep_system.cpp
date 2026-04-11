#include "minphys3d/core/sleep_system.hpp"

namespace minphys3d::core_internal {

void SleepSystem::UpdateSleeping(const SleepSystemContext& context) const {
    std::vector<bool> visited(context.bodies.size(), false);
    for (std::uint32_t start = 0; start < context.bodies.size(); ++start) {
        if (visited[start] || context.bodies[start].invMass == 0.0f) continue;

        std::vector<std::uint32_t> islandBodies;
        std::vector<std::uint32_t> stack{start};
        visited[start] = true;
        bool islandNearlyStill = true;

        while (!stack.empty()) {
            const std::uint32_t id = stack.back();
            stack.pop_back();
            Body& body = context.bodies[id];
            islandBodies.push_back(id);

            const bool nearlyStill = LengthSquared(body.velocity) < (context.linearThreshold * context.linearThreshold)
                && LengthSquared(body.angularVelocity) < (context.angularThreshold * context.angularThreshold);
            islandNearlyStill = islandNearlyStill && nearlyStill;

            auto enqueueConnectedBodies = [&](std::uint32_t other) {
                if (other < context.bodies.size() && context.bodies[other].invMass != 0.0f && !visited[other]) {
                    visited[other] = true;
                    stack.push_back(other);
                }
            };

            for (const Manifold& m : context.manifolds) {
                if (m.a == id) enqueueConnectedBodies(m.b);
                else if (m.b == id) enqueueConnectedBodies(m.a);
            }
            for (const DistanceJoint& j : context.joints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
            for (const HingeJoint& j : context.hingeJoints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
            for (const BallSocketJoint& j : context.ballSocketJoints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
            for (const FixedJoint& j : context.fixedJoints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
            for (const PrismaticJoint& j : context.prismaticJoints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
            for (const ServoJoint& j : context.servoJoints) {
                if (j.a == id) enqueueConnectedBodies(j.b);
                else if (j.b == id) enqueueConnectedBodies(j.a);
            }
        }

        for (std::uint32_t id : islandBodies) {
            Body& body = context.bodies[id];
            if (islandNearlyStill) {
                ++body.sleepCounter;
                if (body.sleepCounter >= static_cast<int>(context.framesThreshold)) {
                    body.isSleeping = true;
                    body.velocity = {0.0f, 0.0f, 0.0f};
                    body.angularVelocity = {0.0f, 0.0f, 0.0f};
                }
            } else {
                body.isSleeping = false;
                body.sleepCounter = 0;
            }
        }
    }
}

} // namespace minphys3d::core_internal
