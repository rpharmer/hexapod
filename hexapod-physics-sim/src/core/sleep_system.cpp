#include "minphys3d/core/sleep_system.hpp"

namespace minphys3d::core_internal {

void SleepSystem::UpdateSleeping(const SleepSystemContext& context) const {
    const std::size_t n = context.bodies.size();

    // Reuse per-call allocations; vectors retain capacity across substeps.
    visited_.assign(n, false);
    adj_.assign(n, {});

    // Build adjacency list in one pass over each constraint type — O(E) instead of O(N*E) in BFS.
    auto addEdge = [&](std::uint32_t a, std::uint32_t b) {
        if (a < n) adj_[a].push_back(b);
        if (b < n) adj_[b].push_back(a);
    };
    for (const Manifold& m : context.manifolds) {
        addEdge(m.a, m.b);
    }
    for (const DistanceJoint& j : context.joints) {
        addEdge(j.a, j.b);
    }
    for (const HingeJoint& j : context.hingeJoints) {
        addEdge(j.a, j.b);
    }
    for (const BallSocketJoint& j : context.ballSocketJoints) {
        addEdge(j.a, j.b);
    }
    for (const FixedJoint& j : context.fixedJoints) {
        addEdge(j.a, j.b);
    }
    for (const PrismaticJoint& j : context.prismaticJoints) {
        addEdge(j.a, j.b);
    }
    for (const ServoJoint& j : context.servoJoints) {
        addEdge(j.a, j.b);
    }

    for (std::uint32_t start = 0; start < n; ++start) {
        if (visited_[start] || context.bodies[start].invMass == 0.0f) continue;

        islandBodies_.clear();
        stack_.clear();
        stack_.push_back(start);
        visited_[start] = true;
        bool islandNearlyStill = true;

        while (!stack_.empty()) {
            const std::uint32_t id = stack_.back();
            stack_.pop_back();
            Body& body = context.bodies[id];
            islandBodies_.push_back(id);

            const bool nearlyStill = LengthSquared(body.velocity) < (context.linearThreshold * context.linearThreshold)
                && LengthSquared(body.angularVelocity) < (context.angularThreshold * context.angularThreshold);
            islandNearlyStill = islandNearlyStill && nearlyStill;

            for (std::uint32_t other : adj_[id]) {
                if (other < n && context.bodies[other].invMass != 0.0f && !visited_[other]) {
                    visited_[other] = true;
                    stack_.push_back(other);
                }
            }
        }

        for (std::uint32_t id : islandBodies_) {
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
