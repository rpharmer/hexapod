// Determinism test: identical Worlds run identical Step() sequences must produce
// bit-exact-identical states. Any drift indicates uninitialised memory, undefined
// container ordering, or scheduling-dependent code paths in the sim core. This is
// a SHOULD-PASS test — if it fails we have a serious latent bug.

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

struct Snapshot {
    Vec3 bodyPos{};
    Vec3 bodyVel{};
    Vec3 bodyAngVel{};
    Quat bodyOri{};
    std::array<Real, 18> jointAngles{};
    std::array<Real, 18> jointSpeeds{};
};

Snapshot SnapshotWorld(World& world, const HexapodSceneObjects& scene) {
    Snapshot s{};
    const Body& body = world.GetBody(scene.body);
    s.bodyPos = body.position;
    s.bodyVel = body.velocity;
    s.bodyAngVel = body.angularVelocity;
    s.bodyOri = body.orientation;
    std::size_t i = 0;
    for (const LegLinkIds& leg : scene.legs) {
        for (const std::uint32_t jid : {leg.bodyToCoxaJoint, leg.coxaToFemurJoint, leg.femurToTibiaJoint}) {
            s.jointAngles[i] = world.GetServoJointAngle(jid);
            s.jointSpeeds[i] = world.GetServoJoint(jid).servoImpulseSum;
            ++i;
        }
    }
    return s;
}

// Bit-exact equality is the goal. Use std::memcmp on the Real storage so NaN bits
// would also be detected. We do NOT use approxEq — any deviation indicates a real
// determinism bug.
bool BitwiseEqual(const Snapshot& a, const Snapshot& b) {
    return std::memcmp(&a, &b, sizeof(Snapshot)) == 0;
}

int runCase() {
    World worldA({0.0, -9.81, 0.0});
    World worldB({0.0, -9.81, 0.0});
    const HexapodSceneObjects sceneA = BuildHexapodScene(worldA);
    const HexapodSceneObjects sceneB = BuildHexapodScene(worldB);
    RelaxBuiltInHexapodServos(worldA, sceneA);
    RelaxBuiltInHexapodServos(worldB, sceneB);

    constexpr Real kDt = 1.0 / 240.0;
    constexpr int kSteps = 480; // 2 seconds
    int firstDivergeStep = -1;
    for (int step = 0; step < kSteps; ++step) {
        worldA.Step(kDt, 16);
        worldB.Step(kDt, 16);
        const Snapshot a = SnapshotWorld(worldA, sceneA);
        const Snapshot b = SnapshotWorld(worldB, sceneB);
        if (!BitwiseEqual(a, b)) {
            firstDivergeStep = step;
            std::cerr << "world_step_determinism: divergence at step " << step << "\n"
                      << "  bodyPos A=(" << a.bodyPos.x << "," << a.bodyPos.y << "," << a.bodyPos.z << ")\n"
                      << "          B=(" << b.bodyPos.x << "," << b.bodyPos.y << "," << b.bodyPos.z << ")\n"
                      << "  bodyOri A.w=" << a.bodyOri.w << " B.w=" << b.bodyOri.w << "\n";
            // Find first differing joint to give a hint
            for (std::size_t j = 0; j < 18; ++j) {
                if (a.jointAngles[j] != b.jointAngles[j]) {
                    std::cerr << "  joint[" << j << "] A=" << a.jointAngles[j]
                              << " B=" << b.jointAngles[j]
                              << " diff=" << (a.jointAngles[j] - b.jointAngles[j]) << "\n";
                    break;
                }
            }
            break;
        }
    }
    if (firstDivergeStep >= 0) {
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
