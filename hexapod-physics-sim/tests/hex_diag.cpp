#include <cmath>
#include <cstdio>
#include <cstdint>
#include <algorithm>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

using namespace minphys3d;
using namespace minphys3d::demo;

void RunConfig(const char* label, float stabOverride) {
    World world({0.0f, -9.81f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    joint_cfg.hingeAnchorBiasFactor = 0.25f;
    joint_cfg.hingeAnchorDampingFactor = 0.3f;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    if (stabOverride >= 0.0f) {
        for (int leg = 0; leg < 6; ++leg) {
            world.GetServoJointMutable(scene.legs[leg].bodyToCoxaJoint).angleStabilizationScale = stabOverride;
            world.GetServoJointMutable(scene.legs[leg].coxaToFemurJoint).angleStabilizationScale = stabOverride;
            world.GetServoJointMutable(scene.legs[leg].femurToTibiaJoint).angleStabilizationScale = stabOverride;
        }
    }

    constexpr float kFrameDt = 1.0f / 60.0f;
    constexpr int kSubsteps = 6;
    constexpr int kIter = 80;
    const float subDt = kFrameDt / float(kSubsteps);

    for (int frame = 0; frame < 480; ++frame) {
        for (int sub = 0; sub < kSubsteps; ++sub) {
            world.Step(subDt, kIter);
        }

        if (frame == 59 || frame == 479) {
            const Body& ch = world.GetBody(scene.body);

            float total_chassis_N = 0.0f, total_foot_N = 0.0f;
            for (const Manifold& m : world.DebugManifolds()) {
                if ((m.a == scene.plane && m.b == scene.body) || (m.a == scene.body && m.b == scene.plane)) {
                    for (const Contact& c : m.contacts) total_chassis_N += std::max(c.normalImpulseSum, 0.0f);
                }
                for (int leg = 0; leg < 6; ++leg) {
                    if ((m.a == scene.plane && m.b == scene.legs[leg].tibia) ||
                        (m.a == scene.legs[leg].tibia && m.b == scene.plane)) {
                        for (const Contact& c : m.contacts) total_foot_N += std::max(c.normalImpulseSum, 0.0f);
                    }
                }
            }

            const ServoJoint& hip0 = world.GetServoJoint(scene.legs[0].bodyToCoxaJoint);
            const ServoJoint& fem0 = world.GetServoJoint(scene.legs[0].coxaToFemurJoint);
            const Body& bA = world.GetBody(hip0.a);
            const Body& bB = world.GetBody(hip0.b);
            const Vec3 hipWorldA = bA.position + Rotate(bA.orientation, hip0.localAnchorA);
            const Vec3 hipWorldB = bB.position + Rotate(bB.orientation, hip0.localAnchorB);
            const Body& fA = world.GetBody(fem0.a);
            const Vec3 femWorld = fA.position + Rotate(fA.orientation, fem0.localAnchorA);

            const Vec3 axA0 = Normalize(Rotate(bA.orientation, hip0.localAxisA));
            const Vec3 axB0 = Normalize(Rotate(bB.orientation, hip0.localAxisB));
            const float axisErr = Length(Cross(axA0, axB0));

            std::printf("[%s] frame=%d chassis_y=%.4f bottom=%.4f | hip_y=%.4f femJ_y=%.4f coxa_tilt=%.4f | "
                "chassisN=%.4f feetN=%.4f ratio=%.0f%% | axisAlignErr=%.6f\n",
                label, frame, ch.position.y, ch.position.y - 0.034f,
                hipWorldA.y, femWorld.y, femWorld.y - hipWorldA.y,
                total_chassis_N, total_foot_N,
                (total_chassis_N + total_foot_N > 0.001f) ? 100.0f * total_chassis_N / (total_chassis_N + total_foot_N) : 0.0f,
                axisErr);
        }
    }
}

int main() {
    RunConfig("stab=0.0", 0.0f);
    RunConfig("stab=0.1", 0.1f);
    return 0;
}
