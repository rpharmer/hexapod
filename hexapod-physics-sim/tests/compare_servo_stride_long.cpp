#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

struct Result {
    int stride = 1;
    double wallMs = 0.0;
    HexapodPoseHoldMetrics pose{};
    HexapodStandingStats standing{};
};

Result RunStride(int stride, int frames) {
    Result out{};
    out.stride = stride;

    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    JointSolverConfig jc = world.GetJointSolverConfig();
    jc.servoPositionSolveStride = static_cast<std::uint8_t>(std::max(1, stride));
    world.SetJointSolverConfig(jc);

    constexpr float kFrameDt = 1.0f / 60.0f;
    const float subDt = kFrameDt / static_cast<float>(kHexapodPoseHoldBenchmarkSubstepsPerFrame);
    std::array<float, 18> previousAngles{};
    std::size_t ai = 0;
    for (const std::uint32_t id : HexapodServoJointIds(scene)) {
        previousAngles[ai++] = world.GetServoJointAngle(id);
    }

    const auto t0 = std::chrono::steady_clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (int sub = 0; sub < kHexapodPoseHoldBenchmarkSubstepsPerFrame; ++sub) {
            world.Step(subDt, kHexapodPoseHoldBenchmarkSolverIterations);
            AccumulateHexapodPoseHoldFromSubstep(world, scene, scene.body, frame, out.pose);
        }
        const Body& chassis = world.GetBody(scene.body);
        const float bodyRoll = BodyRollRadStability(chassis.orientation);
        const float bodyPitch = BodyPitchRadStability(chassis.orientation);
        const std::array<HexapodLegContactRollup, 6> contacts =
            SummarizeHexapodGroundContacts(world, scene);
        int totalManifolds = 0;
        int totalPoints = 0;
        for (const HexapodLegContactRollup& c : contacts) {
            totalManifolds += c.coxa.manifolds + c.femur.manifolds + c.tibia.manifolds;
            totalPoints += c.coxa.points + c.femur.points + c.tibia.points;
        }
        const float maxJointSpeed = MaxHexapodJointSpeedRadSFrame(world, scene, previousAngles, kFrameDt);
        UpdateHexapodStandingStats(
            out.standing, chassis, bodyRoll, bodyPitch, totalManifolds, totalPoints, maxJointSpeed);
    }
    const auto t1 = std::chrono::steady_clock::now();
    out.wallMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    FinalizeHexapodPoseHold(world.GetBody(scene.body), out.pose);
    return out;
}

} // namespace

int main(int argc, char** argv) {
    int frames = 720;
    if (argc >= 2) {
        frames = std::max(60, std::atoi(argv[1]));
    }

    const Result s2 = RunStride(2, frames);
    const Result s3 = RunStride(3, frames);

    std::printf("frames=%d,solver_iters=%d,substeps=%d\n",
                frames,
                kHexapodPoseHoldBenchmarkSolverIterations,
                kHexapodPoseHoldBenchmarkSubstepsPerFrame);
    std::printf("stride,wall_ms,peak_linear_settled,peak_angular,peak_joint_error,final_y,final_speed,max_roll_deg,max_pitch_deg,max_joint_speed\n");
    auto row = [](const Result& r) {
        std::printf("%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    r.stride,
                    r.wallMs,
                    r.pose.peakLinearSettled,
                    r.pose.peakAngular,
                    r.pose.peakJointErrorRad,
                    r.pose.finalPosition.y,
                    r.pose.finalSpeed,
                    r.standing.maxAbsRollDeg,
                    r.standing.maxAbsPitchDeg,
                    r.standing.maxJointSpeedRadS);
    };
    row(s2);
    row(s3);
    return 0;
}
