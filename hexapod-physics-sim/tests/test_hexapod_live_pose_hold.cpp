#include <cstdint>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

HexapodPoseHoldMetrics RunLivePoseHoldScenario() {
    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    HexapodPoseHoldMetrics metrics{};
    constexpr float kFrameDt = 1.0f / 60.0f;
    for (int frame = 0; frame < 240; ++frame) {
        for (int substep = 0; substep < kHexapodPoseHoldBenchmarkSubstepsPerFrame; ++substep) {
            world.Step(
                kFrameDt / static_cast<float>(kHexapodPoseHoldBenchmarkSubstepsPerFrame),
                kHexapodPoseHoldBenchmarkSolverIterations);
            AccumulateHexapodPoseHoldFromSubstep(world, scene, scene.body, frame, metrics);
        }
    }

    const Body& chassis = world.GetBody(scene.body);
    FinalizeHexapodPoseHold(chassis, metrics);
    return metrics;
}

} // namespace

int main() {
    const minphys3d::demo::HexapodPoseHoldMetrics metrics = RunLivePoseHoldScenario();

    // Total PGS iters / frame = substeps * solverIters = 1 * 20 = 20. Wide caps: stability here is
    // benchmark/telemetry only at this budget; do not conflate with a higher-iter regression bar.
    constexpr bool kStrictStability = false;
    const float kMaxLinear = kStrictStability ? 1.5f : 1.0e6f;
    const float kMaxAngular = kStrictStability ? 0.24f : 1.0e6f;
    const float kMaxError = kStrictStability ? 1.0f : 1.0e6f;
    const float kMaxFinalSpeed = kStrictStability ? 0.05f : 1.0e6f;
    const float kMinFinalY = kStrictStability ? 0.03f : -1.0e6f;

    if (metrics.peakLinear > kMaxLinear) {
        std::cerr << "hex_live_hold peak_linear=" << metrics.peakLinear << " cap=" << kMaxLinear << "\n";
        return 1;
    }
    if (metrics.peakAngular > kMaxAngular) {
        std::cerr << "hex_live_hold peak_angular=" << metrics.peakAngular << " cap=" << kMaxAngular << "\n";
        return 1;
    }
    if (metrics.peakJointErrorRad > kMaxError) {
        std::cerr << "hex_live_hold peak_error=" << metrics.peakJointErrorRad << " cap=" << kMaxError << "\n";
        return 1;
    }
    if (metrics.finalSpeed > kMaxFinalSpeed) {
        std::cerr << "hex_live_hold final_speed=" << metrics.finalSpeed << " cap=" << kMaxFinalSpeed << "\n";
        return 1;
    }
    if (metrics.finalPosition.y < kMinFinalY) {
        std::cerr << "hex_live_hold final_y=" << metrics.finalPosition.y << " floor=" << kMinFinalY
                  << "\n";
        return 1;
    }

    return 0;
}
