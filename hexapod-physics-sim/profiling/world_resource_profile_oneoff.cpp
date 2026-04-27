#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
// One-off source for manual compile: captures World section profiler after a hexapod workload.
#include "demo/scenes.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_stability.hpp"
#include "process_resource_monitoring.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>

int main() {
    using namespace minphys3d;
    using namespace minphys3d::demo;

    World world(Vec3{0.0f, -9.81f, 0.0f});
    JointSolverConfig joint_cfg = world.GetJointSolverConfig();
    joint_cfg.servoPositionPasses = 8;
    world.SetJointSolverConfig(joint_cfg);

    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);

    constexpr float kFrameDt = 1.0f / 60.0f;
    const int kSubsteps = kHexapodPoseHoldBenchmarkSubstepsPerFrame;
    const int kIter = kHexapodPoseHoldBenchmarkSolverIterations;
    const float subDt = kFrameDt / static_cast<float>(kSubsteps);

    const auto t0 = std::chrono::steady_clock::now();
    constexpr int kFrames = 120;
    for (int frame = 0; frame < kFrames; ++frame) {
        for (int sub = 0; sub < kSubsteps; ++sub) {
            world.Step(subDt, kIter);
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double wall_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    const auto summary = world.SnapshotFullResourceSections(false);
    std::cout << "workload: hexapod " << kFrames << " frames @ 60Hz outer, " << kSubsteps
              << " substeps, " << kIter << " solver iters/substep; wall_ms=" << wall_ms << "\n";
    std::cout << "sections: " << summary.count << " (full snapshot, sorted by total self time)\n";
    std::cout << resource_monitoring::formatResourceSectionSummary(
                      summary, world_resource_monitoring::kMaxSections)
              << "\n";
    for (std::size_t i = 0; i < summary.count; ++i) {
        const auto& s = summary.sections[i];
        if (s.label == nullptr) {
            continue;
        }
        std::printf(
            "%-40s  total_ms=%8.3f  calls=%8llu  max_ms=%8.3f  window_ms=%8.3f\n",
            s.label,
            static_cast<double>(s.total_self_ns) / 1.0e6,
            static_cast<unsigned long long>(s.call_count),
            static_cast<double>(s.max_self_ns) / 1.0e6,
            static_cast<double>(s.window_self_ns) / 1.0e6);
    }
    return 0;
}
