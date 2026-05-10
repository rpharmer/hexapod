// Records pose-hold + standing window metrics to a text file (baseline for solver tuning).
// See FormatHexapodStabilityRecord in hexapod_stability.hpp.

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

static const char kStabilityGit[] =
#include "stability_git.inc"
    ;

void RunRecord(const char* out_path) {
    World world({0.0, -9.81, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    HexapodPoseHoldMetrics pose{};
    HexapodStandingStats standing{};

    constexpr Real kFrameDt = 1.0 / 60.0;
    const Real sub_dt = kFrameDt / static_cast<float>(kHexapodPoseHoldBenchmarkSubstepsPerFrame);

    std::array<Real, 18> previous_angles{};
    {
        std::size_t i = 0;
        for (const std::uint32_t joint_id : HexapodServoJointIds(scene)) {
            previous_angles[i++] = world.GetServoJointAngle(joint_id);
        }
    }

    for (int frame = 0; frame < 240; ++frame) {
        for (int substep = 0; substep < kHexapodPoseHoldBenchmarkSubstepsPerFrame; ++substep) {
            world.Step(sub_dt, kHexapodPoseHoldBenchmarkSolverIterations);
            AccumulateHexapodPoseHoldFromSubstep(world, scene, scene.body, frame, pose);
        }

        // Match RunHexapod demo: one standing sample per display frame, joint rates vs frame dt.
        const Body& chassis = world.GetBody(scene.body);
        const Real body_roll = BodyRollRadStability(chassis.orientation);
        const Real body_pitch = BodyPitchRadStability(chassis.orientation);
        const std::array<HexapodLegContactRollup, 6> contact_summary =
            SummarizeHexapodGroundContacts(world, scene);
        int total_manifolds = 0;
        int total_points = 0;
        for (const HexapodLegContactRollup& leg_contacts : contact_summary) {
            total_manifolds +=
                leg_contacts.coxa.manifolds + leg_contacts.femur.manifolds + leg_contacts.tibia.manifolds;
            total_points +=
                leg_contacts.coxa.points + leg_contacts.femur.points + leg_contacts.tibia.points;
        }
        Real max_joint_speed_rad_s = 0.0;
        for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
            const LegLinkIds& leg = scene.legs[leg_index];
            const Real coxa_angle = world.GetServoJointAngle(leg.bodyToCoxaJoint);
            const Real femur_angle = world.GetServoJointAngle(leg.coxaToFemurJoint);
            const Real tibia_angle = world.GetServoJointAngle(leg.femurToTibiaJoint);
            const Real coxa_speed = (coxa_angle - previous_angles[leg_index * 3 + 0]) / kFrameDt;
            const Real femur_speed = (femur_angle - previous_angles[leg_index * 3 + 1]) / kFrameDt;
            const Real tibia_speed = (tibia_angle - previous_angles[leg_index * 3 + 2]) / kFrameDt;
            max_joint_speed_rad_s = std::max(
                max_joint_speed_rad_s,
                std::max(std::abs(coxa_speed), std::max(std::abs(femur_speed), std::abs(tibia_speed))));
            previous_angles[leg_index * 3 + 0] = coxa_angle;
            previous_angles[leg_index * 3 + 1] = femur_angle;
            previous_angles[leg_index * 3 + 2] = tibia_angle;
        }
        UpdateHexapodStandingStats(
            standing, chassis, body_roll, body_pitch, total_manifolds, total_points, max_joint_speed_rad_s);
    }

    const Body& final_ch = world.GetBody(scene.body);
    FinalizeHexapodPoseHold(final_ch, pose);

    const std::string text = FormatHexapodStabilityRecord(pose, standing, kStabilityGit);
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(out_path).parent_path(), ec);
    std::ofstream f(out_path);
    if (!f) {
        std::cerr << "record_hexapod_stability: failed to open " << out_path << "\n";
        std::exit(1);
    }
    f << text;
    std::cout << text;
}

} // namespace

int main(int argc, char** argv) {
    const char* out =
        (argc >= 2) ? argv[1] : "profiling/hexapod_stability_baseline_2026-04-26.txt";
    RunRecord(out);
    return 0;
}
