#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

struct KnobConfig {
    int solverIterations = 20;
    int substeps = 1;
    int servoPositionPasses = 8;
    int servoPositionSolveStride = 1;
    bool anchorEarlyOut = true;
    float anchorEarlyOutError = 2.5e-4f;
    float anchorEarlyOutSpeed = 0.03f;
};

struct KnobResult {
    KnobConfig cfg{};
    HexapodPoseHoldMetrics pose{};
    HexapodStandingStats standing{};
    double wallMs = 0.0;
    double stepMs = 0.0;
    double solveServoMs = 0.0;
    double solveJointPositionsServoMs = 0.0;
    double getServoAngleMs = 0.0;
};

double FindSectionMs(const world_resource_monitoring::SectionSummary& summary, const char* label) {
    for (std::size_t i = 0; i < summary.count; ++i) {
        const auto& s = summary.sections[i];
        if (s.label != nullptr && std::string(s.label) == label) {
            return static_cast<double>(s.total_self_ns) / 1.0e6;
        }
    }
    return 0.0;
}

KnobResult RunKnob(const KnobConfig& cfg) {
    KnobResult out{};
    out.cfg = cfg;

    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    JointSolverConfig jc = world.GetJointSolverConfig();
    jc.servoPositionPasses = static_cast<std::uint8_t>(std::max(0, cfg.servoPositionPasses));
    jc.servoPositionSolveStride = static_cast<std::uint8_t>(std::max(1, cfg.servoPositionSolveStride));
    jc.enableServoAnchorEarlyOut = cfg.anchorEarlyOut;
    jc.servoAnchorEarlyOutError = cfg.anchorEarlyOutError;
    jc.servoAnchorEarlyOutSpeed = cfg.anchorEarlyOutSpeed;
    world.SetJointSolverConfig(jc);

    constexpr float kFrameDt = 1.0f / 60.0f;
    constexpr int kFrames = 240;
    const float subDt = kFrameDt / static_cast<float>(std::max(cfg.substeps, 1));

    std::array<float, 18> previousAngles{};
    std::size_t ai = 0;
    for (const std::uint32_t id : HexapodServoJointIds(scene)) {
        previousAngles[ai++] = world.GetServoJointAngle(id);
    }

    const auto t0 = std::chrono::steady_clock::now();
    for (int frame = 0; frame < kFrames; ++frame) {
        for (int sub = 0; sub < std::max(cfg.substeps, 1); ++sub) {
            world.Step(subDt, std::max(cfg.solverIterations, 1));
            AccumulateHexapodPoseHoldFromSubstep(world, scene, scene.body, frame, out.pose);
        }

        const Body& chassis = world.GetBody(scene.body);
        const float bodyRoll = BodyRollRadStability(chassis.orientation);
        const float bodyPitch = BodyPitchRadStability(chassis.orientation);
        const std::array<HexapodLegContactRollup, 6> contactSummary =
            SummarizeHexapodGroundContacts(world, scene);
        int totalManifolds = 0;
        int totalPoints = 0;
        for (const HexapodLegContactRollup& c : contactSummary) {
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

    const auto summary = world.SnapshotFullResourceSections(false);
    out.stepMs = FindSectionMs(summary, "world.step");
    out.solveServoMs = FindSectionMs(summary, "world.solve_servo_joint");
    out.solveJointPositionsServoMs = FindSectionMs(summary, "world.solve_joint_positions.servo");
    out.getServoAngleMs = FindSectionMs(summary, "world.get_servo_joint_angle");
    return out;
}

} // namespace

int main() {
    const std::vector<KnobConfig> sweep = {
        // Focused anchor-fastpath A/B at current tuned operating point.
        // Anchor early-out sweep (stride 1).
        {40, 2, 8, 1, false, 0.0f, 0.0f},
        {40, 2, 8, 1, true, 1.0e-4f, 0.02f},
        {40, 2, 8, 1, true, 2.5e-4f, 0.03f},
        {40, 2, 8, 1, true, 5.0e-4f, 0.05f},
        // SolveJointPositions stride sweep at chosen early-out thresholds.
        {40, 2, 8, 2, true, 1.0e-4f, 0.02f},
        {40, 2, 8, 3, true, 1.0e-4f, 0.02f},
    };

    std::printf(
        "solver_iters,substeps,servo_pos_passes,servo_pos_stride,anchor_early_out,anchor_err,anchor_speed,wall_ms,step_ms,solve_servo_ms,"
        "solve_joint_pos_servo_ms,get_servo_angle_ms,peak_linear_settled,peak_angular,peak_joint_error,final_y,final_speed,"
        "max_roll_deg,max_pitch_deg,max_joint_speed\n");
    for (const KnobConfig& cfg : sweep) {
        const KnobResult r = RunKnob(cfg);
        std::printf(
            "%d,%d,%d,%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            r.cfg.solverIterations,
            r.cfg.substeps,
            r.cfg.servoPositionPasses,
            r.cfg.servoPositionSolveStride,
            r.cfg.anchorEarlyOut ? 1 : 0,
            static_cast<double>(r.cfg.anchorEarlyOutError),
            static_cast<double>(r.cfg.anchorEarlyOutSpeed),
            r.wallMs,
            r.stepMs,
            r.solveServoMs,
            r.solveJointPositionsServoMs,
            r.getServoAngleMs,
            r.pose.peakLinearSettled,
            r.pose.peakAngular,
            r.pose.peakJointErrorRad,
            r.pose.finalPosition.y,
            r.pose.finalSpeed,
            r.standing.maxAbsRollDeg,
            r.standing.maxAbsPitchDeg,
            r.standing.maxJointSpeedRadS);
    }
    return 0;
}
