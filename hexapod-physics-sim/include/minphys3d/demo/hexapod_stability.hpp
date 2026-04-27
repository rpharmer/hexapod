#pragma once

#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/joints/types.hpp"
#include "minphys3d/math/vec3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

namespace minphys3d::demo {

inline constexpr float kHexapodStabilityPi = 3.14159265358979323846f;

inline float WrapHexapodAngle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline Vec3 BodyUpVectorStability(const Quat& orientation) {
    return Rotate(orientation, Vec3{0.0f, 1.0f, 0.0f});
}

inline float BodyRollRadStability(const Quat& orientation) {
    const Vec3 up = BodyUpVectorStability(orientation);
    return std::atan2(up.x, std::max(up.y, 1.0e-6f));
}

inline float BodyPitchRadStability(const Quat& orientation) {
    const Vec3 up = BodyUpVectorStability(orientation);
    return std::atan2(-up.z, std::max(up.y, 1.0e-6f));
}

// --- Ground / contact rollups (moved from scenes.cpp; used for standing metrics) ---

struct HexapodLinkContactRollup {
    int manifolds = 0;
    int points = 0;
    float totalNormalImpulse = 0.0f;
    float maxPenetration = 0.0f;
};

struct HexapodLegContactRollup {
    HexapodLinkContactRollup coxa{};
    HexapodLinkContactRollup femur{};
    HexapodLinkContactRollup tibia{};
};

inline HexapodLinkContactRollup* FindLegContactRollup(
    std::array<HexapodLegContactRollup, 6>& summaries,
    const HexapodSceneObjects& scene,
    std::uint32_t body_id) {
    for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
        const LegLinkIds& leg = scene.legs[leg_index];
        if (leg.coxa == body_id) {
            return &summaries[leg_index].coxa;
        }
        if (leg.femur == body_id) {
            return &summaries[leg_index].femur;
        }
        if (leg.tibia == body_id) {
            return &summaries[leg_index].tibia;
        }
    }
    return nullptr;
}

inline std::array<HexapodLegContactRollup, 6> SummarizeHexapodGroundContacts(
    const World& world, const HexapodSceneObjects& scene) {
    std::array<HexapodLegContactRollup, 6> summaries{};
    for (const Manifold& manifold : world.DebugManifolds()) {
        const bool a_is_plane = manifold.a == scene.plane;
        const bool b_is_plane = manifold.b == scene.plane;
        if (a_is_plane == b_is_plane) {
            continue;
        }
        const std::uint32_t body_id = a_is_plane ? manifold.b : manifold.a;
        HexapodLinkContactRollup* summary = FindLegContactRollup(summaries, scene, body_id);
        if (summary == nullptr) {
            continue;
        }
        ++summary->manifolds;
        summary->points += static_cast<int>(manifold.contacts.size());
        for (const Contact& contact : manifold.contacts) {
            summary->totalNormalImpulse += std::max(contact.normalImpulseSum, 0.0f);
            summary->maxPenetration = std::max(summary->maxPenetration, contact.penetration);
        }
    }
    return summaries;
}

/// Min/max body height, peak tilt, joint speed, and minimum support (contact) counts over a run.
struct HexapodStandingStats {
    float minBodyHeight = std::numeric_limits<float>::infinity();
    float maxBodyHeight = -std::numeric_limits<float>::infinity();
    float maxAbsRollDeg = 0.0f;
    float maxAbsPitchDeg = 0.0f;
    float maxJointSpeedRadS = 0.0f;
    int minContactManifolds = std::numeric_limits<int>::max();
    int minContactPoints = std::numeric_limits<int>::max();
};

inline void UpdateHexapodStandingStats(
    HexapodStandingStats& stats,
    const Body& chassis,
    float roll_rad,
    float pitch_rad,
    int contact_manifolds,
    int contact_points,
    float max_joint_speed_rad_s) {
    stats.minBodyHeight = std::min(stats.minBodyHeight, chassis.position.y);
    stats.maxBodyHeight = std::max(stats.maxBodyHeight, chassis.position.y);
    stats.maxAbsRollDeg = std::max(stats.maxAbsRollDeg, std::abs(roll_rad) * 180.0f / kHexapodStabilityPi);
    stats.maxAbsPitchDeg = std::max(stats.maxAbsPitchDeg, std::abs(pitch_rad) * 180.0f / kHexapodStabilityPi);
    stats.maxJointSpeedRadS = std::max(stats.maxJointSpeedRadS, max_joint_speed_rad_s);
    stats.minContactManifolds = std::min(stats.minContactManifolds, contact_manifolds);
    stats.minContactPoints = std::min(stats.minContactPoints, contact_points);
}

/// First outer frame (0-based) at which "settled" peak linear speed is tracked. Before this, the
/// hexapod may still be loading contacts / rebounding; peaks there are excluded from
/// `peakLinearSettled`. Assumes ~60 Hz outer steps (`Step` once per display frame after substeps).
inline constexpr int kHexapodPoseHoldSettledStartFrame = 60;

/// PGS iters per `World::Step(subdt, n)` (main constraint loop). Tuned scenario keeps **total** PGS
/// work per ~60Hz frame = `kHexapodPoseHoldBenchmarkSubstepsPerFrame * this` **fixed** (e.g. 1×20=20);
/// do not raise substeps or this without matching cuts if the budget is 20 PGS iters / frame.
inline constexpr int kHexapodPoseHoldBenchmarkSolverIterations = 40;

/// Substeps per ~60Hz frame. With [substeps]×[solver iters] capped, use **1** so total PGS iters / frame
/// = 1×20 = 20 (not 3×20 = 60).
inline constexpr int kHexapodPoseHoldBenchmarkSubstepsPerFrame = 2;

/// Post-velocity joint position/anchor passes (`SolveJointPositions`); not counted as PGS `solverIterations`
/// but use default 8 to avoid extra work unless profiling shows a win.
inline constexpr int kHexapodPoseHoldJointServoPositionPasses = 8;
inline constexpr int kHexapodPoseHoldJointServoPositionStride = 3;
inline constexpr float kHexapodPoseHoldHingeAnchorBias = 0.35f;
inline constexpr float kHexapodPoseHoldHingeAnchorDamping = 0.45f;
inline constexpr float kHexapodPoseHoldServoEarlyOutResumeScale = 1.6f;
inline constexpr float kHexapodPoseHoldServoEarlyOutImpulseGuard = 0.18f;
inline constexpr float kHexapodPoseHoldServoAngularEarlyOutError = 0.0014f;
inline constexpr float kHexapodPoseHoldServoAngularEarlyOutSpeed = 0.055f;
inline constexpr float kHexapodPoseHoldServoHingeEarlyOutError = 0.0011f;
inline constexpr float kHexapodPoseHoldServoHingeEarlyOutSpeed = 0.045f;
/// Slightly reduce aggressive omega_n and add damping to limit chatter / residual velocity at 20 iters.
inline constexpr float kHexapodPoseHoldServoPositionGainScale = 0.70f;
inline constexpr float kHexapodPoseHoldServoDampingGainScale = 1.50f;
inline constexpr float kHexapodPoseHoldServoMaxSpeedScale = 0.45f;
/// Small post-step hinge snap (Relax zeros this; re-enable for under-converged velocity motors).
inline constexpr float kHexapodPoseHoldAngleStabScale = 0.10f;

/// Same metrics as `test_hexapod_live_pose_hold` — useful for iteration / tuning sweeps.
struct HexapodPoseHoldMetrics {
    /// Max |v| for the chassis over the whole run (includes landing / initial transients).
    float peakLinear = 0.0f;
    /// Max |v| for the chassis only for outer_frame_index >= kHexapodPoseHoldSettledStartFrame.
    float peakLinearSettled = 0.0f;
    float peakAngular = 0.0f;
    float peakJointErrorRad = 0.0f;
    float finalSpeed = 0.0f;
    Vec3 finalPosition{};
};

inline float MaxHexapodServoAngleErrorRads(const World& world, const HexapodSceneObjects& scene) {
    float err = 0.0f;
    for (const LegLinkIds& leg : scene.legs) {
        for (const std::uint32_t joint_id :
             {leg.bodyToCoxaJoint, leg.coxaToFemurJoint, leg.femurToTibiaJoint}) {
            const float a = world.GetServoJointAngle(joint_id);
            const float t = world.GetServoJoint(joint_id).targetAngle;
            err = std::max(err, std::abs(WrapHexapodAngle(a - t)));
        }
    }
    return err;
}

inline void AccumulateHexapodPoseHoldFromSubstep(
    const World& world,
    const HexapodSceneObjects& scene,
    std::uint32_t chassis_id,
    int outer_frame_index,
    HexapodPoseHoldMetrics& m) {
    const Body& chassis = world.GetBody(chassis_id);
    const float linear_speed = Length(chassis.velocity);
    m.peakLinear = std::max(m.peakLinear, linear_speed);
    if (outer_frame_index >= kHexapodPoseHoldSettledStartFrame) {
        m.peakLinearSettled = std::max(m.peakLinearSettled, linear_speed);
    }
    m.peakAngular = std::max(m.peakAngular, Length(chassis.angularVelocity));
    m.peakJointErrorRad = std::max(m.peakJointErrorRad, MaxHexapodServoAngleErrorRads(world, scene));
}

inline void FinalizeHexapodPoseHold(const Body& chassis, HexapodPoseHoldMetrics& m) {
    m.finalPosition = chassis.position;
    m.finalSpeed = Length(chassis.velocity);
}

inline std::array<std::uint32_t, 18> HexapodServoJointIds(const HexapodSceneObjects& scene) {
    return {
        scene.legs[0].bodyToCoxaJoint,
        scene.legs[0].coxaToFemurJoint,
        scene.legs[0].femurToTibiaJoint,
        scene.legs[1].bodyToCoxaJoint,
        scene.legs[1].coxaToFemurJoint,
        scene.legs[1].femurToTibiaJoint,
        scene.legs[2].bodyToCoxaJoint,
        scene.legs[2].coxaToFemurJoint,
        scene.legs[2].femurToTibiaJoint,
        scene.legs[3].bodyToCoxaJoint,
        scene.legs[3].coxaToFemurJoint,
        scene.legs[3].femurToTibiaJoint,
        scene.legs[4].bodyToCoxaJoint,
        scene.legs[4].coxaToFemurJoint,
        scene.legs[4].femurToTibiaJoint,
        scene.legs[5].bodyToCoxaJoint,
        scene.legs[5].coxaToFemurJoint,
        scene.legs[5].femurToTibiaJoint,
    };
}

/// Call after `BuildHexapodScene` + `RelaxBuiltInHexapodServos`. Tunes joint solver + per-servo fields.
inline void ApplyHexapodPoseHoldStabilityTuning(World& world, const HexapodSceneObjects& scene) {
    JointSolverConfig j = world.GetJointSolverConfig();
    j.servoPositionPasses = kHexapodPoseHoldJointServoPositionPasses;
    j.servoPositionSolveStride = static_cast<std::uint8_t>(kHexapodPoseHoldJointServoPositionStride);
    j.hingeAnchorBiasFactor = kHexapodPoseHoldHingeAnchorBias;
    j.hingeAnchorDampingFactor = kHexapodPoseHoldHingeAnchorDamping;
    j.enableServoAngularEarlyOut = false;
    j.enableServoHingeEarlyOut = false;
    j.servoEarlyOutResumeScale = kHexapodPoseHoldServoEarlyOutResumeScale;
    j.servoEarlyOutImpulseGuardFraction = kHexapodPoseHoldServoEarlyOutImpulseGuard;
    j.servoAngularEarlyOutError = kHexapodPoseHoldServoAngularEarlyOutError;
    j.servoAngularEarlyOutSpeed = kHexapodPoseHoldServoAngularEarlyOutSpeed;
    j.servoHingeEarlyOutError = kHexapodPoseHoldServoHingeEarlyOutError;
    j.servoHingeEarlyOutSpeed = kHexapodPoseHoldServoHingeEarlyOutSpeed;
    world.SetJointSolverConfig(j);
    for (const std::uint32_t id : HexapodServoJointIds(scene)) {
        ServoJoint& sj = world.GetServoJointMutable(id);
        sj.positionGain *= kHexapodPoseHoldServoPositionGainScale;
        sj.dampingGain *= kHexapodPoseHoldServoDampingGainScale;
        sj.maxServoSpeed *= kHexapodPoseHoldServoMaxSpeedScale;
        sj.angleStabilizationScale = kHexapodPoseHoldAngleStabScale;
    }
}

/// Per outer-frame max joint rate (|Δangle|/frame_dt) for all 18 servos; updates `angles_prev` in place.
inline float MaxHexapodJointSpeedRadSFrame(
    const World& world,
    const HexapodSceneObjects& scene,
    std::array<float, 18>& angles_prev,
    float frame_dt) {
    float max_speed = 0.0f;
    std::size_t i = 0;
    for (const std::uint32_t joint_id : HexapodServoJointIds(scene)) {
        const float a = world.GetServoJointAngle(joint_id);
        const float w = (a - angles_prev[i]) / frame_dt;
        max_speed = std::max(max_speed, std::abs(w));
        angles_prev[i] = a;
        ++i;
    }
    return max_speed;
}

inline std::string FormatHexapodStabilityRecord(
    const HexapodPoseHoldMetrics& pose,
    const HexapodStandingStats& standing,
    const char* git_hash) {
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(6);
    if (git_hash != nullptr) {
        o << "git=" << git_hash << '\n';
    }
    o << "[pose_hold]\n"
      << "pose_hold_settled_start_frame=" << kHexapodPoseHoldSettledStartFrame << '\n'
      << "peak_linear_speed=" << pose.peakLinear << '\n'
      << "peak_linear_settled_speed=" << pose.peakLinearSettled << '\n'
      << "peak_angular_speed=" << pose.peakAngular << '\n'
      << "peak_joint_error_rad=" << pose.peakJointErrorRad << '\n'
      << "final_chassis_y=" << pose.finalPosition.y << '\n'
      << "final_speed=" << pose.finalSpeed << '\n'
      << "[standing_window]\n"
      << "min_body_height=" << standing.minBodyHeight << '\n'
      << "max_body_height=" << standing.maxBodyHeight << '\n'
      << "max_abs_roll_deg=" << standing.maxAbsRollDeg << '\n'
      << "max_abs_pitch_deg=" << standing.maxAbsPitchDeg << '\n'
      << "max_joint_speed_rad_s=" << standing.maxJointSpeedRadS << '\n'
      << "min_contact_manifolds=" << standing.minContactManifolds << '\n'
      << "min_contact_points=" << standing.minContactPoints << '\n';
    return o.str();
}

} // namespace minphys3d::demo
