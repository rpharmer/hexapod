#include "robot_runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

#include "geometry_config.hpp"
#include "foot_terrain.hpp"
#include "local_map.hpp"
#include "plane_estimation.hpp"
#include "replay_json.hpp"
#include "physics_sim_bridge.hpp"

namespace {

enum class PhysicsSimCorrectionMode {
    None = 0,
    Soft,
    Strong,
    HardReset,
};

const char* correctionModeName(const PhysicsSimCorrectionMode mode) {
    switch (mode) {
        case PhysicsSimCorrectionMode::Soft:
            return "soft";
        case PhysicsSimCorrectionMode::Strong:
            return "strong";
        case PhysicsSimCorrectionMode::HardReset:
            return "hard_reset";
        case PhysicsSimCorrectionMode::None:
        default:
            return "none";
    }
}

telemetry::FusionCorrectionMode toTelemetryCorrectionMode(const PhysicsSimCorrectionMode mode) {
    switch (mode) {
        case PhysicsSimCorrectionMode::Soft:
            return telemetry::FusionCorrectionMode::Soft;
        case PhysicsSimCorrectionMode::Strong:
            return telemetry::FusionCorrectionMode::Strong;
        case PhysicsSimCorrectionMode::HardReset:
            return telemetry::FusionCorrectionMode::HardReset;
        case PhysicsSimCorrectionMode::None:
        default:
            return telemetry::FusionCorrectionMode::None;
    }
}

physics_sim::ContactPhase toPhysicsSimPhase(const ContactPhase phase) {
    switch (phase) {
        case ContactPhase::Swing:
            return physics_sim::ContactPhase::Swing;
        case ContactPhase::ExpectedTouchdown:
            return physics_sim::ContactPhase::ExpectedTouchdown;
        case ContactPhase::ContactCandidate:
            return physics_sim::ContactPhase::ContactCandidate;
        case ContactPhase::ConfirmedStance:
            return physics_sim::ContactPhase::ConfirmedStance;
        case ContactPhase::LostCandidate:
            return physics_sim::ContactPhase::LostCandidate;
        case ContactPhase::Search:
        default:
            return physics_sim::ContactPhase::Search;
    }
}

const char* contactPhaseName(const ContactPhase phase) {
    switch (phase) {
        case ContactPhase::Swing:
            return "swing";
        case ContactPhase::ExpectedTouchdown:
            return "expected_touchdown";
        case ContactPhase::ContactCandidate:
            return "contact_candidate";
        case ContactPhase::ConfirmedStance:
            return "confirmed_stance";
        case ContactPhase::LostCandidate:
            return "lost_candidate";
        case ContactPhase::Search:
        default:
            return "search";
    }
}

Vec3 serverToSimVec(const Vec3& v) {
    return Vec3{v.y, v.z, -v.x};
}

std::array<float, 4> mat3ToQuaternion(const Mat3& m) {
    const double trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (m.m[2][1] - m.m[1][2]) / s;
        y = (m.m[0][2] - m.m[2][0]) / s;
        z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        const double s = std::sqrt(1.0 + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0;
        w = (m.m[2][1] - m.m[1][2]) / s;
        x = 0.25 * s;
        y = (m.m[0][1] + m.m[1][0]) / s;
        z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        const double s = std::sqrt(1.0 + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0;
        w = (m.m[0][2] - m.m[2][0]) / s;
        x = (m.m[0][1] + m.m[1][0]) / s;
        y = 0.25 * s;
        z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0;
        w = (m.m[1][0] - m.m[0][1]) / s;
        x = (m.m[0][2] + m.m[2][0]) / s;
        y = (m.m[1][2] + m.m[2][1]) / s;
        z = 0.25 * s;
    }
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm <= 0.0) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    return {static_cast<float>(w / norm),
            static_cast<float>(x / norm),
            static_cast<float>(y / norm),
            static_cast<float>(z / norm)};
}

BodyPose makeBodyPose(const RobotState& est) {
    BodyPose pose{};
    if (!est.has_body_twist_state) {
        return pose;
    }
    pose.position = est.body_twist_state.body_trans_m;
    pose.yaw = AngleRad{est.body_twist_state.twist_pos_rad.z};
    pose.pitch = AngleRad{est.body_twist_state.twist_pos_rad.y};
    pose.roll = AngleRad{est.body_twist_state.twist_pos_rad.x};
    return pose;
}

bool solveTerrainPlane(const std::array<Vec3, kNumLegs>& points,
                       const std::array<bool, kNumLegs>& valid,
                       double& a,
                       double& b,
                       double& c) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_xx = 0.0;
    double sum_yy = 0.0;
    double sum_xy = 0.0;
    double sum_xz = 0.0;
    double sum_yz = 0.0;
    int n = 0;

    for (int i = 0; i < kNumLegs; ++i) {
        if (!valid[i]) {
            continue;
        }
        const Vec3& p = points[static_cast<std::size_t>(i)];
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        sum_xx += p.x * p.x;
        sum_yy += p.y * p.y;
        sum_xy += p.x * p.y;
        sum_xz += p.x * p.z;
        sum_yz += p.y * p.z;
        ++n;
    }

    if (n < 3) {
        return false;
    }

    const double dn = static_cast<double>(n);
    const double m00 = sum_xx;
    const double m01 = sum_xy;
    const double m02 = sum_x;
    const double m10 = sum_xy;
    const double m11 = sum_yy;
    const double m12 = sum_y;
    const double m20 = sum_x;
    const double m21 = sum_y;
    const double m22 = dn;

    const double det = m00 * (m11 * m22 - m12 * m21) -
                       m01 * (m10 * m22 - m12 * m20) +
                       m02 * (m10 * m21 - m11 * m20);
    if (std::abs(det) < 1e-8) {
        return false;
    }

    const double r0 = sum_xz;
    const double r1 = sum_yz;
    const double r2 = sum_z;

    const double det_a = r0 * (m11 * m22 - m12 * m21) -
                         m01 * (r1 * m22 - m12 * r2) +
                         m02 * (r1 * m21 - m11 * r2);
    const double det_b = m00 * (r1 * m22 - m12 * r2) -
                         r0 * (m10 * m22 - m12 * m20) +
                         m02 * (m10 * r2 - r1 * m20);
    const double det_c = m00 * (m11 * r2 - r1 * m21) -
                         m01 * (m10 * r2 - r1 * m20) +
                         r0 * (m10 * m21 - m11 * m20);

    a = det_a / det;
    b = det_b / det;
    c = det_c / det;
    return true;
}

struct PhysicsSimCorrectionPacket {
    physics_sim::StateCorrection packet{};
    PhysicsSimCorrectionMode mode{PhysicsSimCorrectionMode::None};
};

PhysicsSimCorrectionPacket buildPhysicsSimCorrection(const RobotState& est,
                                                     TimePointUs now,
                                                     const LocalMapSnapshot* terrain_snapshot,
                                                     const control_config::FusionConfig& fusion_cfg,
                                                     const PhysicsSimCorrectionMode last_mode,
                                                     const uint64_t last_mode_sample_id) {
    PhysicsSimCorrectionPacket out{};
    out.packet.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    out.packet.sequence_id = est.sample_id != 0 ? static_cast<std::uint32_t>(est.sample_id) : 0;
    out.packet.timestamp_us = now.value;

    const bool has_pose = est.has_body_twist_state;
    if (has_pose) {
        const Vec3 pos_sim = serverToSimVec(est.body_twist_state.body_trans_m.raw());
        const Vec3 vel_sim = serverToSimVec(est.body_twist_state.body_trans_mps.raw());
        const BodyPose body_pose = makeBodyPose(est);
        const Mat3 server_rot = body_pose.rotationBodyToWorld();
        Mat3 sim_to_server{};
        sim_to_server.m[0][0] = 0.0;
        sim_to_server.m[0][1] = 0.0;
        sim_to_server.m[0][2] = -1.0;
        sim_to_server.m[1][0] = 1.0;
        sim_to_server.m[1][1] = 0.0;
        sim_to_server.m[1][2] = 0.0;
        sim_to_server.m[2][0] = 0.0;
        sim_to_server.m[2][1] = 1.0;
        sim_to_server.m[2][2] = 0.0;
        const Mat3 server_to_sim = sim_to_server.transpose();
        const Mat3 sim_rot = server_to_sim * server_rot * sim_to_server;

        out.packet.flags |= physics_sim::kStateCorrectionPoseValid;
        out.packet.flags |= physics_sim::kStateCorrectionTwistValid;
        out.packet.body_position = {static_cast<float>(pos_sim.x),
                                    static_cast<float>(pos_sim.y),
                                    static_cast<float>(pos_sim.z)};
        out.packet.body_orientation = mat3ToQuaternion(sim_rot);
        out.packet.body_linear_velocity = {static_cast<float>(vel_sim.x),
                                           static_cast<float>(vel_sim.y),
                                           static_cast<float>(vel_sim.z)};
        const Vec3 ang_vel_sim = serverToSimVec(est.body_twist_state.twist_vel_radps.raw());
        out.packet.body_angular_velocity = {static_cast<float>(ang_vel_sim.x),
                                            static_cast<float>(ang_vel_sim.y),
                                            static_cast<float>(ang_vel_sim.z)};
    }

    bool have_contact = false;
    for (std::size_t leg = 0; leg < est.foot_contact_fusion.size(); ++leg) {
        const FootContactFusion& fc = est.foot_contact_fusion[leg];
        out.packet.foot_contact_phase[leg] = static_cast<std::uint8_t>(toPhysicsSimPhase(fc.phase));
        out.packet.foot_contact_confidence[leg] = std::clamp(fc.confidence, 0.0f, 1.0f);
        if (fc.confidence > 0.0f || fc.phase != ContactPhase::Search) {
            have_contact = true;
        }
    }
    if (have_contact) {
        out.packet.flags |= physics_sim::kStateCorrectionContactValid;
    }

    bool have_terrain = false;
    std::array<Vec3, kNumLegs> terrain_points_world{};
    std::array<bool, kNumLegs> valid_points{};
    std::array<double, kNumLegs> foot_ground_height{};
    std::array<float, kNumLegs> foot_ground_confidence{};
    foot_ground_height.fill(std::numeric_limits<double>::quiet_NaN());
    foot_ground_confidence.fill(0.0f);

    if (terrain_snapshot && terrain_snapshot->fresh && terrain_snapshot->ground_elevation_has_data &&
        has_pose) {
        const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
        const BodyPose body_pose = makeBodyPose(est);
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            const Vec3 foot_world = computeFootWorldFromServoAngles(
                est.leg_states[leg], geometry.legGeometry[leg], body_pose);
            const double ground_z =
                sampleGroundMeanZWorldM(*terrain_snapshot, foot_world.x, foot_world.y);
            if (!std::isfinite(ground_z)) {
                continue;
            }
            valid_points[leg] = true;
            terrain_points_world[leg] = Vec3{foot_world.x, foot_world.y, ground_z};
            foot_ground_height[leg] = ground_z;
            const FootContactFusion& fc = est.foot_contact_fusion[leg];
            const double phase_confidence =
                std::clamp(static_cast<double>(fc.confidence), 0.0, 1.0);
            const double terrain_confidence = terrain_snapshot->has_primary_observations ? 1.0 : 0.6;
            foot_ground_confidence[leg] = static_cast<float>(
                std::clamp(0.35 + 0.65 * phase_confidence * terrain_confidence, 0.0, 1.0));
        }

        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        if (solveTerrainPlane(terrain_points_world, valid_points, a, b, c)) {
            const Vec3 normal_server{-a, -b, 1.0};
            const Vec3 normal_sim = serverToSimVec(normal_server);
            const double norm = vecNorm(normal_sim);
            if (norm > 1e-9) {
                out.packet.terrain_normal = {
                    static_cast<float>(normal_sim.x / norm),
                    static_cast<float>(normal_sim.y / norm),
                    static_cast<float>(normal_sim.z / norm),
                };
            }
            out.packet.terrain_height_m = static_cast<float>(c);
            have_terrain = true;
        } else {
            double weighted_sum = 0.0;
            double weight_total = 0.0;
            for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
                if (!valid_points[leg]) {
                    continue;
                }
                const FootContactFusion& fc = est.foot_contact_fusion[leg];
                const double weight = std::clamp(static_cast<double>(fc.confidence), 0.0, 1.0) *
                                      (fc.phase == ContactPhase::ConfirmedStance ? 1.0 : 0.5);
                if (weight <= 0.0) {
                    continue;
                }
                weighted_sum += weight * foot_ground_height[leg];
                weight_total += weight;
            }
            if (weight_total > 0.0) {
                out.packet.terrain_height_m = static_cast<float>(weighted_sum / weight_total);
                have_terrain = true;
            }
        }

        if (have_terrain) {
            out.packet.flags |= physics_sim::kStateCorrectionTerrainValid;
        }

        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            out.packet.foot_ground_height_m[leg] = static_cast<float>(foot_ground_height[leg]);
            out.packet.foot_ground_confidence[leg] = foot_ground_confidence[leg];
        }
    }

    const bool hard_reset = est.has_fusion_diagnostics && est.fusion.hard_reset_requested;
    const bool strong = est.has_fusion_diagnostics && est.fusion.resync_requested && !hard_reset;
    const double model_trust = est.has_fusion_diagnostics ? est.fusion.model_trust : 1.0;
    const bool soft_residual =
        est.has_fusion_diagnostics &&
        (est.fusion.residuals.max_body_position_error_m > 0.5 * fusion_cfg.soft_pose_resync_m ||
         est.fusion.residuals.max_body_orientation_error_rad >
             0.5 * fusion_cfg.soft_orientation_resync_rad);

    PhysicsSimCorrectionMode selected_mode = PhysicsSimCorrectionMode::None;
    if (hard_reset) {
        selected_mode = PhysicsSimCorrectionMode::HardReset;
        out.packet.flags |= physics_sim::kStateCorrectionHardReset;
        out.packet.correction_strength = 1.0f;
        if (est.has_fusion_diagnostics) {
            out.packet.flags |= physics_sim::kStateCorrectionRelocalize;
        }
    } else if (strong) {
        selected_mode = PhysicsSimCorrectionMode::Strong;
        out.packet.flags |= physics_sim::kStateCorrectionRelocalize;
        out.packet.correction_strength =
            static_cast<float>(std::clamp(0.68 + 0.25 * (1.0 - model_trust), 0.68, 0.95));
    } else if (soft_residual) {
        selected_mode = PhysicsSimCorrectionMode::Soft;
        out.packet.correction_strength =
            static_cast<float>(std::clamp(0.32 + 0.28 * (1.0 - model_trust), 0.32, 0.60));
    } else {
        selected_mode = PhysicsSimCorrectionMode::None;
        out.packet.correction_strength = static_cast<float>(std::clamp(0.20 + 0.15 * (1.0 - model_trust),
                                                                        0.20,
                                                                        0.50));
    }

    const uint64_t sample_age =
        (est.sample_id > last_mode_sample_id) ? (est.sample_id - last_mode_sample_id) : 0;
    const bool within_hold = fusion_cfg.correction_mode_hold_samples > 0 &&
                             sample_age > 0 &&
                             sample_age < static_cast<uint64_t>(fusion_cfg.correction_mode_hold_samples);
    const bool strong_release_guard =
        est.has_fusion_diagnostics &&
        (est.fusion.residuals.max_body_position_error_m >
             fusion_cfg.soft_pose_resync_m * fusion_cfg.correction_mode_strong_release_factor ||
         est.fusion.residuals.max_body_orientation_error_rad >
             fusion_cfg.soft_orientation_resync_rad * fusion_cfg.correction_mode_strong_release_factor);
    const bool soft_release_guard =
        est.has_fusion_diagnostics &&
        (est.fusion.residuals.max_body_position_error_m >
             fusion_cfg.soft_pose_resync_m * fusion_cfg.correction_mode_soft_release_factor ||
         est.fusion.residuals.max_body_orientation_error_rad >
             fusion_cfg.soft_orientation_resync_rad * fusion_cfg.correction_mode_soft_release_factor);

    if (selected_mode == PhysicsSimCorrectionMode::None) {
        if (last_mode == PhysicsSimCorrectionMode::Strong && (within_hold || strong_release_guard)) {
            selected_mode = PhysicsSimCorrectionMode::Strong;
            out.packet.correction_strength = std::max(out.packet.correction_strength, 0.68f);
        } else if (last_mode == PhysicsSimCorrectionMode::Soft && (within_hold || soft_release_guard)) {
            selected_mode = PhysicsSimCorrectionMode::Soft;
            out.packet.correction_strength = std::max(out.packet.correction_strength, 0.32f);
        }
    } else if (selected_mode == PhysicsSimCorrectionMode::Soft &&
               last_mode == PhysicsSimCorrectionMode::Strong &&
               (within_hold || strong_release_guard)) {
        selected_mode = PhysicsSimCorrectionMode::Strong;
        out.packet.correction_strength =
            std::max(out.packet.correction_strength, static_cast<float>(0.68f));
        out.packet.flags |= physics_sim::kStateCorrectionRelocalize;
    }

    out.mode = selected_mode;
    if (out.packet.correction_strength < 0.0f) {
        out.packet.correction_strength = 0.0f;
    }
    if (!has_pose && !have_contact && !have_terrain) {
        out.mode = PhysicsSimCorrectionMode::None;
    }
    return out;
}

bool maybeSendPhysicsSimCorrection(IHardwareBridge* bridge,
                                   const physics_sim::StateCorrection& packet) {
    auto* physics_sim_bridge = dynamic_cast<PhysicsSimBridge*>(bridge);
    if (!physics_sim_bridge) {
        return false;
    }
    return physics_sim_bridge->sendStateCorrection(packet);
}

JointTargets clampJointTargetsToServoDynamics(const JointTargets& previous,
                                              const JointTargets& requested,
                                              const HexapodGeometry& geometry,
                                              const double dt_s) {
    if (dt_s <= 0.0) {
        return requested;
    }

    JointTargets limited = requested;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geometry = geometry.legGeometry[leg];
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const AngleRad prev = previous.leg_states[leg].joint_state[joint].pos_rad;
            const AngleRad req = requested.leg_states[leg].joint_state[joint].pos_rad;
            const double error = req.value - prev.value;
            const ServoJointDynamics& dynamics = leg_geometry.servoDynamics[joint];
            const ServoDirectionDynamics& direction =
                (error >= 0.0) ? dynamics.positive_direction : dynamics.negative_direction;
            const double max_delta = std::max(direction.vmax_radps, 0.0) * dt_s;
            double limited_error = error;
            if (max_delta > 0.0) {
                limited_error = std::clamp(error, -max_delta, max_delta);
            }

            limited.leg_states[leg].joint_state[joint].pos_rad = AngleRad{prev.value + limited_error};
            limited.leg_states[leg].joint_state[joint].vel_radps =
                AngularRateRadPerSec{limited_error / dt_s};
        }
    }
    return limited;
}

replay_json::ReplayTransitionDiagnostics buildReplayTransitionDiagnostics(const RobotState& estimated,
                                                                         const GaitState& gait_state,
                                                                         const JointTargets& joint_targets) {
    replay_json::ReplayTransitionDiagnostics diagnostics{};
    if (estimated.has_body_twist_state) {
        diagnostics.body_height_m = estimated.body_twist_state.body_trans_m.z;
    }

    for (std::size_t leg = 0; leg < gait_state.in_stance.size(); ++leg) {
        diagnostics.stance_leg_count += gait_state.in_stance[leg] ? 1 : 0;
        diagnostics.contact_leg_count += estimated.foot_contacts[leg] ? 1 : 0;
        if (gait_state.in_stance[leg] != estimated.foot_contacts[leg]) {
            ++diagnostics.stance_contact_mismatch_count;
        }

        double sum_sq = 0.0;
        double max_abs = 0.0;
        for (std::size_t joint = 0; joint < joint_targets.leg_states[leg].joint_state.size(); ++joint) {
            const double commanded = joint_targets.leg_states[leg].joint_state[joint].pos_rad.value;
            const double measured = estimated.leg_states[leg].joint_state[joint].pos_rad.value;
            const double error = commanded - measured;
            sum_sq += error * error;
            max_abs = std::max(max_abs, std::abs(error));
        }
        const double denom = static_cast<double>(joint_targets.leg_states[leg].joint_state.size());
        diagnostics.joint_tracking_rms_error_rad[leg] = denom > 0.0 ? std::sqrt(sum_sq / denom) : 0.0;
        diagnostics.joint_tracking_max_abs_error_rad[leg] = max_abs;
    }

    return diagnostics;
}

} // namespace

RobotRuntime::RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config,
                           std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher,
                           std::unique_ptr<replay::IReplayLogger> replay_logger)
    : hw_(std::move(hw)),
      estimator_(std::move(estimator)),
      logger_(std::move(logger)),
      config_(config),
      telemetry_publisher_(std::move(telemetry_publisher)),
      replay_logger_(std::move(replay_logger)),
      pipeline_(config_.gait,
                config_.locomotion_cmd,
                config_.foot_terrain,
                config_.investigation,
                &resource_profiler_),
      safety_(config_.safety),
      freshness_policy_(config_.freshness),
      freshness_gate_(freshness_policy_),
      timing_metrics_(config_, control_dt_sum_us_, control_jitter_max_us_),
      diagnostics_reporter_(logger_, freshness_policy_) {}

bool RobotRuntime::init() {
    if (!hw_ || !estimator_) {
        if (logger_) {
            LOG_ERROR(logger_, "Missing components");
        }
        return false;
    }
    if (!hw_->init()) {
        if (logger_) {
            LOG_ERROR(logger_, "Hardware Bridge init failed");
        }
        return false;
    }

    MotionIntent initial{};
    initial.requested_mode = RobotMode::SAFE_IDLE;
    initial.timestamp_us = now_us();
    initial.sample_id = 1;

    motion_intent_.write(initial);
    status_.write(ControlStatus{});
    safety_state_.write(SafetyState{});
    leg_targets_.write(LegTargets{});
    gait_state_.write(GaitState{});
    joint_targets_.write(JointTargets{});
    control_loop_counter_.store(0);
    control_dt_sum_us_.store(0);
    control_jitter_max_us_.store(0);
    stale_intent_count_.store(0);
    stale_estimator_count_.store(0);
    raw_sample_seq_.store(0);
    intent_sample_seq_.store(1);
    freshness_gate_.reset();
    timing_metrics_.reset();
    next_telemetry_publish_at_ = TimePointUs{};
    next_geometry_refresh_at_ = TimePointUs{};
    last_effective_intent_estimator_sample_id_ = 0;
    last_fusion_reset_sample_id_ = 0;
    last_fusion_correction_sample_id_ = 0;
    last_logged_safety_fault_ = FaultCode::NONE;
    last_fusion_correction_mode_ = static_cast<std::uint8_t>(PhysicsSimCorrectionMode::None);
    last_fusion_correction_ = {};
    last_process_resources_.reset();
    last_resource_sections_.reset();
    resource_profiler_.reset();
    if (estimator_) {
        estimator_->configure(config_.fusion);
        estimator_->reset();
    }
    pipeline_.reset();

    return true;
}

void RobotRuntime::startTelemetry() {
    if (!telemetry_publisher_ || !config_.telemetry.enabled) {
        return;
    }

    telemetry_publisher_->publishGeometry(geometry_config::activeHexapodGeometry());

    const TimePointUs now = now_us();
    next_telemetry_publish_at_ = now;
    next_geometry_refresh_at_ = TimePointUs{
        now.value + static_cast<uint64_t>(config_.telemetry.geometry_refresh_period.count()) * 1000ULL};
}

void RobotRuntime::busStep() {
    RobotState raw{};
    {
        const auto read_scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::BusRead));
        if (!hw_->read(raw)) {
            raw.bus_ok = false;
        }
        (void)read_scope;
    }
    if (raw.sample_id == 0) {
        raw.sample_id = raw_sample_seq_.fetch_add(1) + 1;
    }
    raw_state_.write(raw);

    const JointTargets cmd = joint_targets_.read();
    {
        const auto write_scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::BusWrite));
        (void)hw_->write(cmd);
        (void)write_scope;
    }
}

void RobotRuntime::estimatorStep() {
    const auto scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::EstimatorUpdate));
    const RobotState raw = raw_state_.read();
    const RobotState est = estimator_->update(raw);
    const TimePointUs now = now_us();
    if (logger_ && (last_estimator_snapshot_log_us_.isZero() ||
                    (now.value > last_estimator_snapshot_log_us_.value &&
                     now.value - last_estimator_snapshot_log_us_.value >= 1'000'000ULL))) {
        std::ostringstream snapshot;
        snapshot << "estimator_snapshot raw=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                snapshot << ',';
            }
            snapshot << (raw.foot_contacts[leg] ? '1' : '0');
        }
        snapshot << "] fused=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                snapshot << ',';
            }
            snapshot << (est.foot_contacts[leg] ? '1' : '0');
        }
        snapshot << "] phase=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                snapshot << ',';
            }
            snapshot << contactPhaseName(est.foot_contact_fusion[leg].phase);
        }
        snapshot << "] confidence=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                snapshot << ',';
            }
            snapshot << est.foot_contact_fusion[leg].confidence;
        }
        int raw_contact_count = 0;
        int fused_contact_count = 0;
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            raw_contact_count += raw.foot_contacts[leg] ? 1 : 0;
            fused_contact_count += est.foot_contacts[leg] ? 1 : 0;
        }
        snapshot << "] raw_contact_count=" << raw_contact_count
                 << " fused_contact_count=" << fused_contact_count
                 << " model_trust=" << est.fusion.model_trust
                 << " has_fusion_diagnostics=" << (est.has_fusion_diagnostics ? 1 : 0)
                 << " has_body_twist_state=" << (est.has_body_twist_state ? 1 : 0)
                 << " residual_contact_mismatch=" << est.fusion.residuals.contact_mismatch_ratio
                 << " residual_terrain_m=" << est.fusion.residuals.terrain_residual_m
                 << " residual_pos_m=" << est.fusion.residuals.max_body_position_error_m
                 << " residual_ori_rad=" << est.fusion.residuals.max_body_orientation_error_rad
                 << " raw_sid=" << raw.sample_id
                 << " est_sid=" << est.sample_id
                 << " raw_ts_us=" << raw.timestamp_us.value
                 << " est_ts_us=" << est.timestamp_us.value;
        LOG_INFO(logger_, snapshot.str());
        last_estimator_snapshot_log_us_ = now;
    }
    estimated_state_.write(est);
    (void)scope;
}

void RobotRuntime::controlStep() {
    const auto step_scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlStep));
    const TimePointUs now = now_us();
    timing_metrics_.update(now);
    const JointTargets previous_joint_targets = joint_targets_.read();

    const RobotState est = estimated_state_.read();
    if (navigation_manager_ != nullptr) {
        navigation_manager_->refreshTerrainSnapshot(est, now);
    }
    const LocalMapSnapshot* terrain_ptr = nullptr;
    if (navigation_manager_ != nullptr) {
        terrain_ptr = navigation_manager_->footTerrainSnapshot(now);
    }
    if (config_.investigation.bypass_terrain_snapshot_in_runtime) {
        terrain_ptr = nullptr;
    }
    const FusionControlPolicy fusion_policy = applyFusionConsistency(est, now, terrain_ptr);
    MotionIntent intent = resolveEffectiveIntent(est, now);
    if (fusion_policy.force_stand) {
        intent.requested_mode = RobotMode::STAND;
        intent.speed_mps = LinearRateMps{};
        intent.cmd_vx_mps = LinearRateMps{};
        intent.cmd_vy_mps = LinearRateMps{};
        intent.cmd_yaw_radps = AngularRateRadPerSec{};
        intent.twist.body_trans_m.x = 0.0;
        intent.twist.body_trans_m.y = 0.0;
        intent.twist.twist_vel_radps.x = 0.0;
        intent.twist.twist_vel_radps.y = 0.0;
        intent.twist.twist_vel_radps.z = 0.0;
    } else if (intent.requested_mode == RobotMode::WALK &&
               fusion_policy.aggressiveness_scale < 0.999) {
        scaleMotionIntent(intent, fusion_policy.aggressiveness_scale);
    }
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    uint64_t loop_counter = 0;
    RuntimeFreshnessGate::Decision decision{};
    {
        const auto freshness_scope =
            resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlFreshnessGate));
        const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
            RuntimeFreshnessGate::EvaluationMode::StrictControl, now, est, intent);
        freshness_gate_.recordStrictMetrics(freshness, stale_intent_count_, stale_estimator_count_);

        loop_counter = control_loop_counter_.fetch_add(1) + 1;
        decision = freshness_gate_.computeControlDecision(freshness, bus_ok, loop_counter);
        if (!decision.allow_pipeline && !config_.investigation.bypass_freshness_gate_reject) {
            RuntimeFreshnessGate::maybeLogReject(logger_, freshness, est, intent);
            status_.write(decision.status);
            leg_targets_.write(LegTargets{});
            gait_state_.write(GaitState{});
            joint_targets_.write(decision.joint_targets);
            maybePublishTelemetry(now);
            maybeWriteReplayRecord(now);
            (void)freshness_scope;
            (void)step_scope;
            return;
        }
        if (!decision.allow_pipeline && config_.investigation.bypass_freshness_gate_reject && logger_) {
            LOG_WARN(logger_,
                     "investigation bypass active: ignoring freshness gate reject and running control pipeline");
        }
        (void)freshness_scope;
    }

    const PipelineStepResult result = pipeline_.runStep(
        est,
        intent,
        safety_state,
        bus_ok,
        loop_counter,
        terrain_ptr);

    JointTargets joint_targets = result.joint_targets;
    if (intent.requested_mode == RobotMode::WALK) {
        const double control_dt_s =
            std::max(static_cast<double>(config_.loop_timing.control_loop_period.count()) * 1.0e-6, 1.0e-6);
        joint_targets = clampJointTargetsToServoDynamics(
            previous_joint_targets,
            joint_targets,
            geometry_config::activeHexapodGeometry(),
            control_dt_s);
    }

    leg_targets_.write(result.leg_targets);
    gait_state_.write(result.gait_state);
    joint_targets_.write(joint_targets);
    status_.write(result.status);

    if (logger_) {
        std::ostringstream support_snapshot;
        support_snapshot << "stance_support_snapshot clearance_m=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                support_snapshot << ',';
            }
            support_snapshot << result.gait_state.support_liftoff_clearance_m[leg];
        }
        support_snapshot << "] safe_to_lift=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                support_snapshot << ',';
            }
            support_snapshot << (result.gait_state.support_liftoff_safe_to_lift[leg] ? '1' : '0');
        }
        support_snapshot << "] static_margin_m=" << result.gait_state.static_stability_margin_m
                         << " hold_stance=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                support_snapshot << ',';
            }
            support_snapshot << (result.gait_state.stability_hold_stance[leg] ? '1' : '0');
        }
        support_snapshot << ']';
        LOG_INFO(logger_, support_snapshot.str());
    }

    maybePublishTelemetry(now);
    maybeWriteReplayRecord(now);
    (void)step_scope;
}

void RobotRuntime::maybePublishTelemetry(const TimePointUs& now) {
    if (!telemetry_publisher_ || !config_.telemetry.enabled) {
        return;
    }

    if (now.value < next_telemetry_publish_at_.value) {
        return;
    }

    const uint64_t publish_period_us =
        static_cast<uint64_t>(config_.telemetry.publish_period.count()) * 1000ULL;
    next_telemetry_publish_at_ = TimePointUs{now.value + publish_period_us};

    if (now.value >= next_geometry_refresh_at_.value) {
        telemetry_publisher_->publishGeometry(geometry_config::activeHexapodGeometry());
        const uint64_t geometry_refresh_period_us =
            static_cast<uint64_t>(config_.telemetry.geometry_refresh_period.count()) * 1000ULL;
        next_geometry_refresh_at_ = TimePointUs{now.value + geometry_refresh_period_us};
    }

    const auto scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::TelemetryPublish));
    telemetry::ControlStepTelemetry telemetry_sample{};
    telemetry_sample.estimated_state = estimated_state_.read();
    telemetry_sample.joint_targets = joint_targets_.read();
    telemetry_sample.status = status_.read();
    telemetry_sample.fusion.has_data = telemetry_sample.estimated_state.has_fusion_diagnostics;
    telemetry_sample.fusion.diagnostics = telemetry_sample.estimated_state.fusion;
    telemetry_sample.fusion.foot_contact_fusion = telemetry_sample.estimated_state.foot_contact_fusion;
    telemetry_sample.fusion.correction = last_fusion_correction_;
    if (navigation_manager_) {
        telemetry_sample.navigation = navigation_manager_->monitor();
    }
    telemetry_sample.process_resources = last_process_resources_;
    telemetry_sample.resource_sections = last_resource_sections_;
    telemetry_sample.timestamp_us = now;
    telemetry_publisher_->publishControlStep(telemetry_sample);
    (void)scope;
}

void RobotRuntime::maybeWriteReplayRecord(const TimePointUs& now) {
    if (!replay_logger_) {
        return;
    }

    const auto scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ReplayWrite));
    replay_json::ReplayTelemetryRecord record{};
    record.timestamp_us = now;
    record.sample_id = estimated_state_.read().sample_id;
    record.status = status_.read();
    record.estimated_state = estimated_state_.read();
    record.leg_targets = leg_targets_.read();
    record.gait_state = gait_state_.read();
    record.joint_targets = joint_targets_.read();
    record.transition_diagnostics =
        buildReplayTransitionDiagnostics(record.estimated_state, record.gait_state, record.joint_targets);
    if (navigation_manager_) {
        record.terrain_snapshot = navigation_manager_->latestMapSnapshot(now);
    }
    replay_logger_->write(record);
    (void)scope;
}

void RobotRuntime::safetyStep() {
    const auto scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::SafetyEvaluate));
    const RobotState raw = raw_state_.read();
    const RobotState est = estimated_state_.read();
    const TimePointUs now = now_us();
    const MotionIntent intent = resolveEffectiveIntent(est, now);
    const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
        RuntimeFreshnessGate::EvaluationMode::SafetyLenient, now, est, intent);

    const SafetySupervisor::FreshnessInputs freshness_inputs{
        freshness.estimator.valid,
        freshness.intent.valid};
    const SafetyState s = safety_.evaluate(raw, est, intent, freshness_inputs);
    if (logger_ && s.active_fault != last_logged_safety_fault_) {
        int contact_count = 0;
        for (const bool foot_contact : raw.foot_contacts) {
            if (foot_contact) {
                ++contact_count;
            }
        }
        LOG_WARN(logger_,
                 "safety_fault_transition fault=",
                 static_cast<int>(s.active_fault),
                 " lifecycle=",
                 static_cast<int>(s.fault_lifecycle),
                 " inhibit_motion=",
                 s.inhibit_motion ? 1 : 0,
                 " torque_cut=",
                 s.torque_cut ? 1 : 0,
                 " est_fresh=",
                 freshness.estimator.valid ? 1 : 0,
                 " est_stale_age=",
                 freshness.estimator.stale_age ? 1 : 0,
                 " est_missing_ts=",
                 freshness.estimator.missing_timestamp ? 1 : 0,
                 " est_invalid_sid=",
                 freshness.estimator.invalid_sample_id ? 1 : 0,
                 " est_nonmono_sid=",
                 freshness.estimator.non_monotonic_sample_id ? 1 : 0,
                 " intent_fresh=",
                 freshness.intent.valid ? 1 : 0,
                 " intent_stale_age=",
                 freshness.intent.stale_age ? 1 : 0,
                 " intent_missing_ts=",
                 freshness.intent.missing_timestamp ? 1 : 0,
                 " intent_invalid_sid=",
                 freshness.intent.invalid_sample_id ? 1 : 0,
                 " intent_nonmono_sid=",
                 freshness.intent.non_monotonic_sample_id ? 1 : 0,
                 " est_age_us=",
                 freshness.estimator.age_us,
                 " intent_age_us=",
                 freshness.intent.age_us,
                 " est_sid=",
                 est.sample_id,
                 " intent_sid=",
                 intent.sample_id,
                 " contacts=",
                 contact_count,
                 " bus_ok=",
                 raw.bus_ok ? 1 : 0,
                 " has_power=",
                 raw.has_power_state ? 1 : 0,
                 " voltage=",
                 raw.voltage,
                 " current=",
                 raw.current,
                 " roll=",
                 est.body_twist_state.twist_pos_rad.x,
                 " pitch=",
                 est.body_twist_state.twist_pos_rad.y,
                 " mode=",
                 static_cast<int>(intent.requested_mode));
        last_logged_safety_fault_ = s.active_fault;
    }
    safety_state_.write(s);
    (void)scope;
}

void RobotRuntime::diagnosticsStep() {
    resource_monitoring::ProcessResourceSnapshot process_resources{};
    {
        const auto sample_scope =
            resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::DiagnosticsSampleResources));
        process_resources = process_resource_sampler_.sample();
        (void)sample_scope;
    }
    last_process_resources_ = process_resources;
    last_resource_sections_ = resource_profiler_.topSections(runtime_resource_monitoring::kTopSectionsToReport);
    const auto report_scope = resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::DiagnosticsReport));
    const auto st = status_.read();
    const RobotState raw = raw_state_.read();
    const RobotState est = estimated_state_.read();
    const SafetyState safety = safety_state_.read();
    const auto bridge_result = hw_ ? hw_->last_bridge_result() : std::nullopt;
    const uint64_t loops = control_loop_counter_.load();
    diagnostics_reporter_.recordVisualizerTelemetry(telemetry_publisher_->counters(), now_us());
    diagnostics_reporter_.report(st,
                                 bridge_result,
                                 loops,
                                 timing_metrics_.averageControlDtUs(loops),
                                 control_jitter_max_us_.load(),
                                 stale_intent_count_.load(),
                                 stale_estimator_count_.load(),
                                 process_resources,
                                 last_resource_sections_);
    const MotionIntent intent = resolveEffectiveIntent(est, now_us());
    if (logger_) {
        const double commanded_body_height_m = intent.twist.body_trans_m.z;
        const double measured_body_height_m =
            est.has_body_twist_state ? est.body_twist_state.body_trans_m.z : 0.0;
        const double fusion_body_position_error_m = est.fusion.residuals.body_position_error_m.z;
        const double body_height_setpoint_error_m = commanded_body_height_m - measured_body_height_m;
        LOG_INFO(logger_,
                 "body_height_snapshot cmd_body_height_m=",
                 commanded_body_height_m,
                 " measured_body_height_m=",
                 measured_body_height_m,
                 " body_height_setpoint_error_m=",
                 body_height_setpoint_error_m,
                 " fusion_body_position_error_m=",
                 fusion_body_position_error_m,
                 " fusion_max_body_position_error_m=",
                 est.fusion.residuals.max_body_position_error_m,
                 " active_mode=",
                 static_cast<int>(st.active_mode),
                 " safety_fault=",
                 static_cast<int>(safety.active_fault),
                 " inhibit_motion=",
                 (safety.inhibit_motion ? 1 : 0),
                 " est_valid=",
                 (st.estimator_valid ? 1 : 0));
    }
    if (logger_) {
        std::ostringstream contact_snapshot;
        contact_snapshot << "contact_truth_snapshot raw=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                contact_snapshot << ',';
            }
            contact_snapshot << (raw.foot_contacts[leg] ? '1' : '0');
        }
        contact_snapshot << "] fused=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                contact_snapshot << ',';
            }
            contact_snapshot << (est.foot_contacts[leg] ? '1' : '0');
        }
        contact_snapshot << "] phase=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                contact_snapshot << ',';
            }
            contact_snapshot << contactPhaseName(est.foot_contact_fusion[leg].phase);
        }
        contact_snapshot << "] confidence=[";
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            if (leg > 0) {
                contact_snapshot << ',';
            }
            contact_snapshot << est.foot_contact_fusion[leg].confidence;
        }
        contact_snapshot << "] mismatch_ratio=" << est.fusion.residuals.contact_mismatch_ratio
                         << " terrain_residual_m=" << est.fusion.residuals.terrain_residual_m
                         << " model_trust=" << est.fusion.model_trust
                         << " safety_fault=" << static_cast<int>(safety.active_fault)
                         << " inhibit_motion=" << (safety.inhibit_motion ? 1 : 0)
                         << " active_mode=" << static_cast<int>(st.active_mode)
                         << " bus_ok=" << (st.bus_ok ? 1 : 0)
                         << " est_valid=" << (st.estimator_valid ? 1 : 0);
        LOG_INFO(logger_, contact_snapshot.str());
    }
    if (logger_) {
        LOG_DEBUG(logger_,
                  "fusion_correction_stats hard_reset_requests=",
                  fusion_hard_reset_request_count_.load(),
                  " resync_requests=",
                  fusion_resync_request_count_.load(),
                  " emitted_soft=",
                  fusion_emit_soft_count_.load(),
                  " emitted_strong=",
                  fusion_emit_strong_count_.load(),
                  " emitted_hard_reset=",
                  fusion_emit_hard_reset_count_.load(),
                  " suppressed=",
                  fusion_suppressed_count_.load(),
                  " suppress_physics_sim_corrections=",
                  config_.fusion.emit_physics_sim_corrections ? 0 : 1,
                  " suppress_fusion_resets=",
                  config_.fusion.suppress_fusion_resets ? 1 : 0);
    }
    (void)report_scope;
}

void RobotRuntime::setMotionIntent(const MotionIntent& intent) {
    MotionIntent stamped_intent = intent;
    if (stamped_intent.timestamp_us.isZero()) {
        stamped_intent.timestamp_us = now_us();
    }
    if (stamped_intent.sample_id == 0) {
        stamped_intent.sample_id = intent_sample_seq_.fetch_add(1) + 1;
    }
    motion_intent_.write(stamped_intent);
}

void RobotRuntime::setMotionIntentForTest(const MotionIntent& intent) {
    motion_intent_.write(intent);
}

bool RobotRuntime::setSimFaultToggles(const SimHardwareFaultToggles& toggles) {
    if (auto* sim_hw = dynamic_cast<SimHardwareBridge*>(hw_.get())) {
        sim_hw->setFaultToggles(toggles);
        return true;
    }
    // Scenarios only need this hook to succeed; physics UDP uses real contacts (fault overrides N/A for now).
    if (dynamic_cast<PhysicsSimBridge*>(hw_.get()) != nullptr) {
        (void)toggles;
        return true;
    }
    return false;
}

void RobotRuntime::setNavigationManager(std::unique_ptr<NavigationManager> navigation_manager) {
    navigation_manager_ = std::move(navigation_manager);
    last_effective_intent_estimator_sample_id_ = 0;
}

RobotRuntime::FusionControlPolicy RobotRuntime::applyFusionConsistency(const RobotState& est,
                                                                      const TimePointUs now,
                                                                      const LocalMapSnapshot* terrain_snapshot) {
    const auto scope =
        resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlFusionConsistency));
    FusionControlPolicy policy{};
    if (!est.has_fusion_diagnostics) {
        last_fusion_correction_ = {};
        (void)scope;
        return policy;
    }

    policy.aggressiveness_scale = std::clamp(0.30 + 0.70 * est.fusion.model_trust, 0.20, 1.0);
    if (est.fusion.resync_requested) {
        policy.aggressiveness_scale = std::min(policy.aggressiveness_scale, 0.70);
    }

    const auto sample_id = est.sample_id != 0 ? est.sample_id : control_loop_counter_.load();
    const bool hard_reset_requested = est.fusion.hard_reset_requested;
    if (hard_reset_requested) {
        fusion_hard_reset_request_count_.fetch_add(1);
    }
    if (est.fusion.resync_requested && !hard_reset_requested) {
        fusion_resync_request_count_.fetch_add(1);
    }

    last_fusion_correction_ = {};
    PhysicsSimCorrectionPacket correction = buildPhysicsSimCorrection(
        est,
        now,
        terrain_snapshot,
        config_.fusion,
        static_cast<PhysicsSimCorrectionMode>(last_fusion_correction_mode_),
        last_fusion_correction_sample_id_);
    const bool should_emit_correction =
        correction.mode != PhysicsSimCorrectionMode::None &&
        sample_id != last_fusion_correction_sample_id_ &&
        static_cast<std::uint8_t>(correction.mode) != last_fusion_correction_mode_;
    const bool suppress_corrections = !config_.fusion.emit_physics_sim_corrections;
    const bool suppress_hard_reset = config_.fusion.suppress_fusion_resets && hard_reset_requested;
    const bool allow_emit =
        !suppress_corrections && !(suppress_hard_reset && correction.mode == PhysicsSimCorrectionMode::HardReset);

    if (should_emit_correction && allow_emit) {
        correction.packet.sequence_id = static_cast<std::uint32_t>(sample_id);
        const bool sent = maybeSendPhysicsSimCorrection(hw_.get(), correction.packet);
        if (sent) {
            last_fusion_correction_sample_id_ = sample_id;
            last_fusion_correction_mode_ = static_cast<std::uint8_t>(correction.mode);
            if (correction.mode == PhysicsSimCorrectionMode::Soft) {
                fusion_emit_soft_count_.fetch_add(1);
            } else if (correction.mode == PhysicsSimCorrectionMode::Strong) {
                fusion_emit_strong_count_.fetch_add(1);
            } else if (correction.mode == PhysicsSimCorrectionMode::HardReset) {
                fusion_emit_hard_reset_count_.fetch_add(1);
            }
            last_fusion_correction_.has_data = true;
            last_fusion_correction_.mode = toTelemetryCorrectionMode(correction.mode);
            last_fusion_correction_.sample_id = sample_id;
            last_fusion_correction_.timestamp_us = now;
            last_fusion_correction_.correction_strength = correction.packet.correction_strength;
            last_fusion_correction_.residuals = est.fusion.residuals;
            if (logger_) {
                LOG_INFO(logger_,
                         "fusion correction emitted mode=",
                         correctionModeName(correction.mode),
                         " sample_id=",
                         sample_id,
                         " trust=",
                         est.fusion.model_trust,
                         " pos_err=",
                         est.fusion.residuals.max_body_position_error_m,
                         " yaw_err=",
                         est.fusion.residuals.max_body_orientation_error_rad,
                         " contact_mismatch=",
                         est.fusion.residuals.contact_mismatch_ratio,
                         " terrain_err=",
                         est.fusion.residuals.terrain_residual_m);
            }
        } else if (logger_ && dynamic_cast<PhysicsSimBridge*>(hw_.get()) != nullptr) {
            LOG_WARN(logger_,
                     "failed to emit physics-sim correction mode=",
                     correctionModeName(correction.mode),
                     " sample_id=",
                     sample_id,
                     " suppress_corrections=",
                     suppress_corrections ? 1 : 0,
                     " suppress_hard_reset=",
                     suppress_hard_reset ? 1 : 0);
        }
    } else if (should_emit_correction && !allow_emit) {
        fusion_suppressed_count_.fetch_add(1);
        if (logger_) {
            LOG_INFO(logger_,
                     "suppressed physics-sim correction mode=",
                     correctionModeName(correction.mode),
                     " sample_id=",
                     sample_id,
                     " suppress_corrections=",
                     suppress_corrections ? 1 : 0,
                     " suppress_hard_reset=",
                     suppress_hard_reset ? 1 : 0);
        }
    }

    if (!hard_reset_requested || config_.fusion.suppress_fusion_resets) {
        (void)scope;
        return policy;
    }

    if (sample_id != 0 && sample_id == last_fusion_reset_sample_id_) {
        policy.force_stand = true;
        policy.aggressiveness_scale = std::min(policy.aggressiveness_scale, 0.20);
        return policy;
    }

    last_fusion_reset_sample_id_ = sample_id;
    policy.force_stand = true;
    policy.aggressiveness_scale = std::min(policy.aggressiveness_scale, 0.20);

    if (logger_) {
        LOG_WARN(logger_,
                 "fusion hard reset requested; resyncing estimator and navigation state (sample_id=",
                 sample_id,
                 ", trust=",
                 est.fusion.model_trust,
                 ")");
    }

    if (navigation_manager_) {
        navigation_manager_->reset();
    }
    if (estimator_) {
        estimator_->reset();
    }
    pipeline_.reset();
    freshness_gate_.reset();
    last_effective_intent_estimator_sample_id_ = 0;
    (void)scope;
    return policy;
}

void RobotRuntime::scaleMotionIntent(MotionIntent& intent, const double scale) {
    const double s = std::clamp(scale, 0.0, 1.0);
    intent.cmd_vx_mps = LinearRateMps{intent.cmd_vx_mps.value * s};
    intent.cmd_vy_mps = LinearRateMps{intent.cmd_vy_mps.value * s};
    intent.cmd_yaw_radps = AngularRateRadPerSec{intent.cmd_yaw_radps.value * s};
    intent.speed_mps = LinearRateMps{intent.speed_mps.value * s};
    intent.twist.body_trans_m.x *= s;
    intent.twist.body_trans_m.y *= s;
    intent.twist.twist_vel_radps.x *= s;
    intent.twist.twist_vel_radps.y *= s;
    intent.twist.twist_vel_radps.z *= s;
}

ControlStatus RobotRuntime::getStatus() const {
    return status_.read();
}

SafetyState RobotRuntime::getSafetyState() const {
    return safety_state_.read();
}

MotionIntent RobotRuntime::resolveEffectiveIntent(const RobotState& est, const TimePointUs now) {
    const auto scope =
        resource_profiler_.scope(runtime_resource_monitoring::toIndex(runtime_resource_monitoring::Section::ControlResolveIntent));
    if (!navigation_manager_) {
        const MotionIntent intent = motion_intent_.read();
        effective_motion_intent_.write(intent);
        (void)scope;
        return intent;
    }

    const MotionIntent fallback = motion_intent_.read();
    if (!navigation_manager_->active()) {
        effective_motion_intent_.write(fallback);
        last_effective_intent_estimator_sample_id_ = est.sample_id;
        (void)scope;
        return fallback;
    }

    const MotionIntent effective = navigation_manager_->mergeIntent(fallback, est, now);
    effective_motion_intent_.write(effective);
    last_effective_intent_estimator_sample_id_ = est.sample_id;
    (void)scope;
    return effective;
}
