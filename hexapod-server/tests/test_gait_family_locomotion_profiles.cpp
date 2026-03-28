#include "body_controller.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

const char* gaitName(GaitType gait) {
    switch (gait) {
    case GaitType::TRIPOD:
        return "TRIPOD";
    case GaitType::RIPPLE:
        return "RIPPLE";
    case GaitType::WAVE:
        return "WAVE";
    }
    return "UNKNOWN";
}

RobotState nominalEstimate() {
    RobotState est{};
    est.foot_contacts = {true, true, true, true, true, true};
    est.has_valid_flag = true;
    est.valid = true;
    for (auto& leg : est.leg_states) {
        leg.joint_state[COXA].pos_rad = AngleRad{0.0};
        leg.joint_state[FEMUR].pos_rad = AngleRad{-0.6};
        leg.joint_state[TIBIA].pos_rad = AngleRad{-0.8};
    }
    return est;
}

struct SegmentCommand {
    int ticks{0};
    double speed_mps{0.0};
    double yaw_radps{0.0};
};

MotionIntent makeIntent(GaitType gait, double speed_mps, double yaw_radps) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.gait = gait;
    intent.speed_mps = LinearRateMps{speed_mps};
    intent.body_pose_setpoint.body_trans_mps.x = speed_mps;
    intent.body_pose_setpoint.angular_velocity_radps.z = yaw_radps;
    intent.timestamp_us = now_us();
    return intent;
}

struct ProfileMetrics {
    double peak_foot_velocity_mps{0.0};
    double peak_joint_velocity_radps{0.0};
    std::vector<GaitFallbackStage> fallback_transitions{};
    bool completed{false};
};

void recordFallbackTransition(std::vector<GaitFallbackStage>& transitions,
                              GaitFallbackStage stage) {
    if (transitions.empty() || transitions.back() != stage) {
        transitions.push_back(stage);
    }
}

ProfileMetrics executeLocomotionProfile(GaitType gait,
                                        const std::vector<SegmentCommand>& profile,
                                        bool inject_fallback_sequence) {
    GaitPolicyPlanner planner{};
    GaitScheduler scheduler{};
    BodyController body_controller{};
    LegIK ik{};

    RobotState estimated = nominalEstimate();
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.stable = true;

    ProfileMetrics metrics{};

    std::array<Vec3, kNumLegs> previous_foot_positions{};
    std::array<std::array<double, kJointsPerLeg>, kNumLegs> previous_joint_positions{};
    bool have_previous_sample = false;

    int tick = 0;
    int total_ticks = 0;
    for (const SegmentCommand& segment : profile) {
        total_ticks += segment.ticks;
    }

    for (const SegmentCommand& segment : profile) {
        for (int local_tick = 0; local_tick < segment.ticks; ++local_tick, ++tick) {
            if (inject_fallback_sequence) {
                if (tick == total_ticks / 3) {
                    safety.inhibit_motion = true;
                } else if (tick == (total_ticks / 3) + 8) {
                    safety.inhibit_motion = false;
                    safety.active_fault = FaultCode::TIP_OVER;
                } else if (tick == (total_ticks / 3) + 16) {
                    safety.active_fault = FaultCode::NONE;
                }
            }

            MotionIntent intent = makeIntent(gait, segment.speed_mps, segment.yaw_radps);
            const RuntimeGaitPolicy policy = planner.plan(estimated, intent, safety);
            recordFallbackTransition(metrics.fallback_transitions, policy.fallback_stage);

            const GaitState gait_state = scheduler.update(estimated, intent, safety, policy);
            const LegTargets targets = body_controller.update(estimated, intent, gait_state, safety);
            const JointTargets joints = ik.solve(estimated, targets, safety);

            if (have_previous_sample) {
                constexpr double dt_s = 0.005;
                if (tick > 20) {
                    for (int leg = 0; leg < kNumLegs; ++leg) {
                        const Vec3 foot_pos = static_cast<Vec3>(targets.feet[leg].pos_body_m);
                        const Vec3 foot_delta = foot_pos - previous_foot_positions[leg];
                        const double foot_speed = std::sqrt((foot_delta.x * foot_delta.x) +
                                                            (foot_delta.y * foot_delta.y) +
                                                            (foot_delta.z * foot_delta.z)) /
                                                  dt_s;
                        metrics.peak_foot_velocity_mps = std::max(metrics.peak_foot_velocity_mps, foot_speed);
                        previous_foot_positions[leg] = foot_pos;

                        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                            const double pos = joints.leg_states[leg].joint_state[joint].pos_rad.value;
                            const double vel = std::abs(pos - previous_joint_positions[leg][joint]) / dt_s;
                            metrics.peak_joint_velocity_radps =
                                std::max(metrics.peak_joint_velocity_radps, vel);
                            previous_joint_positions[leg][joint] = pos;
                        }
                    }
                }
            } else {
                for (int leg = 0; leg < kNumLegs; ++leg) {
                    previous_foot_positions[leg] = static_cast<Vec3>(targets.feet[leg].pos_body_m);
                    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                        previous_joint_positions[leg][joint] =
                            joints.leg_states[leg].joint_state[joint].pos_rad.value;
                    }
                }
            }

            have_previous_sample = true;
            estimated.leg_states = joints.leg_states;
            estimated.foot_contacts = gait_state.in_stance;

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    metrics.completed = true;
    return metrics;
}

bool testParameterizedLocomotionProfilesAcrossGaitFamilies() {
    const std::vector<GaitType> gait_families{GaitType::TRIPOD, GaitType::RIPPLE, GaitType::WAVE};
    const std::vector<SegmentCommand> shared_profile{
        SegmentCommand{.ticks = 50, .speed_mps = 0.08, .yaw_radps = 0.00},
        SegmentCommand{.ticks = 50, .speed_mps = 0.08, .yaw_radps = 0.25},
        SegmentCommand{.ticks = 50, .speed_mps = 0.12, .yaw_radps = 0.00},
    };

    constexpr double kFootVelocityBoundMps = 15.0;
    constexpr double kJointVelocityBoundRadps = 120.0;

    for (const GaitType gait : gait_families) {
        const ProfileMetrics nominal = executeLocomotionProfile(gait, shared_profile, false);
        if (!expect(nominal.completed,
                    std::string{"nominal profile should complete without faults for "} + gaitName(gait))) {
            return false;
        }
        if (!expect(!nominal.fallback_transitions.empty() &&
                        nominal.fallback_transitions.front() == GaitFallbackStage::NONE &&
                        nominal.fallback_transitions.back() == GaitFallbackStage::NONE,
                    std::string{"nominal fallback stage should remain NONE for "} + gaitName(gait))) {
            return false;
        }
        if (!expect(nominal.peak_foot_velocity_mps < kFootVelocityBoundMps,
                    std::string{"peak foot velocity should stay bounded for "} + gaitName(gait))) {
            return false;
        }
        if (!expect(nominal.peak_joint_velocity_radps < kJointVelocityBoundRadps,
                    std::string{"peak joint velocity should stay bounded for "} + gaitName(gait))) {
            return false;
        }

        const ProfileMetrics fault_injected = executeLocomotionProfile(gait, shared_profile, true);
        if (!expect(!fault_injected.fallback_transitions.empty() &&
                        fault_injected.fallback_transitions.front() == GaitFallbackStage::NONE,
                    std::string{"fallback transition trace should begin from NONE for "} + gaitName(gait))) {
            return false;
        }

        const bool saw_safe_stop = std::find(fault_injected.fallback_transitions.begin(),
                                             fault_injected.fallback_transitions.end(),
                                             GaitFallbackStage::SAFE_STOP) !=
                                   fault_injected.fallback_transitions.end();
        const bool saw_fault_hold = std::find(fault_injected.fallback_transitions.begin(),
                                              fault_injected.fallback_transitions.end(),
                                              GaitFallbackStage::FAULT_HOLD) !=
                                    fault_injected.fallback_transitions.end();

        if (!expect(saw_safe_stop,
                    std::string{"fallback transitions should include SAFE_STOP for "} + gaitName(gait)) ||
            !expect(saw_fault_hold,
                    std::string{"fallback transitions should include FAULT_HOLD for "} + gaitName(gait))) {
            return false;
        }
    }

    return true;
}

} // namespace

int main() {
    if (!testParameterizedLocomotionProfilesAcrossGaitFamilies()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
