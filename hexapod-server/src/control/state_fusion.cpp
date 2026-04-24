#include "state_fusion.hpp"

#include <algorithm>
#include <cmath>

namespace state_fusion {
namespace {

constexpr double kMeasuredPoseBlend{0.86};
constexpr double kPredictivePoseBlend{0.52};
constexpr double kMeasuredVelocityBlend{0.82};
constexpr double kPredictiveVelocityBlend{0.48};
constexpr double kMeasuredRise{0.22};
constexpr double kPredictiveRise{0.14};
constexpr double kMeasuredDecay{0.10};
constexpr double kPredictiveDecay{0.14};
constexpr double kSoftResidualWeight{0.35};
constexpr double kSoftOrientationWeight{0.25};

double wrapAngleDelta(const double delta) {
    return std::atan2(std::sin(delta), std::cos(delta));
}

Vec3 blendVec3(const Vec3& from, const Vec3& to, const double alpha) {
    const double a = std::clamp(alpha, 0.0, 1.0);
    return from * (1.0 - a) + to * a;
}

EulerAnglesRad3 blendAngles(const EulerAnglesRad3& from, const EulerAnglesRad3& to, const double alpha) {
    const double a = std::clamp(alpha, 0.0, 1.0);
    const Vec3 from_v = from;
    const Vec3 to_v = to;
    const Vec3 delta{
        wrapAngleDelta(to_v.x - from_v.x),
        wrapAngleDelta(to_v.y - from_v.y),
        wrapAngleDelta(to_v.z - from_v.z),
    };
    return EulerAnglesRad3{
        from_v.x + delta.x * a,
        from_v.y + delta.y * a,
        from_v.z + delta.z * a,
    };
}

} // namespace

StateFusion::StateFusion(control_config::FusionConfig config)
    : config_(config) {}

void StateFusion::configure(const control_config::FusionConfig& config) {
    config_ = config;
}

void StateFusion::reset() {
    for (FootTracker& tracker : foot_trackers_) {
        tracker = FootTracker{};
    }
    last_output_ = RobotState{};
    diagnostics_ = FusionDiagnostics{};
    last_update_us_ = TimePointUs{};
    have_last_output_ = false;
}

double StateFusion::phaseConfidenceFloor(const ContactPhase phase) {
    switch (phase) {
        case ContactPhase::ConfirmedStance:
            return 0.80;
        case ContactPhase::ContactCandidate:
            return 0.48;
        case ContactPhase::ExpectedTouchdown:
            return 0.32;
        case ContactPhase::LostCandidate:
            return 0.20;
        case ContactPhase::Swing:
            return 0.08;
        case ContactPhase::Search:
        default:
            return 0.05;
    }
}

const FusionDiagnostics& StateFusion::diagnostics() const {
    return diagnostics_;
}

RobotState StateFusion::update(const RobotState& source, const FusionSourceMode mode) {
    RobotState out = source;
    const TimePointUs now = source.timestamp_us;
    const bool predictive_mode = (mode == FusionSourceMode::Predictive);
    const bool have_prev = have_last_output_ && !last_update_us_.isZero();
    const double dt_s = (have_prev && now.value > last_update_us_.value)
                            ? static_cast<double>(now.value - last_update_us_.value) * 1e-6
                            : 0.0;

    RobotState predicted = last_output_;
    if (have_prev && dt_s > 0.0 && last_output_.has_body_twist_state) {
        const Vec3 predicted_trans = last_output_.body_twist_state.body_trans_m.raw() +
                                     last_output_.body_twist_state.body_trans_mps.raw() * dt_s;
        predicted.body_twist_state.body_trans_m = PositionM3{predicted_trans};
        predicted.body_twist_state.body_trans_mps = last_output_.body_twist_state.body_trans_mps;
        const Vec3 predicted_rot{
            last_output_.body_twist_state.twist_pos_rad.x +
                last_output_.body_twist_state.twist_vel_radps.x * dt_s,
            last_output_.body_twist_state.twist_pos_rad.y +
                last_output_.body_twist_state.twist_vel_radps.y * dt_s,
            last_output_.body_twist_state.twist_pos_rad.z +
                last_output_.body_twist_state.twist_vel_radps.z * dt_s,
        };
        predicted.body_twist_state.twist_pos_rad = EulerAnglesRad3{predicted_rot};
        predicted.body_twist_state.twist_vel_radps = last_output_.body_twist_state.twist_vel_radps;
    }

    FusionResidualSummary residuals{};
    if (source.has_body_twist_state && (predicted.has_body_twist_state || have_prev)) {
        const Vec3 measured_pos = source.body_twist_state.body_trans_m.raw();
        const Vec3 predicted_pos = predicted.has_body_twist_state
                                       ? predicted.body_twist_state.body_trans_m.raw()
                                       : measured_pos;
        const Vec3 measured_vel = source.body_twist_state.body_trans_mps.raw();
        const Vec3 predicted_vel = predicted.has_body_twist_state
                                       ? predicted.body_twist_state.body_trans_mps.raw()
                                       : measured_vel;
        const Vec3 measured_ori = source.body_twist_state.twist_pos_rad.raw();
        const Vec3 predicted_ori = predicted.has_body_twist_state
                                       ? predicted.body_twist_state.twist_pos_rad.raw()
                                       : measured_ori;

        residuals.body_position_error_m = measured_pos - predicted_pos;
        residuals.body_velocity_error_mps = measured_vel - predicted_vel;
        residuals.body_orientation_error_rad = EulerAnglesRad3{
            wrapAngleDelta(measured_ori.x - predicted_ori.x),
            wrapAngleDelta(measured_ori.y - predicted_ori.y),
            wrapAngleDelta(measured_ori.z - predicted_ori.z),
        };
        residuals.max_body_position_error_m = vecNorm(residuals.body_position_error_m);
        residuals.max_body_orientation_error_rad = std::max(
            {std::abs(residuals.body_orientation_error_rad.x),
             std::abs(residuals.body_orientation_error_rad.y),
             std::abs(residuals.body_orientation_error_rad.z)});
        residuals.terrain_residual_m = std::abs(residuals.body_position_error_m.z);
    }

    const uint64_t touchdown_window_us =
        static_cast<uint64_t>(std::max<int64_t>(0, static_cast<int64_t>(config_.touchdown_window.count()))) * 1000ULL;
    const uint64_t contact_hold_window_us =
        static_cast<uint64_t>(std::max<int64_t>(0, static_cast<int64_t>(config_.contact_hold_window.count()))) * 1000ULL;
    const int debounce_samples = std::max(1, config_.contact_debounce_samples);
    const double rise = predictive_mode ? kPredictiveRise : kMeasuredRise;
    const double decay = predictive_mode ? kPredictiveDecay : kMeasuredDecay;

    int mismatch_count = 0;
    double confidence_sum = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        FootTracker& tracker = foot_trackers_[static_cast<std::size_t>(leg)];
        FootContactFusion fc = tracker.state;
        const bool raw_contact = source.foot_contacts[static_cast<std::size_t>(leg)];
        bool phase_changed = false;

        if (!have_prev && raw_contact) {
            fc.phase = ContactPhase::ConfirmedStance;
            fc.confidence = 1.0f;
            fc.touchdown_window_start_us = now;
            fc.touchdown_window_end_us = TimePointUs{now.value + contact_hold_window_us};
            fc.last_transition_us = now;
            tracker.raw_contact_streak = 1;
            tracker.raw_gap_streak = 0;
            phase_changed = true;
        } else if (raw_contact) {
            tracker.raw_contact_streak += 1;
            tracker.raw_gap_streak = 0;

            const bool was_confirmed = (fc.phase == ContactPhase::ConfirmedStance);
            if (!was_confirmed && tracker.raw_contact_streak == 1) {
                fc.phase = ContactPhase::ExpectedTouchdown;
                fc.touchdown_window_start_us = now;
                fc.touchdown_window_end_us = TimePointUs{now.value + touchdown_window_us};
                phase_changed = true;
            } else if (!was_confirmed && tracker.raw_contact_streak < static_cast<uint32_t>(debounce_samples)) {
                if (fc.phase != ContactPhase::ContactCandidate) {
                    phase_changed = true;
                }
                fc.phase = ContactPhase::ContactCandidate;
                fc.touchdown_window_start_us = now;
                fc.touchdown_window_end_us = TimePointUs{now.value + touchdown_window_us};
            } else {
                if (!was_confirmed) {
                    phase_changed = true;
                }
                fc.phase = ContactPhase::ConfirmedStance;
                fc.touchdown_window_start_us = now;
                fc.touchdown_window_end_us = TimePointUs{now.value + contact_hold_window_us};
            }

            fc.confidence = static_cast<float>(
                std::min(1.0, std::max(phaseConfidenceFloor(fc.phase), static_cast<double>(fc.confidence)) + rise));
            fc.last_transition_us = phase_changed ? now : fc.last_transition_us;
        } else {
            tracker.raw_contact_streak = 0;
            tracker.raw_gap_streak += 1;

            if (fc.phase == ContactPhase::ConfirmedStance) {
                if (!fc.touchdown_window_end_us.isZero() && now.value <= fc.touchdown_window_end_us.value) {
                    fc.confidence = static_cast<float>(std::max(
                        phaseConfidenceFloor(ContactPhase::ConfirmedStance),
                        static_cast<double>(fc.confidence) - decay * 0.35));
                } else {
                    fc.phase = ContactPhase::LostCandidate;
                    fc.touchdown_window_start_us = now;
                    fc.touchdown_window_end_us = TimePointUs{now.value + touchdown_window_us};
                    fc.last_transition_us = now;
                    fc.confidence = static_cast<float>(std::max(
                        phaseConfidenceFloor(fc.phase),
                        static_cast<double>(fc.confidence) - decay));
                }
            } else if (fc.phase == ContactPhase::ContactCandidate || fc.phase == ContactPhase::ExpectedTouchdown) {
                if (!fc.touchdown_window_end_us.isZero() && now.value > fc.touchdown_window_end_us.value) {
                    fc.phase = ContactPhase::Search;
                    fc.touchdown_window_start_us = now;
                    fc.touchdown_window_end_us = TimePointUs{};
                    fc.last_transition_us = now;
                }
                fc.confidence = static_cast<float>(std::max(
                    phaseConfidenceFloor(fc.phase),
                    static_cast<double>(fc.confidence) - decay));
            } else if (fc.phase == ContactPhase::LostCandidate) {
                if (!fc.touchdown_window_end_us.isZero() && now.value > fc.touchdown_window_end_us.value) {
                    fc.phase = ContactPhase::Search;
                    fc.touchdown_window_start_us = now;
                    fc.touchdown_window_end_us = TimePointUs{};
                    fc.last_transition_us = now;
                }
                fc.confidence = static_cast<float>(std::max(
                    phaseConfidenceFloor(fc.phase),
                    static_cast<double>(fc.confidence) - decay));
            } else {
                fc.phase = ContactPhase::Search;
                fc.touchdown_window_start_us = now;
                fc.touchdown_window_end_us = TimePointUs{};
                fc.last_transition_us = now;
                fc.confidence = static_cast<float>(std::max(
                    phaseConfidenceFloor(fc.phase),
                    static_cast<double>(fc.confidence) - decay));
            }
        }

        if (fc.confidence < phaseConfidenceFloor(fc.phase)) {
            fc.confidence = static_cast<float>(phaseConfidenceFloor(fc.phase));
        }

        const bool fused_contact = (fc.phase == ContactPhase::ConfirmedStance);
        out.foot_contacts[static_cast<std::size_t>(leg)] = fused_contact;
        out.foot_contact_fusion[static_cast<std::size_t>(leg)] = fc;

        if (raw_contact != fused_contact) {
            ++mismatch_count;
        }

        confidence_sum += fc.confidence;
        tracker.state = fc;
    }

    residuals.foot_contact_error.fill(0.0);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        residuals.foot_contact_error[static_cast<std::size_t>(leg)] =
            source.foot_contacts[static_cast<std::size_t>(leg)] ==
                    out.foot_contacts[static_cast<std::size_t>(leg)]
                ? 0.0
                : 1.0;
    }
    residuals.contact_mismatch_ratio = static_cast<double>(mismatch_count) / static_cast<double>(kNumLegs);

    const double avg_confidence = confidence_sum / static_cast<double>(kNumLegs);
    const bool measured_body_available = source.has_body_twist_state;

    if (measured_body_available) {
        const double body_blend = predictive_mode ? kPredictivePoseBlend : kMeasuredPoseBlend;
        const double velocity_blend = predictive_mode ? kPredictiveVelocityBlend : kMeasuredVelocityBlend;
        if (have_prev && predicted.has_body_twist_state) {
            out.body_twist_state.body_trans_m =
                blendVec3(predicted.body_twist_state.body_trans_m,
                          source.body_twist_state.body_trans_m,
                          body_blend);
            out.body_twist_state.body_trans_mps =
                blendVec3(predicted.body_twist_state.body_trans_mps,
                          source.body_twist_state.body_trans_mps,
                          velocity_blend);
            out.body_twist_state.twist_pos_rad = blendAngles(
                predicted.body_twist_state.twist_pos_rad,
                source.body_twist_state.twist_pos_rad,
                body_blend);
            out.body_twist_state.twist_vel_radps =
                blendVec3(predicted.body_twist_state.twist_vel_radps,
                          source.body_twist_state.twist_vel_radps,
                          velocity_blend);
        }
        out.has_body_twist_state = true;
    } else if (predictive_mode && have_prev && last_output_.has_body_twist_state) {
        out.body_twist_state = predicted.body_twist_state;
        out.has_body_twist_state = true;
    } else {
        out.body_twist_state = BodyTwistState{};
        out.has_body_twist_state = false;
    }

    const double pose_error_scale = std::max(config_.soft_pose_resync_m, 1e-6);
    const double orientation_error_scale = std::max(config_.soft_orientation_resync_rad, 1e-6);
    const double contact_penalty =
        std::clamp(residuals.contact_mismatch_ratio * config_.trust_decay_per_mismatch, 0.0, 1.0);
    double trust = 0.20 + 0.80 * avg_confidence;
    if (predictive_mode) {
        trust *= std::clamp(config_.predictive_trust_bias, 0.0, 1.0);
    }
    trust -= contact_penalty;
    trust -= kSoftResidualWeight * std::clamp(residuals.max_body_position_error_m / pose_error_scale, 0.0, 1.5);
    trust -= kSoftOrientationWeight *
             std::clamp(residuals.max_body_orientation_error_rad / orientation_error_scale, 0.0, 1.5);
    trust = std::clamp(trust, 0.0, 1.0);

    // Contact disagreement is expected during gait transitions. Keep it in diagnostics and trust,
    // but only escalate resync / reset on actual pose-orientation divergence.
    const bool resync_requested =
        residuals.max_body_position_error_m > config_.soft_pose_resync_m ||
        residuals.max_body_orientation_error_rad > config_.soft_orientation_resync_rad;
    const bool hard_reset_requested =
        residuals.max_body_position_error_m > config_.hard_pose_resync_m ||
        residuals.max_body_orientation_error_rad > config_.hard_orientation_resync_rad;

    out.has_valid_flag = (source.valid || source.has_valid_flag || !now.isZero() || source.sample_id != 0);
    out.valid = out.has_valid_flag;
    out.has_fusion_diagnostics = true;
    out.fusion.model_trust = trust;
    out.fusion.resync_requested = resync_requested;
    out.fusion.hard_reset_requested = hard_reset_requested;
    out.fusion.predictive_mode = predictive_mode;
    out.fusion.residuals = residuals;

    diagnostics_ = out.fusion;
    last_output_ = out;
    last_update_us_ = now;
    have_last_output_ = true;
    return out;
}

} // namespace state_fusion
