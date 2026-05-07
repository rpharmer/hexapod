/**
 * Motion performance metrics (tiered cases) against the physics sim.
 * Linux + HEXAPOD_PHYSICS_SIM_EXE or argv --sim required.
 */

#include "control_config.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "locomotion_metrics.hpp"
#include "locomotion_motion_sequence.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_test_utils.hpp"
#include "test_limits_manifest.hpp"
#include "replay_logger.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

using namespace locomotion_test;

struct PhysicsSimProcess {
    PhysicsSimProcess(std::string exe_path, int port)
        : exe_path_(std::move(exe_path)), port_(port) {}

    ~PhysicsSimProcess() { stop(); }

    bool start() {
#if !defined(__linux__)
        (void)exe_path_;
        (void)port_;
        return false;
#else
        pid_ = ::fork();
        if (pid_ < 0) {
            std::cerr << "fork failed\n";
            return false;
        }
        if (pid_ == 0) {
            physics_sim_test_utils::quietChildProcessStdIo();
            const std::string port_str = std::to_string(port_);
            ::execl(exe_path_.c_str(), exe_path_.c_str(), "--serve", "--serve-port", port_str.c_str(), nullptr);
            std::perror("execl");
            _exit(127);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{250});
        return true;
#endif
    }

    void stop() {
#if defined(__linux__)
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            ::waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
#endif
    }

private:
    std::string exe_path_{};
    int port_{0};
#if defined(__linux__)
    pid_t pid_{-1};
#endif
};

class CapturingPhysicsSimBridge final : public IHardwareBridge {
public:
    CapturingPhysicsSimBridge(std::string host,
                              int port,
                              int bus_loop_period_us,
                              int physics_solver_iterations)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
        }
        return ok;
    }

    bool write(const JointTargets& in) override { return inner_.write(in); }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const { return last_state_; }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
};

std::vector<MotionPhase> buildPhasesFromScenario(const std::filesystem::path& path, const std::string& label) {
    if (path.empty()) {
        throw std::runtime_error(label + ": scenario file not found");
    }
    ScenarioDefinition def{};
    std::string error;
    const std::string path_str = path.string();
    if (!ScenarioDriver::loadFromToml(path_str, def, error)) {
        throw std::runtime_error(label + ": failed to load scenario " + path_str + ": " + error);
    }

    std::vector<MotionPhase> phases;
    if (def.events.empty()) {
        return phases;
    }

    std::vector<ScenarioEvent> events = def.events;
    std::sort(events.begin(), events.end(), [](const ScenarioEvent& lhs, const ScenarioEvent& rhs) {
        return lhs.at_ms < rhs.at_ms;
    });

    for (std::size_t i = 0; i < events.size(); ++i) {
        const ScenarioEvent& event = events[i];
        const uint64_t next_at_ms = (i + 1 < events.size()) ? events[i + 1].at_ms : def.duration_ms;
        const uint64_t span_ms = next_at_ms > event.at_ms ? next_at_ms - event.at_ms : def.tick_ms;
        MotionPhase phase{};
        phase.label = label + "_phase_" + std::to_string(i);
        phase.motion = event.motion;
        phase.steps = static_cast<std::size_t>(std::max<uint64_t>(1, span_ms / def.tick_ms));
        phase.refresh_each_step = def.refresh_motion_intent;
        if (event.has_safety_overrides && event.safety.has_legs_enabled) {
            phase.safety_leg_enabled_mask = event.safety.legs_enabled;
        }
        phases.push_back(std::move(phase));
    }
    return phases;
}

MotionPhase makePhase(const std::string& label,
                      const ScenarioMotionIntent& motion,
                      const std::size_t steps,
                      const bool refresh_each_step = true) {
    MotionPhase phase{};
    phase.label = label;
    phase.motion = motion;
    phase.steps = steps;
    phase.refresh_each_step = refresh_each_step;
    return phase;
}

void scalePhasesForHarness(std::vector<MotionPhase>& phases, const int bus_loop_period_us) {
    for (MotionPhase& phase : phases) {
        phase.steps = physics_sim_test_utils::scaledLegacyStepCount(phase.steps, bus_loop_period_us);
    }
}

bool isSevereFault(const FaultCode f) {
    return f == FaultCode::TIP_OVER || f == FaultCode::ESTIMATOR_INVALID || f == FaultCode::BODY_COLLAPSE ||
           f == FaultCode::MOTOR_FAULT || f == FaultCode::JOINT_LIMIT;
}

/** FK foot tips in world frame; minimum Z should stay at/above ground (z=0) for plausible poses. */
double minFootTipWorldZ(const RobotState& state) {
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    BodyPose body_pose{};
    body_pose.position = state.body_twist_state.body_trans_m;
    body_pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    body_pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    body_pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};

    double min_z = std::numeric_limits<double>::infinity();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const FootTarget foot_world =
            fk.footInWorldFrame(state.leg_states[static_cast<std::size_t>(leg)], body_pose, geometry.legGeometry[leg]);
        min_z = std::min(min_z, foot_world.pos_body_m.z);
    }
    return min_z;
}

double g_foot_tip_ground_tolerance_m = -1.0e-4;
/** Stance feet should stay near their touchdown anchors while contact is held (full-profile horizons drift more than smoke). */
double g_max_stance_anchor_drift_m_forward_like = 0.34;
double g_max_stance_contact_tracking_error_m_forward_like = 0.22;
/** Lateral / diagonal headings accumulate more foot slip in the UDP sim; keep separate ceilings. */
double g_max_stance_anchor_drift_m_lateral = 0.38;
double g_max_stance_contact_tracking_error_m_lateral = 0.20;
double g_max_commanded_tracking_error_m_forward_like = 0.22;
double g_max_commanded_tracking_error_m_lateral = 0.20;
double g_min_measured_foot_world_z_m = -0.04;
double g_stand_settle_path_length_m_max = 0.02;
double g_stand_settle_net_displacement_m_max = 0.005;
double g_stand_settle_max_abs_tilt_rad = 0.02;
double g_stand_settle_max_body_rate_radps = 0.5;
double g_stand_settle_max_contact_anchor_max_drift_m = 0.08;
double g_stand_settle_max_commanded_tracking_error_m = 0.06;
double g_stand_settle_min_measured_foot_world_z_m = 0.005;
double g_single_leg_masked_stand_path_length_m_max = 0.02;
double g_single_leg_masked_stand_net_displacement_m_max = 0.005;
double g_single_leg_masked_stand_max_abs_tilt_rad = 0.02;
double g_single_leg_masked_stand_max_body_rate_radps = 0.5;
double g_single_leg_masked_stand_max_contact_anchor_max_drift_m = 0.08;
double g_single_leg_masked_stand_max_commanded_tracking_error_m = 0.06;
double g_single_leg_masked_stand_min_measured_foot_world_z_m = 0.005;
/** Ignore early walk transients before measuring stride cycles. */
std::size_t g_stride_kinematics_walk_warmup_samples = 45;
std::size_t g_stride_kinematics_min_touchdowns = 8;
/** Measured horizontal touchdown span vs commanded `step_length_m` (world-frame, leg-by-leg). */
double g_touchdown_span_min_vs_step_length = 0.22;
double g_touchdown_span_max_vs_step_length = 2.85;
double g_touchdown_span_max_extra_m = 0.07;
/** Swing apex should lift measurably vs liftoff height (fraction of commanded swing height). */
double g_swing_lift_min_vs_swing_height = 0.17;
double g_swing_lift_abs_floor_m = 0.0035;
double g_swing_lift_percentile = 0.20;

void refreshMotionPerformanceLimitsFromManifest() {
    constexpr const char* kSuite = "motion_performance";
    g_foot_tip_ground_tolerance_m =
        test_limits::getDouble(kSuite, "stride_gates", "", "fk_foot_tip_ground_min_m", -1.0e-4);
    g_min_measured_foot_world_z_m =
        test_limits::getDouble(kSuite, "stride_gates", "", "min_measured_foot_world_z_m", -0.04);
    g_stride_kinematics_walk_warmup_samples =
        test_limits::getSizeT(kSuite, "stride_gates", "", "walk_warmup_samples", 45);
    g_stride_kinematics_min_touchdowns =
        test_limits::getSizeT(kSuite, "stride_gates", "", "min_touchdowns", 8);
    g_touchdown_span_min_vs_step_length =
        test_limits::getDouble(kSuite, "stride_gates", "", "touchdown_span_min_vs_step_length", 0.22);
    g_touchdown_span_max_vs_step_length =
        test_limits::getDouble(kSuite, "stride_gates", "", "touchdown_span_max_vs_step_length", 2.85);
    g_touchdown_span_max_extra_m =
        test_limits::getDouble(kSuite, "stride_gates", "", "touchdown_span_max_extra_m", 0.07);
    g_swing_lift_min_vs_swing_height =
        test_limits::getDouble(kSuite, "stride_gates", "", "swing_lift_min_vs_swing_height", 0.17);
    g_swing_lift_abs_floor_m = test_limits::getDouble(kSuite, "stride_gates", "", "swing_lift_abs_floor_m", 0.0035);
    g_swing_lift_percentile = test_limits::getDouble(kSuite, "stride_gates", "", "swing_lift_percentile", 0.20);

    g_max_stance_anchor_drift_m_forward_like =
        test_limits::getDouble(kSuite, "walk_gates", "forward_like", "max_stance_anchor_drift_m", 0.34);
    g_max_stance_contact_tracking_error_m_forward_like = test_limits::getDouble(
        kSuite, "walk_gates", "forward_like", "max_stance_contact_tracking_error_m", 0.22);
    g_max_commanded_tracking_error_m_forward_like =
        test_limits::getDouble(kSuite, "walk_gates", "forward_like", "max_commanded_tracking_error_m", 0.22);
    g_max_stance_anchor_drift_m_lateral =
        test_limits::getDouble(kSuite, "walk_gates", "lateral", "max_stance_anchor_drift_m", 0.38);
    g_max_stance_contact_tracking_error_m_lateral =
        test_limits::getDouble(kSuite, "walk_gates", "lateral", "max_stance_contact_tracking_error_m", 0.20);
    g_max_commanded_tracking_error_m_lateral =
        test_limits::getDouble(kSuite, "walk_gates", "lateral", "max_commanded_tracking_error_m", 0.20);
    g_stand_settle_path_length_m_max =
        test_limits::getDouble(kSuite, "stand_settle", "", "path_length_m_max", 0.02);
    g_stand_settle_net_displacement_m_max =
        test_limits::getDouble(kSuite, "stand_settle", "", "net_displacement_m_max", 0.005);
    g_stand_settle_max_abs_tilt_rad =
        test_limits::getDouble(kSuite, "stand_settle", "", "max_abs_tilt_rad", 0.02);
    g_stand_settle_max_body_rate_radps =
        test_limits::getDouble(kSuite, "stand_settle", "", "max_body_rate_radps", 0.5);
    g_stand_settle_max_contact_anchor_max_drift_m =
        test_limits::getDouble(kSuite, "stand_settle", "", "max_contact_anchor_max_drift_m", 0.08);
    g_stand_settle_max_commanded_tracking_error_m =
        test_limits::getDouble(kSuite, "stand_settle", "", "max_commanded_tracking_error_m", 0.06);
    g_stand_settle_min_measured_foot_world_z_m =
        test_limits::getDouble(kSuite, "stand_settle", "", "min_measured_foot_world_z_m", 0.005);
    g_single_leg_masked_stand_path_length_m_max =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "path_length_m_max", 0.02);
    g_single_leg_masked_stand_net_displacement_m_max =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "net_displacement_m_max", 0.005);
    g_single_leg_masked_stand_max_abs_tilt_rad =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "max_abs_tilt_rad", 0.02);
    g_single_leg_masked_stand_max_body_rate_radps =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "max_body_rate_radps", 0.5);
    g_single_leg_masked_stand_max_contact_anchor_max_drift_m =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "max_contact_anchor_max_drift_m", 0.08);
    g_single_leg_masked_stand_max_commanded_tracking_error_m =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "max_commanded_tracking_error_m", 0.06);
    g_single_leg_masked_stand_min_measured_foot_world_z_m =
        test_limits::getDouble(kSuite, "single_leg_masked_stand", "", "min_measured_foot_world_z_m", 0.005);
}

double medianFromSorted(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if ((values.size() % 2) == 1) {
        return values[mid];
    }
    return 0.5 * (values[mid - 1] + values[mid]);
}

double percentileFromSorted(const std::vector<double>& sorted, const double fraction_01) {
    if (sorted.empty()) {
        return 0.0;
    }
    const double clamped = std::clamp(fraction_01, 0.0, 1.0);
    const double idx = clamped * static_cast<double>(sorted.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
    if (lo == hi) {
        return sorted[lo];
    }
    const double t = idx - static_cast<double>(lo);
    return sorted[lo] * (1.0 - t) + sorted[hi] * t;
}

struct StrideKinematicsSnapshot {
    bool skipped{true};
    std::string skip_reason{};
    std::size_t touchdown_events{0};
    std::size_t planned_touchdown_events{0};
    std::size_t fused_touchdown_events{0};
    std::size_t planned_liftoff_events{0};
    std::size_t fused_liftoff_events{0};
    double median_touchdown_span_m{0.0};
    double median_commanded_step_length_m{0.0};
    double median_commanded_swing_height_m{0.0};
    double swing_lift_percentile_value_m{0.0};
    double gate_touchdown_span_min_m{0.0};
    double gate_touchdown_span_max_m{0.0};
    double gate_swing_lift_floor_m{0.0};
    double gate_swing_lift_percentile{0.0};
};

struct StrideKinematicsEval {
    StrideKinematicsSnapshot snapshot{};
    std::optional<std::string> failure{};
};

StrideKinematicsEval evaluateStrideKinematics(const std::vector<MotionSample>& samples,
                                              const LocomotionMetrics& metrics) {
    StrideKinematicsEval out{};
    StrideKinematicsSnapshot& snap = out.snapshot;
    snap.gate_swing_lift_percentile = g_swing_lift_percentile;

    if (metrics.walk_sample_count < g_stride_kinematics_walk_warmup_samples + 20) {
        snap.skip_reason = "walk_sample_count_or_warmup";
        return out;
    }
    if (metrics.stride_count < 4) {
        snap.skip_reason = "stride_count";
        return out;
    }
    snap.skipped = false;

    std::vector<double> touchdown_spans_m{};
    std::vector<double> swing_lifts_m{};
    std::vector<double> walk_step_lengths_m{};
    std::vector<double> walk_swing_heights_m{};

    std::array<double, kNumLegs> liftoff_x{};
    std::array<double, kNumLegs> liftoff_y{};
    std::array<double, kNumLegs> z_liftoff{};
    std::array<double, kNumLegs> z_peak{};
    std::array<bool, kNumLegs> have_liftoff{};

    bool have_prev_walk = false;
    std::size_t prev_walk_index = 0;
    std::size_t walk_samples_seen = 0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const MotionSample& s = samples[i];
        if (s.status.active_mode != RobotMode::WALK) {
            have_prev_walk = false;
            have_liftoff.fill(false);
            continue;
        }
        if (!have_prev_walk) {
            have_prev_walk = true;
            prev_walk_index = i;
            continue;
        }

        const MotionSample& prev = samples[prev_walk_index];
        ++walk_samples_seen;
        const bool past_warmup = walk_samples_seen > g_stride_kinematics_walk_warmup_samples;

        if (past_warmup) {
            walk_step_lengths_m.push_back(s.step_length_m);
            walk_swing_heights_m.push_back(s.swing_height_m);
        }

        if (past_warmup && prev.locomotion_debug.valid && s.locomotion_debug.valid) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t li = static_cast<std::size_t>(leg);
                const bool ps = prev.locomotion_debug.planned_stance[li];
                const bool cs = s.locomotion_debug.planned_stance[li];
                const bool p_support = prev.locomotion_debug.fused_support[li];
                const bool c_support = s.locomotion_debug.fused_support[li];
                const Vec3& pm = prev.locomotion_debug.measured_foot_world_m[li];
                const Vec3& cm = s.locomotion_debug.measured_foot_world_m[li];

                if (ps && !cs) {
                    snap.planned_liftoff_events += 1;
                }
                if (!ps && cs) {
                    snap.planned_touchdown_events += 1;
                }

                if (p_support && !c_support) {
                    snap.fused_liftoff_events += 1;
                }
                if (!p_support && c_support) {
                    snap.fused_touchdown_events += 1;
                }

                if (ps && !cs) {
                    liftoff_x[li] = pm.x;
                    liftoff_y[li] = pm.y;
                    z_liftoff[li] = pm.z;
                    z_peak[li] = pm.z;
                    have_liftoff[li] = true;
                } else if (!ps && !cs && have_liftoff[li]) {
                    z_peak[li] = std::max(z_peak[li], cm.z);
                } else if (!ps && cs && have_liftoff[li]) {
                    z_peak[li] = std::max(z_peak[li], pm.z);
                    const double span =
                        std::hypot(cm.x - liftoff_x[li], cm.y - liftoff_y[li]);
                    const double lift = z_peak[li] - z_liftoff[li];
                    touchdown_spans_m.push_back(span);
                    swing_lifts_m.push_back(lift);
                    have_liftoff[li] = false;
                }
            }
        }

        prev_walk_index = i;
    }

    snap.touchdown_events = touchdown_spans_m.size();
    if (snap.fused_touchdown_events < std::min(g_stride_kinematics_min_touchdowns, snap.planned_touchdown_events)) {
        snap.skipped = true;
        snap.skip_reason = std::string("support_transition_count_insufficient: planned_touchdowns=") +
                           std::to_string(snap.planned_touchdown_events) + ", fused_touchdowns=" +
                           std::to_string(snap.fused_touchdown_events) + ", fused_liftoffs=" +
                           std::to_string(snap.fused_liftoff_events);
        return out;
    }

    if (touchdown_spans_m.empty()) {
        out.failure = std::string("stride kinematics: no planner-segmented touchdown events (planned_touchdowns=") +
                      std::to_string(snap.planned_touchdown_events) + ", stride_count=" +
                      std::to_string(metrics.stride_count) + ')';
        return out;
    }

    if (touchdown_spans_m.size() < g_stride_kinematics_min_touchdowns) {
        out.failure = std::string("stride kinematics: too few touchdown events (n=") +
                      std::to_string(touchdown_spans_m.size()) + ", planned_touchdowns=" +
                      std::to_string(snap.planned_touchdown_events) + ", fused_touchdowns=" +
                      std::to_string(snap.fused_touchdown_events) + ')';
        return out;
    }

    const double median_span = medianFromSorted(touchdown_spans_m);
    const double median_step =
        walk_step_lengths_m.empty() ? 0.0 : medianFromSorted(walk_step_lengths_m);
    const double median_swing_h =
        walk_swing_heights_m.empty() ? 0.0 : medianFromSorted(walk_swing_heights_m);

    snap.median_touchdown_span_m = median_span;
    snap.median_commanded_step_length_m = median_step;
    snap.median_commanded_swing_height_m = median_swing_h;

    const double step_ref = std::max(median_step, 1.0e-6);
    const double span_min = g_touchdown_span_min_vs_step_length * step_ref;
    const double span_max = g_touchdown_span_max_vs_step_length * step_ref + g_touchdown_span_max_extra_m;
    snap.gate_touchdown_span_min_m = span_min;
    snap.gate_touchdown_span_max_m = span_max;

    if (!(median_span >= span_min && median_span <= span_max)) {
        out.failure = std::string("stride kinematics: median touchdown span ") + formatDouble(median_span) +
                      " m outside [" + formatDouble(span_min) + ", " + formatDouble(span_max) + "] (median step_length_m=" +
                      formatDouble(median_step) + ')';
        return out;
    }

    std::sort(swing_lifts_m.begin(), swing_lifts_m.end());
    const double swing_ref = std::max(median_swing_h, 1.0e-6);
    const double lift_floor = std::max(g_swing_lift_abs_floor_m, g_swing_lift_min_vs_swing_height * swing_ref);
    snap.gate_swing_lift_floor_m = lift_floor;
    const double lift_pct = percentileFromSorted(swing_lifts_m, g_swing_lift_percentile);
    snap.swing_lift_percentile_value_m = lift_pct;
    if (!(lift_pct >= lift_floor)) {
        out.failure = std::string("stride kinematics: swing vertical lift too low (") +
                      formatDouble(g_swing_lift_percentile * 100.0) + "%ile=" + formatDouble(lift_pct) +
                      " m, need >= " + formatDouble(lift_floor) + " m; median swing_height_m=" +
                      formatDouble(median_swing_h) + ')';
    }

    return out;
}

std::string strideKinematicsSnapshotToJson(const StrideKinematicsSnapshot& s) {
    std::ostringstream o;
    o << "{\"skipped\":" << (s.skipped ? "true" : "false");
    if (!s.skip_reason.empty()) {
        o << ",\"skip_reason\":\"" << jsonEscape(s.skip_reason) << '"';
    }
    o << ",\"touchdown_events\":" << s.touchdown_events << ','
      << "\"planned_touchdown_events\":" << s.planned_touchdown_events << ','
      << "\"fused_touchdown_events\":" << s.fused_touchdown_events << ','
      << "\"planned_liftoff_events\":" << s.planned_liftoff_events << ','
      << "\"fused_liftoff_events\":" << s.fused_liftoff_events << ','
      << "\"median_touchdown_span_m\":" << formatDouble(s.median_touchdown_span_m) << ','
      << "\"median_commanded_step_length_m\":" << formatDouble(s.median_commanded_step_length_m) << ','
      << "\"median_commanded_swing_height_m\":" << formatDouble(s.median_commanded_swing_height_m) << ','
      << "\"swing_lift_percentile_value_m\":" << formatDouble(s.swing_lift_percentile_value_m) << ','
      << "\"gate_touchdown_span_min_m\":" << formatDouble(s.gate_touchdown_span_min_m) << ','
      << "\"gate_touchdown_span_max_m\":" << formatDouble(s.gate_touchdown_span_max_m) << ','
      << "\"gate_swing_lift_floor_m\":" << formatDouble(s.gate_swing_lift_floor_m) << ','
      << "\"gate_swing_lift_percentile\":" << formatDouble(s.gate_swing_lift_percentile) << '}';
    return o.str();
}

std::string walkLimitsAppliedToJson(const bool lateral_style,
                                    const double lim_cmd_track,
                                    const double lim_anchor,
                                    const double lim_stance_track) {
    std::ostringstream o;
    o << "{\"gate_profile\":\"" << jsonEscape(lateral_style ? "lateral" : "forward_like") << "\","
      << "\"fk_foot_tip_ground_min_m\":" << formatDouble(g_foot_tip_ground_tolerance_m) << ','
      << "\"max_commanded_tracking_error_m\":" << formatDouble(lim_cmd_track) << ','
      << "\"max_contact_anchor_max_drift_m\":" << formatDouble(lim_anchor) << ','
      << "\"max_contact_tracking_error_m\":" << formatDouble(lim_stance_track) << ','
      << "\"min_measured_foot_world_z_m\":" << formatDouble(g_min_measured_foot_world_z_m) << ','
      << "\"touchdown_span_min_vs_step_length\":" << formatDouble(g_touchdown_span_min_vs_step_length) << ','
      << "\"touchdown_span_max_vs_step_length\":" << formatDouble(g_touchdown_span_max_vs_step_length) << ','
      << "\"touchdown_span_max_extra_m\":" << formatDouble(g_touchdown_span_max_extra_m) << ','
      << "\"swing_lift_min_vs_swing_height\":" << formatDouble(g_swing_lift_min_vs_swing_height) << ','
      << "\"swing_lift_abs_floor_m\":" << formatDouble(g_swing_lift_abs_floor_m) << ','
      << "\"swing_lift_percentile\":" << formatDouble(g_swing_lift_percentile) << '}';
    return o.str();
}

std::string standLimitsAppliedToJson(const std::string& case_name) {
    const bool is_single_leg = case_name == "single_leg_masked_stand";
    const double path_max =
        is_single_leg ? g_single_leg_masked_stand_path_length_m_max : g_stand_settle_path_length_m_max;
    const double displacement_max =
        is_single_leg ? g_single_leg_masked_stand_net_displacement_m_max : g_stand_settle_net_displacement_m_max;
    const double max_tilt =
        is_single_leg ? g_single_leg_masked_stand_max_abs_tilt_rad : g_stand_settle_max_abs_tilt_rad;
    const double max_body_rate =
        is_single_leg ? g_single_leg_masked_stand_max_body_rate_radps : g_stand_settle_max_body_rate_radps;
    const double max_anchor =
        is_single_leg ? g_single_leg_masked_stand_max_contact_anchor_max_drift_m : g_stand_settle_max_contact_anchor_max_drift_m;
    const double max_track =
        is_single_leg ? g_single_leg_masked_stand_max_commanded_tracking_error_m : g_stand_settle_max_commanded_tracking_error_m;
    const double min_foot_z =
        is_single_leg ? g_single_leg_masked_stand_min_measured_foot_world_z_m : g_stand_settle_min_measured_foot_world_z_m;
    std::ostringstream o;
    o << "{\"gate_profile\":\"" << jsonEscape(case_name) << "\","
      << "\"path_length_m_max\":" << formatDouble(path_max) << ','
      << "\"net_displacement_m_max\":" << formatDouble(displacement_max) << ','
      << "\"max_abs_tilt_rad\":" << formatDouble(max_tilt) << ','
      << "\"max_body_rate_radps\":" << formatDouble(max_body_rate) << ','
      << "\"max_contact_anchor_max_drift_m\":" << formatDouble(max_anchor) << ','
      << "\"max_commanded_tracking_error_m\":" << formatDouble(max_track) << ','
      << "\"min_measured_foot_world_z_m\":" << formatDouble(min_foot_z) << '}';
    return o.str();
}

void emitMotionPerformanceJsonLine(const std::string& case_name,
                                   const bool passed,
                                   const LocomotionMetrics& metrics,
                                   const bool any_walk,
                                   const bool lateral_style,
                                   const double lim_cmd_track,
                                   const double lim_anchor,
                                   const double lim_stance_track,
                                   const StrideKinematicsSnapshot& stride,
                                   const double min_fk_foot_tip_world_z_m) {
    std::cout << "{\"suite\":\"motion_performance\",\"name\":\"" << jsonEscape(case_name) << "\",\"passed\":"
              << (passed ? "true" : "false") << ",\"metrics\":" << metricsToJson(metrics) << ',';
    if (any_walk) {
        std::cout << "\"limits_applied\":" << walkLimitsAppliedToJson(lateral_style, lim_cmd_track, lim_anchor, lim_stance_track)
                  << ",\"stride_kinematics\":" << strideKinematicsSnapshotToJson(stride) << ','
                  << "\"min_fk_foot_tip_world_z_m\":" << formatDouble(min_fk_foot_tip_world_z_m);
    } else {
        std::cout << "\"limits_applied\":" << standLimitsAppliedToJson(case_name) << ','
                  << "\"stride_kinematics\":{\"skipped\":true,\"skip_reason\":\"no_walk_samples\"},"
                  << "\"min_fk_foot_tip_world_z_m\":null";
    }
    std::cout << "}\n";
}

struct CaseSpec {
    std::string name{};
    std::vector<MotionPhase> phases{};
};

bool isLateralOrDiagonalCompassName(const std::string& name) {
    return name.find("strafe") != std::string::npos || name.find("diag") != std::string::npos;
}

CaseSpec caseStandSettle(const std::size_t stand_steps) {
    ScenarioMotionIntent stand{};
    stand.enabled = true;
    stand.mode = RobotMode::STAND;
    stand.gait = GaitType::TRIPOD;
    stand.body_height_m = 0.14;
    stand.speed_mps = 0.0;
    CaseSpec spec;
    spec.name = "stand_settle";
    spec.phases.push_back(makePhase("stand", stand, stand_steps));
    return spec;
}

CaseSpec caseSlowTripodForward(const std::size_t stand_steps, const std::size_t walk_steps) {
    ScenarioMotionIntent stand{};
    stand.enabled = true;
    stand.mode = RobotMode::STAND;
    stand.gait = GaitType::TRIPOD;
    stand.body_height_m = 0.14;
    stand.speed_mps = 0.0;

    ScenarioMotionIntent walk = stand;
    walk.mode = RobotMode::WALK;
    walk.speed_mps = 0.06;

    CaseSpec spec;
    spec.name = "slow_tripod_forward";
    spec.phases.push_back(makePhase("stand", stand, stand_steps));
    spec.phases.push_back(makePhase("walk", walk, walk_steps));
    return spec;
}

CaseSpec caseGaitForward(const std::string& name, const GaitType gait, const std::size_t stand_steps, const std::size_t walk_steps) {
    ScenarioMotionIntent stand{};
    stand.enabled = true;
    stand.mode = RobotMode::STAND;
    stand.gait = GaitType::TRIPOD;
    stand.body_height_m = 0.14;
    stand.speed_mps = 0.0;

    ScenarioMotionIntent walk = stand;
    walk.mode = RobotMode::WALK;
    walk.gait = gait;
    walk.speed_mps = 0.06;

    CaseSpec spec;
    spec.name = name;
    spec.phases.push_back(makePhase("stand", stand, stand_steps));
    spec.phases.push_back(makePhase("walk", walk, walk_steps));
    return spec;
}

CaseSpec caseSingleLegMasked(const std::filesystem::path& scenario_path) {
    CaseSpec spec;
    spec.name = "single_leg_masked_stand";
    spec.phases = buildPhasesFromScenario(scenario_path, spec.name);
    return spec;
}

/** Tripod walk along `heading_rad` in body frame (+X forward, +Y left); speed in m/s. */
CaseSpec caseCompassWalk(const std::string& name,
                         const double heading_rad,
                         const double speed_mps,
                         const std::size_t stand_steps,
                         const std::size_t walk_steps) {
    ScenarioMotionIntent stand{};
    stand.enabled = true;
    stand.mode = RobotMode::STAND;
    stand.gait = GaitType::TRIPOD;
    stand.body_height_m = 0.14;
    stand.speed_mps = 0.0;

    ScenarioMotionIntent walk = stand;
    walk.mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.speed_mps = speed_mps;
    walk.heading_rad = heading_rad;

    CaseSpec spec;
    spec.name = name;
    spec.phases.push_back(makePhase("stand", stand, stand_steps));
    spec.phases.push_back(makePhase("compass_walk", walk, walk_steps));
    const std::size_t exit_steps = std::max<std::size_t>(20, stand_steps / 2);
    spec.phases.push_back(makePhase("stand_exit", stand, exit_steps));
    return spec;
}

bool runCase(const std::string& sim_exe,
             const CaseSpec& spec,
             const bool emit_metrics_json,
             LocomotionMetrics& out_metrics,
             std::string& fail_reason) {
    const int port = 28000 + (static_cast<int>(::getpid()) % 3000) +
                     static_cast<int>(std::hash<std::string>{}(spec.name) % 1000);
    PhysicsSimProcess sim(sim_exe, port);
    if (!sim.start()) {
        fail_reason = "failed to start physics sim";
        if (emit_metrics_json) {
            LocomotionMetrics empty{};
            StrideKinematicsSnapshot idle{};
            idle.skip_reason = "sim_start_failed";
            emitMotionPerformanceJsonLine(
                spec.name, false, empty, false, false, 0.0, 0.0, 0.0, idle, 0.0);
        }
        return false;
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    auto bridge = std::make_unique<CapturingPhysicsSimBridge>(
        "127.0.0.1",
        port,
        harness.bus_loop_period_us,
        harness.physics_solver_iterations);
    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.control_loop_trace_enabled = false;
    cfg.telemetry.enabled = false;
    cfg.replay_log.enabled = false;

    std::vector<MotionPhase> phases = spec.phases;
    scalePhasesForHarness(phases, harness.bus_loop_period_us);

    std::vector<MotionSample> samples;
    LocomotionMetrics metrics{};
    metrics.sample_period_s = physics_sim_test_utils::controlLoopPeriodSeconds(cfg);

    RobotRuntime runtime(
        std::move(bridge),
        std::make_unique<PhysicsSimEstimator>(),
        nullptr,
        cfg,
        telemetry::makeNoopTelemetryPublisher(),
        replay::makeNoopReplayLogger());
    if (!runtime.init()) {
        fail_reason = "runtime init failed";
        if (emit_metrics_json) {
            LocomotionMetrics empty{};
            StrideKinematicsSnapshot idle{};
            idle.skip_reason = "runtime_init_failed";
            emitMotionPerformanceJsonLine(
                spec.name, false, empty, false, false, 0.0, 0.0, 0.0, idle, 0.0);
        }
        return false;
    }

    if (!runMotionSequence(runtime, phases, samples, metrics)) {
        fail_reason = "runMotionSequence failed";
        if (emit_metrics_json) {
            LocomotionMetrics empty{};
            StrideKinematicsSnapshot idle{};
            idle.skip_reason = "run_motion_sequence_failed";
            emitMotionPerformanceJsonLine(
                spec.name, false, empty, false, false, 0.0, 0.0, 0.0, idle, 0.0);
        }
        return false;
    }

    out_metrics = metrics;

    if (metrics.saw_fault && isSevereFault(metrics.first_fault)) {
        fail_reason = std::string("severe fault: ") + faultName(metrics.first_fault);
        if (emit_metrics_json) {
            StrideKinematicsSnapshot stride_idle{};
            stride_idle.skip_reason = "severe_fault";
            emitMotionPerformanceJsonLine(
                spec.name,
                false,
                metrics,
                false,
                false,
                0.0,
                0.0,
                0.0,
                stride_idle,
                0.0);
        }
        return false;
    }

    bool any_walk_sample = false;
    double min_foot_tip_world_z_m = std::numeric_limits<double>::infinity();
    for (const MotionSample& sample : samples) {
        if (sample.status.active_mode != RobotMode::WALK) {
            continue;
        }
        any_walk_sample = true;
        min_foot_tip_world_z_m = std::min(min_foot_tip_world_z_m, minFootTipWorldZ(sample.estimated));
    }

    const bool lateral_style = isLateralOrDiagonalCompassName(spec.name);
    const double lim_cmd_track =
        lateral_style ? g_max_commanded_tracking_error_m_lateral : g_max_commanded_tracking_error_m_forward_like;
    const double lim_anchor =
        lateral_style ? g_max_stance_anchor_drift_m_lateral : g_max_stance_anchor_drift_m_forward_like;
    const double lim_stance_track =
        lateral_style ? g_max_stance_contact_tracking_error_m_lateral : g_max_stance_contact_tracking_error_m_forward_like;

    StrideKinematicsSnapshot stride_snap{};
    stride_snap.skipped = true;
    stride_snap.skip_reason = "no_walk_samples";
    bool case_ok = true;

    if (any_walk_sample) {
        const StrideKinematicsEval stride_eval = evaluateStrideKinematics(samples, metrics);
        stride_snap = stride_eval.snapshot;

        if (!(min_foot_tip_world_z_m >= g_foot_tip_ground_tolerance_m)) {
            fail_reason = std::string("FK foot tips below ground plane (min_z=") + formatDouble(min_foot_tip_world_z_m) + ")";
            case_ok = false;
        } else if (metrics.max_commanded_tracking_error_m > lim_cmd_track) {
            fail_reason = std::string("commanded vs measured foot tracking too large (max_err_m=") +
                          formatDouble(metrics.max_commanded_tracking_error_m) + " limit=" + formatDouble(lim_cmd_track) + ")";
            case_ok = false;
        } else if (!(metrics.max_contact_anchor_max_drift_m <= lim_anchor)) {
            fail_reason = std::string("stance foot anchor drift too large (max_drift_m=") +
                          formatDouble(metrics.max_contact_anchor_max_drift_m) + " limit=" + formatDouble(lim_anchor) + ")";
            case_ok = false;
        } else if (!(metrics.max_contact_tracking_error_m <= lim_stance_track)) {
            fail_reason = std::string("stance commanded vs measured foot error too large (max_err_m=") +
                          formatDouble(metrics.max_contact_tracking_error_m) + " limit=" + formatDouble(lim_stance_track) + ")";
            case_ok = false;
        } else if (!(metrics.min_measured_foot_world_z_m >= g_min_measured_foot_world_z_m)) {
            fail_reason = std::string("measured foot tip too far below ground (min_world_z_m=") +
                          formatDouble(metrics.min_measured_foot_world_z_m) + ")";
            case_ok = false;
        } else if (stride_eval.failure.has_value()) {
            fail_reason = *stride_eval.failure;
            case_ok = false;
        }
    }

    if (!any_walk_sample) {
        const bool is_single_leg = spec.name == "single_leg_masked_stand";
        const double path_max =
            is_single_leg ? g_single_leg_masked_stand_path_length_m_max : g_stand_settle_path_length_m_max;
        const double displacement_max =
            is_single_leg ? g_single_leg_masked_stand_net_displacement_m_max : g_stand_settle_net_displacement_m_max;
        const double max_tilt =
            is_single_leg ? g_single_leg_masked_stand_max_abs_tilt_rad : g_stand_settle_max_abs_tilt_rad;
        const double max_body_rate =
            is_single_leg ? g_single_leg_masked_stand_max_body_rate_radps : g_stand_settle_max_body_rate_radps;
        const double max_anchor =
            is_single_leg ? g_single_leg_masked_stand_max_contact_anchor_max_drift_m : g_stand_settle_max_contact_anchor_max_drift_m;
        const double max_track =
            is_single_leg ? g_single_leg_masked_stand_max_commanded_tracking_error_m : g_stand_settle_max_commanded_tracking_error_m;
        const double min_foot_z =
            is_single_leg ? g_single_leg_masked_stand_min_measured_foot_world_z_m : g_stand_settle_min_measured_foot_world_z_m;
        if (!(metrics.path_length_m <= path_max)) {
            fail_reason = std::string("body path too large while standing (path_m=") + formatDouble(metrics.path_length_m) +
                          " limit=" + formatDouble(path_max) + ")";
            case_ok = false;
        } else if (!(metrics.net_displacement_m <= displacement_max)) {
            fail_reason = std::string("body displacement too large while standing (disp_m=") + formatDouble(metrics.net_displacement_m) +
                          " limit=" + formatDouble(displacement_max) + ")";
            case_ok = false;
        } else if (!(std::max(metrics.max_abs_roll_rad, metrics.max_abs_pitch_rad) <= max_tilt)) {
            fail_reason = std::string("tilt too large while standing (tilt_rad=") +
                          formatDouble(std::max(metrics.max_abs_roll_rad, metrics.max_abs_pitch_rad)) +
                          " limit=" + formatDouble(max_tilt) + ")";
            case_ok = false;
        } else if (!(metrics.max_body_rate_radps <= max_body_rate)) {
            fail_reason = std::string("body rate too large while standing (rate_radps=") +
                          formatDouble(metrics.max_body_rate_radps) + " limit=" + formatDouble(max_body_rate) + ")";
            case_ok = false;
        } else if (!(metrics.max_contact_anchor_max_drift_m <= max_anchor)) {
            fail_reason = std::string("stance anchor drift too large while standing (max_drift_m=") +
                          formatDouble(metrics.max_contact_anchor_max_drift_m) + " limit=" + formatDouble(max_anchor) + ")";
            case_ok = false;
        } else if (!(metrics.max_commanded_tracking_error_m <= max_track)) {
            fail_reason = std::string("commanded vs measured foot tracking too large while standing (max_err_m=") +
                          formatDouble(metrics.max_commanded_tracking_error_m) + " limit=" + formatDouble(max_track) + ")";
            case_ok = false;
        } else if (!(metrics.min_measured_foot_world_z_m >= min_foot_z)) {
            fail_reason = std::string("measured foot tip too low while standing (min_world_z_m=") +
                          formatDouble(metrics.min_measured_foot_world_z_m) + " limit=" + formatDouble(min_foot_z) + ")";
            case_ok = false;
        }
    }

    if (emit_metrics_json) {
        emitMotionPerformanceJsonLine(spec.name,
                                      case_ok,
                                      metrics,
                                      any_walk_sample,
                                      lateral_style,
                                      lim_cmd_track,
                                      lim_anchor,
                                      lim_stance_track,
                                      stride_snap,
                                      min_foot_tip_world_z_m);
    } else if (case_ok) {
        std::cout << spec.name << " ok samples=" << metrics.sample_count << " path_m=" << formatDouble(metrics.path_length_m)
                  << " strides=" << metrics.stride_count << " tilt_max=" << formatDouble(std::max(metrics.max_abs_roll_rad, metrics.max_abs_pitch_rad))
                  << '\n';
    }
    return case_ok;
}

void printUsage() {
    std::cerr << "Usage: test_motion_performance_suite [--sim PATH] [--profile smoke|full] [--case NAME] [--list] [--emit-metrics-json]\n";
    std::cerr << "Compass cases (body-frame velocity via heading): compass_forward, compass_backward, compass_strafe_left,\n";
    std::cerr << "  compass_strafe_right, compass_diag_fwd_left, compass_diag_fwd_right\n";
}

std::filesystem::path resolveScenario07() {
    const std::vector<std::filesystem::path> candidates{
        "scenarios/07_single_leg_probe.toml",
        "../scenarios/07_single_leg_probe.toml",
        "hexapod-server/scenarios/07_single_leg_probe.toml",
    };
    for (const auto& c : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(c, ec)) {
            return std::filesystem::weakly_canonical(c, ec);
        }
    }
    return {};
}

} // namespace

int main(int argc, char** argv) {
    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }
    refreshMotionPerformanceLimitsFromManifest();

    std::string sim_exe;
    std::string profile = "smoke";
    std::string single_case;
    bool list_only = false;
    bool emit_metrics_json = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sim") == 0 && i + 1 < argc) {
            sim_exe = argv[++i];
        } else if (std::strcmp(argv[i], "--limits-manifest") == 0 && i + 1 < argc) {
            ++i;
        } else if (std::strcmp(argv[i], "--profile") == 0 && i + 1 < argc) {
            profile = argv[++i];
        } else if (std::strcmp(argv[i], "--case") == 0 && i + 1 < argc) {
            single_case = argv[++i];
        } else if (std::strcmp(argv[i], "--list") == 0) {
            list_only = true;
        } else if (std::strcmp(argv[i], "--emit-metrics-json") == 0) {
            emit_metrics_json = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            printUsage();
            return 0;
        }
    }

    if (list_only) {
        std::cout << "stand_settle\nslow_tripod_forward\ngait_compare_wave\ngait_compare_ripple\n"
                     "compass_forward\ncompass_backward\ncompass_strafe_left\ncompass_strafe_right\n"
                     "compass_diag_fwd_left\ncompass_diag_fwd_right\nsingle_leg_masked_stand\n";
        return 0;
    }

    if (sim_exe.empty()) {
        if (const char* env = std::getenv("HEXAPOD_PHYSICS_SIM_EXE")) {
            sim_exe = env;
        }
    }
#if defined(__linux__)
    if (sim_exe.empty()) {
        std::cout << "skip test_motion_performance_suite (pass --sim or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }
#else
    std::cout << "skip test_motion_performance_suite (linux only)\n";
    return 0;
#endif

    const bool smoke = (profile == "smoke");
    const std::size_t stand_steps_smoke = 40;
    const std::size_t walk_steps_smoke = 120;
    const std::size_t stand_steps_full = 80;
    const std::size_t walk_steps_full = 400;
    constexpr double kCompassWalkSpeedSmoke = 0.05;
    constexpr double kCompassWalkSpeedFull = 0.05;

    std::vector<CaseSpec> full_catalog;
    full_catalog.push_back(caseStandSettle(stand_steps_full));
    full_catalog.push_back(caseSlowTripodForward(stand_steps_full, walk_steps_full));
    full_catalog.push_back(caseGaitForward("gait_compare_wave", GaitType::WAVE, stand_steps_full, walk_steps_full));
    full_catalog.push_back(caseGaitForward("gait_compare_ripple", GaitType::RIPPLE, stand_steps_full, walk_steps_full));
    full_catalog.push_back(
        caseCompassWalk("compass_forward", 0.0, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    full_catalog.push_back(
        caseCompassWalk("compass_backward", M_PI, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    full_catalog.push_back(
        caseCompassWalk("compass_strafe_left", M_PI / 2.0, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    full_catalog.push_back(
        caseCompassWalk("compass_strafe_right", -M_PI / 2.0, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    full_catalog.push_back(
        caseCompassWalk("compass_diag_fwd_left", M_PI / 4.0, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    full_catalog.push_back(caseCompassWalk(
        "compass_diag_fwd_right", -M_PI / 4.0, kCompassWalkSpeedFull, stand_steps_full, walk_steps_full));
    const auto scenario07 = resolveScenario07();
    if (!scenario07.empty()) {
        full_catalog.push_back(caseSingleLegMasked(scenario07));
    }

    std::vector<CaseSpec> smoke_catalog;
    smoke_catalog.push_back(caseStandSettle(stand_steps_smoke));
    smoke_catalog.push_back(caseSlowTripodForward(stand_steps_smoke, walk_steps_smoke));
    smoke_catalog.push_back(
        caseCompassWalk("compass_forward", 0.0, kCompassWalkSpeedSmoke, stand_steps_smoke, walk_steps_smoke));
    smoke_catalog.push_back(
        caseCompassWalk("compass_strafe_left", M_PI / 2.0, kCompassWalkSpeedSmoke, stand_steps_smoke, walk_steps_smoke));

    std::vector<CaseSpec> catalog = smoke ? smoke_catalog : full_catalog;

    if (!single_case.empty()) {
        const std::size_t stand_s = smoke ? stand_steps_smoke : stand_steps_full;
        const std::size_t walk_s = smoke ? walk_steps_smoke : walk_steps_full;
        CaseSpec chosen;
        bool found = false;
        if (single_case == "stand_settle") {
            chosen = caseStandSettle(stand_s);
            found = true;
        } else if (single_case == "slow_tripod_forward") {
            chosen = caseSlowTripodForward(stand_s, walk_s);
            found = true;
        } else if (single_case == "gait_compare_wave") {
            chosen = caseGaitForward("gait_compare_wave", GaitType::WAVE, stand_s, walk_s);
            found = true;
        } else if (single_case == "gait_compare_ripple") {
            chosen = caseGaitForward("gait_compare_ripple", GaitType::RIPPLE, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_forward") {
            chosen = caseCompassWalk("compass_forward", 0.0, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_backward") {
            chosen = caseCompassWalk("compass_backward", M_PI, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_strafe_left") {
            chosen = caseCompassWalk(
                "compass_strafe_left", M_PI / 2.0, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_strafe_right") {
            chosen = caseCompassWalk(
                "compass_strafe_right", -M_PI / 2.0, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_diag_fwd_left") {
            chosen = caseCompassWalk(
                "compass_diag_fwd_left", M_PI / 4.0, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "compass_diag_fwd_right") {
            chosen = caseCompassWalk(
                "compass_diag_fwd_right", -M_PI / 4.0, smoke ? kCompassWalkSpeedSmoke : kCompassWalkSpeedFull, stand_s, walk_s);
            found = true;
        } else if (single_case == "single_leg_masked_stand") {
            if (!scenario07.empty()) {
                chosen = caseSingleLegMasked(scenario07);
                found = true;
            }
        }
        if (!found) {
            std::cerr << "unknown or unavailable case: " << single_case << '\n';
            return 2;
        }
        LocomotionMetrics metrics{};
        std::string reason;
        const bool ok = runCase(sim_exe, chosen, emit_metrics_json, metrics, reason);
        if (!ok) {
            std::cerr << chosen.name << " FAIL: " << reason << '\n';
            std::cerr << chosen.name << " metrics: first_fault=" << faultName(metrics.first_fault)
                      << " first_fault_step=" << metrics.first_fault_step
                      << " max_body_rate_radps=" << formatDouble(metrics.max_body_rate_radps)
                      << " max_abs_roll_rad=" << formatDouble(metrics.max_abs_roll_rad)
                      << " max_abs_pitch_rad=" << formatDouble(metrics.max_abs_pitch_rad) << '\n';
        }
        return ok ? 0 : 1;
    }

    int failures = 0;
    for (const CaseSpec& spec : catalog) {
        LocomotionMetrics metrics{};
        std::string reason;
        const bool ok = runCase(sim_exe, spec, emit_metrics_json, metrics, reason);
        if (!ok) {
            ++failures;
            std::cerr << spec.name << " FAIL: " << reason << '\n';
            std::cerr << spec.name << " metrics: first_fault=" << faultName(metrics.first_fault)
                      << " first_fault_step=" << metrics.first_fault_step
                      << " max_body_rate_radps=" << formatDouble(metrics.max_body_rate_radps)
                      << " max_abs_roll_rad=" << formatDouble(metrics.max_abs_roll_rad)
                      << " max_abs_pitch_rad=" << formatDouble(metrics.max_abs_pitch_rad) << '\n';
        }
    }

    return failures > 0 ? 1 : 0;
}
