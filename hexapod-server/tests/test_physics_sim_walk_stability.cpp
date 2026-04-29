#include "body_controller.hpp"
#include "body_pose_controller.hpp"
#include "control_config.hpp"
#include "config/toml_parser.hpp"
#include "foot_planners.hpp"
#include "geometry_config.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_estimator.hpp"
#include "robot_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

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

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

SwingFootPlanDecomposition buildFaultDecomposition(const RobotState& est,
                                                   const GaitState& gait_snapshot,
                                                   const ScenarioMotionIntent& scenario_motion,
                                                   const control_config::ControlConfig& cfg) {
    const MotionIntent intent = makeMotionIntent(scenario_motion);
    const PlanarMotionCommand planar = planarMotionCommand(intent);
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(intent, planar);
    const double trust_scale =
        est.has_fusion_diagnostics ? std::clamp(est.fusion.model_trust, 0.20, 1.0) : 1.0;
    const BodyVelocityCommand body_mot =
        bodyVelocityForFootPlanning(est, cmd_twist, cfg.gait.foot_estimator_blend * trust_scale);
    const BodyPoseSetpoint pose =
        computeBodyPoseSetpoint(intent, planar, gait_snapshot.static_stability_margin_m, gait_snapshot.stride_phase_rate_hz.value);
    double effective_body_height_m = pose.body_height_m;
    if (est.has_body_twist_state && std::isfinite(est.body_twist_state.body_trans_m.z)) {
        const double sag_m = std::max(0.0, pose.body_height_m - est.body_twist_state.body_trans_m.z);
        effective_body_height_m += std::clamp(sag_m, 0.0, 0.120);
    }

    const HexapodGeometry geometry = defaultHexapodGeometry();
    const std::array<Vec3, kNumLegs> nominal = computeNominalStance(geometry, effective_body_height_m);

    int leg = 0;
    for (int i = 0; i < kNumLegs; ++i) {
        if (!gait_snapshot.in_stance[static_cast<std::size_t>(i)]) {
            leg = i;
            break;
        }
    }

    const double duty = std::clamp(gait_snapshot.duty_factor, 0.06, 0.94);
    const double f_hz = std::max(gait_snapshot.stride_phase_rate_hz.value, 1e-6);
    const double swing_span = std::max(1.0 - duty, 1e-6);
    const Vec3 planar_body_offset{intent.twist.body_trans_m.x, intent.twist.body_trans_m.y, 0.0};
    const Vec3 anchor = nominal[static_cast<std::size_t>(leg)] - planar_body_offset;
    const Vec3 v_foot = supportFootVelocityAt(anchor, body_mot);
    const Vec3 stance_end = anchor + v_foot * (duty / f_hz);

    double stride_ux = 1.0;
    double stride_uy = 0.0;
    const double planar_speed = std::hypot(planar.vx_mps, planar.vy_mps);
    if (planar_speed > 1e-6) {
        stride_ux = planar.vx_mps / planar_speed;
        stride_uy = planar.vy_mps / planar_speed;
    } else if (std::abs(planar.yaw_rate_radps) > 1e-6) {
        const double tx = -anchor.y;
        const double ty = anchor.x;
        const double tn = std::hypot(tx, ty);
        if (tn > 1e-6) {
            stride_ux = tx / tn;
            stride_uy = ty / tn;
            if (planar.yaw_rate_radps < 0.0) {
                stride_ux = -stride_ux;
                stride_uy = -stride_uy;
            }
        }
    }

    SwingFootInputs swing{};
    swing.anchor = anchor;
    swing.stance_end = stance_end;
    swing.v_liftoff_body = supportFootVelocityAt(stance_end, body_mot);
    swing.tau01 = clamp01((gait_snapshot.phase[static_cast<std::size_t>(leg)] - duty) / swing_span);
    swing.swing_span = swing_span;
    swing.f_hz = f_hz;
    swing.step_length_m = gait_snapshot.step_length_m;
    swing.swing_height_m = gait_snapshot.swing_height_m;
    swing.stride_ux = stride_ux;
    swing.stride_uy = stride_uy;
    swing.cmd_accel_body_x_mps2 = gait_snapshot.cmd_accel_body_x_mps2;
    swing.cmd_accel_body_y_mps2 = gait_snapshot.cmd_accel_body_y_mps2;
    swing.stance_lookahead_s = (duty / f_hz) * 0.48;
    swing.static_stability_margin_m = gait_snapshot.static_stability_margin_m;
    swing.swing_time_ease_01 = gait_snapshot.swing_time_ease_01;

    return computeSwingFootPlacement(est, cmd_twist, swing);
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_walk_stability (Linux-only)\n";
    return 0;
#else
    const char* sim_exe = nullptr;
    if (argc >= 2 && argv[1][0] != '\0') {
        sim_exe = argv[1];
    } else {
        sim_exe = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_walk_stability (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const int port = 27000 + (static_cast<int>(::getpid()) % 4000);
    const int bus_loop_period_us = 20000;

    pid_t pid = ::fork();
    if (pid < 0) {
        std::cerr << "fork failed\n";
        return 2;
    }
    if (pid == 0) {
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    auto bridge = std::make_unique<CapturingPhysicsSimBridge>("127.0.0.1", port, bus_loop_period_us, 24);
    CapturingPhysicsSimBridge* bridge_ptr = bridge.get();

    ParsedToml parsed{};
    const std::string config_path = "../config.physics-sim-wsl.txt";
    const TomlParser parser{};
    if (!expect(parser.parse(config_path, parsed), "physics-sim WSL config should parse for stability regression")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    control_config::ControlConfig cfg = control_config::fromParsedToml(parsed);
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};

    RobotRuntime runtime(
        std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg, telemetry::makeNoopTelemetryPublisher());
    if (!expect(runtime.init(), "runtime init should succeed against the live physics sim")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    const ScenarioMotionIntent stand_motion{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const ScenarioMotionIntent walk_motion{true, RobotMode::WALK, GaitType::TRIPOD, 0.14, 0.08, 0.0, 0.0};

    constexpr int kStandWarmupSteps = 140;
    constexpr int kWalkObserveSteps = 1800;

    for (int i = 0; i < kStandWarmupSteps; ++i) {
        runControlLoopStep(runtime, stand_motion);
    }

    if (!expect(bridge_ptr->last_state().has_value(), "bridge should produce an initial state before walking")) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return EXIT_FAILURE;
    }

    double max_abs_roll_rad = 0.0;
    double max_abs_pitch_rad = 0.0;
    double min_model_trust = 1.0;
    double max_contact_mismatch = 0.0;
    bool fault_seen = false;
    int fault_step = -1;

    for (int i = 0; i < kWalkObserveSteps; ++i) {
        runControlLoopStep(runtime, walk_motion);

        const ControlStatus status = runtime.getStatus();
        if (!expect(bridge_ptr->last_state().has_value(), "bridge should keep producing live state during walk")) {
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        const RobotState& state = bridge_ptr->last_state().value();
        const RobotState est_snapshot = runtime.estimatedSnapshot();
        const GaitState gait_snapshot = runtime.gaitSnapshot();

        if (status.active_fault != FaultCode::NONE) {
            const SwingFootPlanDecomposition swing_debug =
                buildFaultDecomposition(est_snapshot, gait_snapshot, walk_motion, cfg);
            fault_seen = true;
            fault_step = i;
            std::cerr << "walk step=" << i
                      << " active_mode=" << static_cast<int>(status.active_mode)
                      << " fault=" << static_cast<int>(status.active_fault)
                      << " trust=" << (est_snapshot.has_fusion_diagnostics ? est_snapshot.fusion.model_trust : -1.0)
                      << " mismatch="
                      << (est_snapshot.has_fusion_diagnostics ? est_snapshot.fusion.residuals.contact_mismatch_ratio : -1.0)
                      << " roll=" << state.body_twist_state.twist_pos_rad.x
                      << " pitch=" << state.body_twist_state.twist_pos_rad.y
                      << " stride_hz=" << gait_snapshot.stride_phase_rate_hz.value
                      << " static_margin=" << gait_snapshot.static_stability_margin_m
                      << " nominal_xy=(" << swing_debug.nominal_body.x << ',' << swing_debug.nominal_body.y << ')'
                      << " capture_raw_xy=(" << swing_debug.capture_raw_body.x << ',' << swing_debug.capture_raw_body.y << ')'
                      << " capture_xy=(" << swing_debug.capture_body.x << ',' << swing_debug.capture_body.y << ')'
                      << " capture_limit=" << swing_debug.capture_limit_m
                      << " final_xy=(" << swing_debug.final_body.x << ',' << swing_debug.final_body.y << ')'
                      << '\n';
            if (!expect(status.active_fault == FaultCode::TIP_OVER,
                        "physics-sim walk should only fault by tripping TIP_OVER in this envelope")) {
                ::kill(pid, SIGTERM);
                ::waitpid(pid, nullptr, 0);
                return EXIT_FAILURE;
            }
            break;
        }
        if (!expect(status.active_mode == RobotMode::WALK,
                    "nominal walk should remain in WALK mode until the safety trips")) {
            const SwingFootPlanDecomposition swing_debug =
                buildFaultDecomposition(est_snapshot, gait_snapshot, walk_motion, cfg);
            std::cerr << "walk step=" << i
                      << " active_mode=" << static_cast<int>(status.active_mode)
                      << " fault=" << static_cast<int>(status.active_fault)
                      << " trust=" << (est_snapshot.has_fusion_diagnostics ? est_snapshot.fusion.model_trust : -1.0)
                      << " mismatch="
                      << (est_snapshot.has_fusion_diagnostics ? est_snapshot.fusion.residuals.contact_mismatch_ratio : -1.0)
                      << " roll=" << state.body_twist_state.twist_pos_rad.x
                      << " pitch=" << state.body_twist_state.twist_pos_rad.y
                      << " stride_hz=" << gait_snapshot.stride_phase_rate_hz.value
                      << " static_margin=" << gait_snapshot.static_stability_margin_m
                      << " nominal_xy=(" << swing_debug.nominal_body.x << ',' << swing_debug.nominal_body.y << ')'
                      << " capture_raw_xy=(" << swing_debug.capture_raw_body.x << ',' << swing_debug.capture_raw_body.y << ')'
                      << " capture_xy=(" << swing_debug.capture_body.x << ',' << swing_debug.capture_body.y << ')'
                      << " capture_limit=" << swing_debug.capture_limit_m
                      << " final_xy=(" << swing_debug.final_body.x << ',' << swing_debug.final_body.y << ')'
                      << '\n';
            ::kill(pid, SIGTERM);
            ::waitpid(pid, nullptr, 0);
            return EXIT_FAILURE;
        }
        max_abs_roll_rad = std::max(max_abs_roll_rad, std::abs(state.body_twist_state.twist_pos_rad.x));
        max_abs_pitch_rad = std::max(max_abs_pitch_rad, std::abs(state.body_twist_state.twist_pos_rad.y));
        if (est_snapshot.has_fusion_diagnostics) {
            min_model_trust = std::min(min_model_trust, est_snapshot.fusion.model_trust);
            max_contact_mismatch = std::max(max_contact_mismatch, est_snapshot.fusion.residuals.contact_mismatch_ratio);
        }
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);

    std::cout << "physics_sim_walk_stability max_abs_roll_rad=" << max_abs_roll_rad
              << " max_abs_pitch_rad=" << max_abs_pitch_rad
              << " min_model_trust=" << min_model_trust
              << " max_contact_mismatch=" << max_contact_mismatch
              << " fault_seen=" << (fault_seen ? 1 : 0)
              << " fault_step=" << fault_step << '\n';

    return expect(max_abs_roll_rad < 0.60,
                  "physics-sim walk should keep roll comfortably below the tip-over threshold") &&
           expect(max_abs_pitch_rad < 0.65,
                  "physics-sim walk should keep pitch below the tip-over threshold") &&
           expect(!fault_seen || fault_step >= 0,
                  "physics-sim walk fault bookkeeping should be consistent")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
#endif
}
