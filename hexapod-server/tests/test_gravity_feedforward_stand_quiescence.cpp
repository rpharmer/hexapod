/**
 * Stand quiescence: compare metrics with gravity feedforward off vs on.
 * Linux + HEXAPOD_PHYSICS_SIM_EXE (or argv --sim); otherwise skip exit 0.
 *
 * Metrics (raw sim bridge state after each bus read):
 * - max_abs_body_world_z_m — drift of chassis world Z vs reference (sim maps body_position into server frame).
 * - rms_joint_vel_radps — RMS joint velocity over stance legs (femur/tibia/coxa).
 * - rms_femur_tibia_tracking_rad — RMS |cmd−meas| on femur/tibia for stance legs (cmd from last bus write).
 *
 * Optional env (harness / sweeps):
 * - HEXAPOD_FF_SCALE_FEMUR, HEXAPOD_FF_SCALE_TIBIA (default tuned 0.52)
 * - HEXAPOD_FF_STIFFNESS_GAIN_SCALE (default 0.62)
 * - HEXAPOD_FF_DELTA_LPF_TAU_S (default 0.05)
 * - HEXAPOD_FF_USE_CODE_DEFAULTS — if set, use struct defaults (1,1,1,0 LPF) instead of harness tuning
 * - HEXAPOD_FF_CSV — print one CSV summary line to stderr
 */

#include "control_config.hpp"
#include "kinematics/math_types.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_estimator.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_test_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "robot_runtime.hpp"
#include "scenario_driver.hpp"
#include "telemetry_json.hpp"
#include "test_limits_manifest.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
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
    }
    return condition;
}

struct MeasureWindow {
    bool active{false};
    double sum_sq_femur_tibia_tracking{0.0};
    int n_femur_tibia_tracking{0};
};

class StandQuiescenceBridge final : public IHardwareBridge {
public:
    StandQuiescenceBridge(std::string host,
                          int port,
                          int bus_loop_period_us,
                          int physics_solver_iterations,
                          MeasureWindow* measure)
        : inner_(std::move(host), port, bus_loop_period_us, physics_solver_iterations, nullptr),
          measure_(measure) {}

    bool init() override { return inner_.init(); }

    bool read(RobotState& out) override {
        const bool ok = inner_.read(out);
        if (ok) {
            last_state_ = out;
            if (measure_ != nullptr && measure_->active && have_last_cmd_) {
                for (int leg = 0; leg < kNumLegs; ++leg) {
                    if (!out.foot_contacts[static_cast<std::size_t>(leg)]) {
                        continue;
                    }
                    for (const int j : {FEMUR, TIBIA}) {
                        const double cmd =
                            last_cmd_.leg_states[static_cast<std::size_t>(leg)]
                                .joint_state[static_cast<std::size_t>(j)]
                                .pos_rad.value;
                        const double meas =
                            out.leg_states[static_cast<std::size_t>(leg)]
                                .joint_state[static_cast<std::size_t>(j)]
                                .pos_rad.value;
                        const double e = std::abs(shortestAngleDeltaRad(cmd, meas));
                        measure_->sum_sq_femur_tibia_tracking += e * e;
                        ++measure_->n_femur_tibia_tracking;
                    }
                }
            }
        }
        return ok;
    }

    bool write(const JointTargets& in) override {
        last_cmd_ = in;
        have_last_cmd_ = true;
        return inner_.write(in);
    }

    std::optional<BridgeCommandResultMetadata> last_bridge_result() const override {
        return inner_.last_bridge_result();
    }

    const std::optional<RobotState>& last_state() const { return last_state_; }

private:
    PhysicsSimBridge inner_;
    std::optional<RobotState> last_state_{};
    JointTargets last_cmd_{};
    bool have_last_cmd_{false};
    MeasureWindow* measure_{nullptr};
};

void runControlLoopStep(RobotRuntime& runtime, const ScenarioMotionIntent& motion) {
    runtime.setMotionIntent(makeMotionIntent(motion));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();
}

struct StandMetrics {
    double max_abs_body_world_z_m{0.0};
    double rms_joint_vel_radps{0.0};
    double rms_femur_tibia_tracking_rad{0.0};
};

double envDouble(const char* key, double fallback) {
    const char* s = std::getenv(key);
    if (s == nullptr || s[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double v = std::strtod(s, &end);
    if (end == s) {
        return fallback;
    }
    return v;
}

bool envFlag(const char* key) {
    const char* s = std::getenv(key);
    return s != nullptr && s[0] != '\0' && std::strcmp(s, "0") != 0;
}

void applyHarnessFfTuning(control_config::GravityFeedforwardConfig& g) {
    if (envFlag("HEXAPOD_FF_USE_CODE_DEFAULTS")) {
        return;
    }
    g.scale_coxa = 0.0;
    g.scale_femur = envDouble("HEXAPOD_FF_SCALE_FEMUR", 0.30);
    g.scale_tibia = envDouble("HEXAPOD_FF_SCALE_TIBIA", 0.30);
    g.stiffness_gain_scale = envDouble("HEXAPOD_FF_STIFFNESS_GAIN_SCALE", 0.62);
    g.delta_lpf_tau_s = envDouble("HEXAPOD_FF_DELTA_LPF_TAU_S", 0.08);
}

StandMetrics measureStandQuiescence(const std::string& sim_exe,
                                    bool gravity_ff_enabled,
                                    int port,
                                    const physics_sim_test_utils::HarnessSettings& harness) {
    StandMetrics out{};
#if !defined(__linux__)
    (void)sim_exe;
    (void)gravity_ff_enabled;
    (void)port;
    (void)harness;
    return out;
#else
    pid_t pid = ::fork();
    if (pid < 0) {
        return out;
    }
    if (pid == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string port_str = std::to_string(port);
        ::execl(sim_exe.c_str(), sim_exe.c_str(), "--serve", "--serve-port", port_str.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    MeasureWindow measure{};
    auto bridge = std::make_unique<StandQuiescenceBridge>(
        "127.0.0.1", port, harness.bus_loop_period_us, harness.physics_solver_iterations, &measure);
    StandQuiescenceBridge* bridge_ptr = bridge.get();

    control_config::ControlConfig cfg = harness.control_cfg;
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.gravity_feedforward.enabled = gravity_ff_enabled;
    cfg.gravity_feedforward.include_foot_reaction = true;
    cfg.gravity_feedforward.include_self_weight = false;
    if (gravity_ff_enabled) {
        applyHarnessFfTuning(cfg.gravity_feedforward);
    }

    RobotRuntime runtime(std::move(bridge), std::make_unique<PhysicsSimEstimator>(), nullptr, cfg,
                         telemetry::makeNoopTelemetryPublisher());
    if (!runtime.init()) {
        ::kill(pid, SIGTERM);
        ::waitpid(pid, nullptr, 0);
        return out;
    }

    const ScenarioMotionIntent stand{true, RobotMode::STAND, GaitType::TRIPOD, 0.14, 0.0, 0.0, 0.0};
    const int warm =
        static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(240, harness.bus_loop_period_us));
    const int measure_steps =
        static_cast<int>(physics_sim_test_utils::scaledLegacyStepCount(800, harness.bus_loop_period_us));

    for (int i = 0; i < warm; ++i) {
        runControlLoopStep(runtime, stand);
    }

    double ref_z = 0.0;
    if (bridge_ptr->last_state().has_value() && bridge_ptr->last_state()->has_body_twist_state) {
        ref_z = bridge_ptr->last_state()->body_twist_state.body_trans_m.z;
    }

    measure.active = true;
    double sum_sq_vel = 0.0;
    int n_vel = 0;
    for (int i = 0; i < measure_steps; ++i) {
        runControlLoopStep(runtime, stand);
        if (!bridge_ptr->last_state().has_value()) {
            continue;
        }
        const RobotState& st = *bridge_ptr->last_state();
        if (st.has_body_twist_state) {
            out.max_abs_body_world_z_m =
                std::max(out.max_abs_body_world_z_m, std::abs(st.body_twist_state.body_trans_m.z - ref_z));
        }
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!st.foot_contacts[static_cast<std::size_t>(leg)]) {
                continue;
            }
            for (int j = 0; j < kJointsPerLeg; ++j) {
                const double v = st.leg_states[static_cast<std::size_t>(leg)].joint_state[static_cast<std::size_t>(j)]
                                     .vel_radps.value;
                sum_sq_vel += v * v;
                ++n_vel;
            }
        }
    }
    measure.active = false;
    if (n_vel > 0) {
        out.rms_joint_vel_radps = std::sqrt(sum_sq_vel / static_cast<double>(n_vel));
    }
    if (measure.n_femur_tibia_tracking > 0) {
        out.rms_femur_tibia_tracking_rad =
            std::sqrt(measure.sum_sq_femur_tibia_tracking / static_cast<double>(measure.n_femur_tibia_tracking));
    }

    ::kill(pid, SIGTERM);
    ::waitpid(pid, nullptr, 0);
    return out;
#endif
}

double primaryScore(const StandMetrics& m) {
    // Weight tracking lightly vs vertical drift and velocity (cmd–meas includes bus-step latency).
    return 0.28 * m.rms_femur_tibia_tracking_rad + m.max_abs_body_world_z_m + 0.022 * m.rms_joint_vel_radps;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_gravity_feedforward_stand_quiescence (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe_c = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe_c);
    std::string manifest_err;
    if (!test_limits::init(argc, argv, manifest_err)) {
        std::cerr << manifest_err << '\n';
        return 2;
    }
    if (sim_exe_c == nullptr || sim_exe_c[0] == '\0') {
        sim_exe_c = std::getenv("HEXAPOD_PHYSICS_SIM_EXE");
    }
    if (sim_exe_c == nullptr || sim_exe_c[0] == '\0') {
        std::cout << "skip test_gravity_feedforward_stand_quiescence (pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }
    const std::string sim_exe{sim_exe_c};

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int port = 28000 + (static_cast<int>(::getpid()) % 3000);

    const StandMetrics off = measureStandQuiescence(sim_exe, false, port, harness);
    const StandMetrics on = measureStandQuiescence(sim_exe, true, port + 17, harness);

    constexpr const char* kSuite = "gravity_feedforward_stand";
    constexpr const char* kCase = "stand_compare";
    const double quiet_primary =
        test_limits::getDouble(kSuite, kCase, "", "quiet_primary_score", 0.0025);
    const double improve_frac =
        test_limits::getDouble(kSuite, kCase, "", "min_primary_improve_frac", 0.12);

    const double primary_off = primaryScore(off);
    const double primary_on = primaryScore(on);
    const bool both_quiet = std::max(primary_off, primary_on) < quiet_primary;
    const bool improved =
        primary_off > 1e-7 && primary_on < primary_off * (1.0 - improve_frac);

    if (envFlag("HEXAPOD_FF_CSV")) {
        std::cerr << "ff,primary,rms_track,rms_vel,max_z\n";
        std::cerr << "0," << primary_off << ',' << off.rms_femur_tibia_tracking_rad << ',' << off.rms_joint_vel_radps
                  << ',' << off.max_abs_body_world_z_m << '\n';
        std::cerr << "1," << primary_on << ',' << on.rms_femur_tibia_tracking_rad << ',' << on.rms_joint_vel_radps
                  << ',' << on.max_abs_body_world_z_m << '\n';
    }

    if (!expect(both_quiet || improved,
                "feedforward-on should lower primary score vs off, or both runs stay under the quiet floor")) {
        std::cerr << "primary_off=" << primary_off << " primary_on=" << primary_on
                  << " rms_track_off=" << off.rms_femur_tibia_tracking_rad
                  << " rms_track_on=" << on.rms_femur_tibia_tracking_rad << " max_z_off=" << off.max_abs_body_world_z_m
                  << " max_z_on=" << on.max_abs_body_world_z_m << " vel_off=" << off.rms_joint_vel_radps
                  << " vel_on=" << on.rms_joint_vel_radps << '\n';
        return 1;
    }

    (void)emit_metrics_json;
    return 0;
#endif
}
