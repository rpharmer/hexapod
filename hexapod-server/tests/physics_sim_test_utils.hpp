#pragma once

#include "control_config.hpp"
#include "hexapod-server.hpp"
#include "physics_sim_bridge.hpp"
#include "toml_parser.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace physics_sim_test_utils {

// Opt-in experiment, never a production default. Keep reaction forces off and
// all existing angle/gyro/quality/speed limits. No simulator oracle is queried.
inline void applySelfWeightOnlyScreen(control_config::ControlConfig& config) {
    const char* value = std::getenv("HEXAPOD_WALK_TEST_SELF_WEIGHT");
    if (value == nullptr || std::string{value} != "1") return;
    auto& ff = config.gravity_feedforward;
    ff.enabled = true;
    ff.mode = control_config::GravityFeedforwardMode::Bounded;
    ff.include_foot_reaction = false;
    ff.include_self_weight = true;
    ff.scale_coxa = 0.0;
    ff.scale_femur = ff.scale_tibia = 1.0;
    ff.stiffness_gain_scale = 1.0;
    ff.delta_lpf_tau_s = .08;
}

struct HarnessSettings {
    std::filesystem::path config_path{};
    ParsedToml parsed{};
    control_config::ControlConfig control_cfg{};
    int bus_loop_period_us{0};
    int physics_solver_iterations{0};
};

/** When `prefer_test_harness_config` is true, prefer `config.physics-sim-test-harness.txt` (CTest walk/speed profile). */
inline std::filesystem::path resolvePhysicsSimConfigPath(bool prefer_test_harness_config = false) {
    std::vector<std::filesystem::path> candidates{};
    if (prefer_test_harness_config) {
        candidates.push_back("config.physics-sim-test-harness.txt");
        candidates.push_back("../config.physics-sim-test-harness.txt");
        candidates.push_back("hexapod-server/config.physics-sim-test-harness.txt");
    }
    candidates.push_back("config.physics-sim-wsl.txt");
    candidates.push_back("../config.physics-sim-wsl.txt");
    candidates.push_back("hexapod-server/config.physics-sim-wsl.txt");
    for (const auto& candidate : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(candidate, ec)) {
            return std::filesystem::weakly_canonical(candidate, ec);
        }
    }
    return {};
}

inline HarnessSettings loadHarnessSettings(bool prefer_test_harness_config = false) {
    HarnessSettings settings{};
    settings.config_path = resolvePhysicsSimConfigPath(prefer_test_harness_config);
    if (settings.config_path.empty()) {
        throw std::runtime_error(
            "unable to locate config.physics-sim-test-harness.txt (when requested) or config.physics-sim-wsl.txt");
    }

    TomlParser parser{};
    if (!parser.parse(settings.config_path.string(), settings.parsed)) {
        throw std::runtime_error("failed to parse " + settings.config_path.string());
    }

    settings.control_cfg = control_config::fromParsedToml(settings.parsed);
    settings.bus_loop_period_us =
        static_cast<int>(settings.control_cfg.loop_timing.bus_loop_period.count());
    settings.physics_solver_iterations = settings.parsed.physicsSimSolverIterations;
    return settings;
}

/** Production WSL physics-sim solver: pinocchio-proximal at cap 24. Does not change the
 *  iterations-only PhysicsSimBridge ctor, which remains LegacyPgs. */
inline PhysicsSimSolverSettings productionProximalSolverSettings() {
    PhysicsSimSolverSettings settings{};
    settings.mode = physics_sim::PhysicsSolverMode::PinocchioProximal;
    settings.iterations = 24;
    return settings;
}

inline std::size_t scaledLegacyStepCount(const std::size_t legacy_steps,
                                         const int actual_period_us,
                                         const int legacy_period_us = 20000) {
    if (actual_period_us <= 0 || legacy_period_us <= 0) {
        return legacy_steps;
    }

    const double scale =
        static_cast<double>(legacy_period_us) / static_cast<double>(actual_period_us);
    return std::max<std::size_t>(
        1,
        static_cast<std::size_t>(std::llround(static_cast<double>(legacy_steps) * scale)));
}

inline double controlLoopPeriodSeconds(const control_config::ControlConfig& cfg) {
    return std::max(
        1.0e-6,
        static_cast<double>(cfg.loop_timing.control_loop_period.count()) * 1.0e-6);
}

inline void quietChildProcessStdIo() {
#if defined(__linux__)
    const char* keep_stdio = std::getenv("HEXAPOD_TEST_KEEP_SIM_STDIO");
    if (keep_stdio != nullptr && keep_stdio[0] != '\0' && keep_stdio[0] != '0') {
        return;
    }

    const int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull < 0) {
        return;
    }
    (void)::dup2(devnull, STDOUT_FILENO);
    (void)::dup2(devnull, STDERR_FILENO);
    if (devnull > STDERR_FILENO) {
        ::close(devnull);
    }
#endif
}

} // namespace physics_sim_test_utils
