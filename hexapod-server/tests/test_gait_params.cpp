#include "control_config.hpp"
#include "gait_params.hpp"
#include "types.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

} // namespace

int main() {
    control_config::GaitConfig gait{};
    constexpr double kMinSwingHeightM = 0.014;
    constexpr double kTripodSwingFloorM = 0.021;
    constexpr double kRippleSwingFloorM = 0.019;
    constexpr double kWaveSwingFloorM = 0.018;

    const UnifiedGaitDescription crawl_ref =
        buildTargetUnifiedGait(GaitType::CRAWL, 0.0, 0.0, 0.0, gait, 0.0, 0.0);
    const UnifiedGaitDescription wave_ref =
        buildTargetUnifiedGait(GaitType::WAVE, 0.6, 0.0, 0.0, gait, 0.0, 0.0);
    const UnifiedGaitDescription tripod_low =
        buildAdaptiveTripodCrawlGait(0.06, 0.0, 0.0, 0.0, 0.0, gait);

    const UnifiedGaitDescription adaptive_low =
        buildAdaptiveWaveCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription adaptive_high =
        buildAdaptiveWaveCrawlGait(0.6, 0.0, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription turn_like =
        buildAdaptiveTripodCrawlGait(0.0, 0.0, 0.45, 0.0, 0.0, gait);

    if (!nearlyEq(adaptive_low.duty_factor, wave_ref.duty_factor)) {
        std::cerr << "FAIL: low-speed adaptive WAVE should match WAVE duty\n";
        return EXIT_FAILURE;
    }
    if (!nearlyEq(adaptive_high.duty_factor, wave_ref.duty_factor)) {
        std::cerr << "FAIL: high-speed adaptive WAVE should match WAVE-only duty\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_high.step_length_m > adaptive_low.step_length_m)) {
        std::cerr << "FAIL: higher-speed adaptive WAVE should increase step length\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_high.swing_height_m >= adaptive_low.swing_height_m)) {
        std::cerr << "FAIL: higher-speed adaptive WAVE should not reduce swing clearance\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_low.swing_height_m >= kMinSwingHeightM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive WAVE should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(tripod_low.swing_height_m >= kTripodSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive TRIPOD should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(crawl_ref.swing_height_m >= 0.015 - 1e-9)) {
        std::cerr << "FAIL: crawl nominal swing height should keep a modest clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(turn_like.swing_height_m >= 0.018 - 1e-9)) {
        std::cerr << "FAIL: yaw-heavy low-speed gait should lift higher to avoid turn drag\n";
        return EXIT_FAILURE;
    }
    if (!(turn_like.swing_height_m > adaptive_low.swing_height_m)) {
        std::cerr << "FAIL: yaw-heavy low-speed gait should gain more lift than still low-speed motion\n";
        return EXIT_FAILURE;
    }

    // Lateral (crabbing) walk should get more swing clearance than straight forward walking at
    // the same total planar speed. vy=0.06 m/s → lat_frac=1.0 → full oblique boost applies;
    // vx=0.06 m/s → lat_frac=0 → only the base half-boost applies.
    const UnifiedGaitDescription lateral_like =
        buildAdaptiveTripodCrawlGait(0.0, 0.06, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription fwd_same_speed =
        buildAdaptiveTripodCrawlGait(0.06, 0.0, 0.0, 0.0, 0.0, gait);
    if (!(lateral_like.swing_height_m > fwd_same_speed.swing_height_m)) {
        std::cerr << "FAIL: lateral low-speed gait should have higher swing floor than straight forward at same speed\n";
        return EXIT_FAILURE;
    }
    if (!(wave_ref.swing_height_m >= kWaveSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: fast WAVE should retain its swing floor\n";
        return EXIT_FAILURE;
    }
    if (!(buildAdaptiveRippleCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait).swing_height_m >=
          kRippleSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive RIPPLE should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }

    // Scenario 05 slow-phase swing heights: assert every phase's floor sits above the physics
    // foot sphere radius (0.018 m) so the sphere bottom always clears the ground at peak swing.
    // Computed against nominal_planar_speed_mps = 0.32 m/s (default GaitConfig).
    constexpr double kPhysicsFootRadiusM = 0.018;  // kHexapodFootRadiusM in physics_sim_protocol.hpp

    struct Scenario05Case {
        GaitType gait;
        double vx_mps;
        double yaw_rate_radps;
        const char* label;
    };
    const Scenario05Case scenario05_phases[] = {
        {GaitType::TRIPOD, 0.06, 0.0, "scenario05_tripod_0.06mps"},
        {GaitType::RIPPLE, 0.07, 0.0, "scenario05_ripple_0.07mps"},
        {GaitType::WAVE,   0.05, 0.0, "scenario05_wave_0.05mps"},
        {GaitType::TRIPOD, 0.05, 0.0, "scenario05_tripod_0.05mps"},
        {GaitType::RIPPLE, 0.04, 0.0, "scenario05_ripple_0.04mps"},
    };
    for (const auto& c : scenario05_phases) {
        const UnifiedGaitDescription desc =
            buildTargetUnifiedGait(c.gait, c.vx_mps, 0.0, c.yaw_rate_radps, gait, 0.0, 0.0);
        std::cout << c.label << " swing_height_m=" << desc.swing_height_m << '\n';
        if (!(desc.swing_height_m >= kPhysicsFootRadiusM - 1e-9)) {
            std::cerr << "FAIL: " << c.label << " swing_height_m=" << desc.swing_height_m
                      << " must be >= physics foot radius " << kPhysicsFootRadiusM << '\n';
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
