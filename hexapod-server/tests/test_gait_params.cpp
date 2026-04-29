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
    if (!(wave_ref.swing_height_m >= kWaveSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: fast WAVE should retain its swing floor\n";
        return EXIT_FAILURE;
    }
    if (!(buildAdaptiveRippleCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait).swing_height_m >=
          kRippleSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive RIPPLE should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
