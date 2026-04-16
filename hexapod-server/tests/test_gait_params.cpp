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

    const UnifiedGaitDescription crawl_ref =
        buildTargetUnifiedGait(GaitType::CRAWL, 0.0, 0.0, 0.0, gait, 0.0, 0.0);
    const UnifiedGaitDescription wave_ref =
        buildTargetUnifiedGait(GaitType::WAVE, 0.6, 0.0, 0.0, gait, 0.0, 0.0);

    const UnifiedGaitDescription adaptive_low =
        buildAdaptiveWaveCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription adaptive_high =
        buildAdaptiveWaveCrawlGait(0.6, 0.0, 0.0, 0.0, 0.0, gait);

    if (!nearlyEq(adaptive_low.duty_factor, crawl_ref.duty_factor)) {
        std::cerr << "FAIL: low-speed adaptive WAVE should match CRAWL duty\n";
        return EXIT_FAILURE;
    }
    if (!nearlyEq(adaptive_high.duty_factor, wave_ref.duty_factor)) {
        std::cerr << "FAIL: high-speed adaptive WAVE should match WAVE-only duty\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_high.duty_factor < adaptive_low.duty_factor - 1e-4)) {
        std::cerr << "FAIL: blended WAVE should reduce duty vs crawl at high command\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
