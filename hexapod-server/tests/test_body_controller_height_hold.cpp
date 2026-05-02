#include "body_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEqual(const double lhs, const double rhs, const double eps = 1e-9) {
    return std::abs(lhs - rhs) <= eps;
}

bool testReducedSagTriggersFastUnwind() {
    constexpr double kCommandedBodyHeightM = 0.14;
    constexpr double kSlowPhaseMeasuredBodyHeightM = 0.12;  // 20 mm sag
    constexpr double kFastPhaseMeasuredBodyHeightM = 0.136; // 4 mm sag

    double integral_m = 0.0;
    for (int i = 0; i < 7; ++i) {
        integral_m = body_controller_detail::updateBodyHeightHoldIntegralM(
            integral_m,
            kCommandedBodyHeightM,
            true,
            kSlowPhaseMeasuredBodyHeightM);
    }
    if (!expect(integral_m > 0.020 && integral_m < 0.022,
                "slow sag phase should preload roughly 21 mm of integral hold")) {
        return false;
    }

    const double preloaded_integral_m = integral_m;
    integral_m = body_controller_detail::updateBodyHeightHoldIntegralM(
        integral_m,
        kCommandedBodyHeightM,
        true,
        kFastPhaseMeasuredBodyHeightM);
    if (!expect(integral_m < preloaded_integral_m,
                "reduced sag should unwind the retained integral instead of increasing it")) {
        return false;
    }

    for (int i = 0; i < 14; ++i) {
        integral_m = body_controller_detail::updateBodyHeightHoldIntegralM(
            integral_m,
            kCommandedBodyHeightM,
            true,
            kFastPhaseMeasuredBodyHeightM);
    }

    return expect(integral_m < 0.008,
                  "reduced-sag transition should collapse the stale integral within a few hundred milliseconds");
}

bool testOvershootStillUsesFastDecay() {
    constexpr double kCommandedBodyHeightM = 0.14;
    constexpr double kMeasuredAboveCommandM = 0.142;

    const double decayed_m = body_controller_detail::updateBodyHeightHoldIntegralM(
        0.020,
        kCommandedBodyHeightM,
        true,
        kMeasuredAboveCommandM);
    return expect(nearlyEqual(decayed_m, 0.020 * 0.93),
                  "body above command should continue using the fast overshoot decay branch");
}

bool testMissingMeasurementUsesSlowDecay() {
    const double decayed_m = body_controller_detail::updateBodyHeightHoldIntegralM(
        0.020,
        0.14,
        false,
        0.0);
    return expect(nearlyEqual(decayed_m, 0.020 * 0.99),
                  "missing body-height measurement should fall back to the slow decay path");
}

} // namespace

int main() {
    return testReducedSagTriggersFastUnwind() &&
                   testOvershootStillUsesFastDecay() &&
                   testMissingMeasurementUsesSlowDecay()
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
