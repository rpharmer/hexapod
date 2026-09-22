#include "locomotion_metrics.hpp"
#include <cmath>
#include <iostream>

int main() {
    using namespace locomotion_test;
    std::vector<MotionSample> samples(4);
    for (auto& s : samples) s.horizontal_speed_mps = .2;
    samples[2].status.active_fault = FaultCode::TIP_OVER;
    samples[2].horizontal_speed_mps = 100;
    samples[3].horizontal_speed_mps = 100; // even later recovery must not add travel
    bool ok = std::abs(pathBeforeFirstFaultM(samples, .005) - .002) < 1e-12;
    samples[0].status.active_fault = FaultCode::TIP_OVER;
    ok &= pathBeforeFirstFaultM(samples, .005) == 0;
    samples.clear();
    ok &= pathBeforeFirstFaultM(samples, .005) == 0;
    if (!ok) std::cerr << "FAIL: post-fault motion must not satisfy a pre-fault path gate\n";
    return ok ? 0 : 1;
}
