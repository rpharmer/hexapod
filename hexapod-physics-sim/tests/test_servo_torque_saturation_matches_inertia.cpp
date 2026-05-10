#include <cmath>
#include <algorithm>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct AccelSample {
    Real peak_impulse = 0.0;
    Real peak_norm_utilization = 0.0;
    bool respected_limit = true;
};

AccelSample runSample(Real max_servo_torque) {
    World world({0.0, 0.0, 0.0});

    const std::uint32_t base_id = world.CreateBody(MakeStaticBase());
    constexpr Real kLinkLength = 1.0;
    const std::uint32_t link_id = world.CreateBody(MakeArmLink({0.5 * kLinkLength, 0.0, 0.0}, kLinkLength, 1.2));

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        1.2,
        max_servo_torque,
        35.0,
        3.0);

    constexpr Real kDt = 1.0 / 240.0;
    AccelSample out{};
    for (int step = 0; step < 180; ++step) {
        world.AddTorque(link_id, {0.0, 0.0, -40.0});
        world.Step(kDt, 48);
        const ServoJoint& servo = world.GetServoJoint(servo_id);
        const Real impulse = std::abs(servo.servoImpulseSum);
        out.peak_impulse = std::max(out.peak_impulse, impulse);
        out.peak_norm_utilization = std::max(out.peak_norm_utilization, impulse / max_servo_torque);
        if (impulse > max_servo_torque + 1.0e-3) {
            out.respected_limit = false;
        }
    }
    return out;
}

int runCase() {
    const AccelSample low = runSample(1.0);
    const AccelSample high = runSample(5.0);
    if (!low.respected_limit || !high.respected_limit) {
        std::cerr << "torque_limit violation low_ok=" << low.respected_limit
                  << " high_ok=" << high.respected_limit << "\n";
        return 1;
    }
    if (low.peak_norm_utilization < 0.70) {
        std::cerr << "torque_limit low-cap utilization too small=" << low.peak_norm_utilization << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
