#include <cmath>
#include <algorithm>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::tests;

struct AccelSample {
    float peak_impulse = 0.0f;
    float peak_norm_utilization = 0.0f;
    bool respected_limit = true;
};

AccelSample runSample(float max_servo_torque) {
    World world({0.0f, 0.0f, 0.0f});

    const std::uint32_t base_id = world.CreateBody(MakeStaticBase());
    constexpr float kLinkLength = 1.0f;
    const std::uint32_t link_id = world.CreateBody(MakeArmLink({0.5f * kLinkLength, 0.0f, 0.0f}, kLinkLength, 1.2f));

    const std::uint32_t servo_id = world.CreateServoJoint(
        base_id,
        link_id,
        {0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        1.2f,
        max_servo_torque,
        35.0f,
        3.0f);

    constexpr float kDt = 1.0f / 240.0f;
    AccelSample out{};
    for (int step = 0; step < 180; ++step) {
        world.AddTorque(link_id, {0.0f, 0.0f, -40.0f});
        world.Step(kDt, 48);
        const ServoJoint& servo = world.GetServoJoint(servo_id);
        const float impulse = std::abs(servo.servoImpulseSum);
        out.peak_impulse = std::max(out.peak_impulse, impulse);
        out.peak_norm_utilization = std::max(out.peak_norm_utilization, impulse / max_servo_torque);
        if (impulse > max_servo_torque + 1.0e-3f) {
            out.respected_limit = false;
        }
    }
    return out;
}

int runCase() {
    const AccelSample low = runSample(1.0f);
    const AccelSample high = runSample(5.0f);
    if (!low.respected_limit || !high.respected_limit) {
        std::cerr << "torque_limit violation low_ok=" << low.respected_limit
                  << " high_ok=" << high.respected_limit << "\n";
        return 1;
    }
    if (low.peak_norm_utilization < 0.70f) {
        std::cerr << "torque_limit low-cap utilization too small=" << low.peak_norm_utilization << "\n";
        return 1;
    }
    return 0;
}

} // namespace

int main() {
    return runCase();
}
