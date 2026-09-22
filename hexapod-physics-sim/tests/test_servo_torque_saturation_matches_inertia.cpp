#include <cmath>
#include <algorithm>
#include <cstdint>
#include <iostream>

#include "solver_validation_helpers.hpp"
#include "minphys3d/joints/servo_motor.hpp"

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
        const Real impulse_limit = max_servo_torque * kDt;
        out.peak_norm_utilization = std::max(out.peak_norm_utilization, impulse / impulse_limit);
        if (impulse > impulse_limit + 1.0e-6) {
            out.respected_limit = false;
        }
    }
    return out;
}

int runCase() {
    // A PGS row must converge to one motor envelope, not alternate between
    // full drive and no drive as its own impulse changes the sampled speed.
    for (Real sign : {-1.0, 1.0}) {
        constexpr Real stallImpulse = 100.0 / 240.0;
        constexpr Real inverseInertia = 100.0;
        constexpr Real noLoadSpeed = 7.48;
        const Real first = ClampServoMotorImpulse(sign * 10.0, 0, 0,
                                                  inverseInertia, stallImpulse, noLoadSpeed);
        const Real speed = inverseInertia * first;
        const Real repeated = ClampServoMotorImpulse(sign * 10.0, first, speed,
                                                     inverseInertia, stallImpulse, noLoadSpeed);
        const Real available = stallImpulse * std::max(0.0, 1.0 - std::abs(speed) / noLoadSpeed);
        if (std::abs(first - repeated) > 1e-12 || std::abs(first) > available + 1e-12) {
            std::cerr << "motor envelope depends on its own iteration: first=" << first
                      << " repeated=" << repeated << " speed=" << speed << '\n';
            return 1;
        }
    }
    for (const Real dt : {1.0 / 120.0, 1.0 / 240.0, 1.0 / 480.0}) {
        const Real limit = 1.471 * dt;
        // Above no-load speed, assisting torque is zero, while opposing
        // torque retains the same stall-torque budget at every cadence.
        if (ClampServoMotorImpulse(10, 0, 10, 1, limit, 7.48) != 0
            || std::abs(ClampServoMotorImpulse(-10, 0, 10, 1, limit, 7.48) + limit) > 1e-12
            || std::abs(ClampServoMotorImpulse(10, 0, 10, 1, limit, 0) - limit) > 1e-12) {
            std::cerr << "motor envelope changed braking or torque-times-dt budget\n";
            return 1;
        }
    }
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
