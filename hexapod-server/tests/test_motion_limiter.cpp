#include "motion_limiter.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

MotionIntent makeWalkIntent(double speed_mps, double yaw_rate_radps, double body_x_m) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.speed_mps = LinearRateMps{speed_mps};
    intent.heading_rad = AngleRad{0.0};
    intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, yaw_rate_radps};
    intent.body_pose_setpoint.body_trans_m = Vec3{body_x_m, 0.0, 0.20};
    return intent;
}

SafetyState safeState() {
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.active_fault = FaultCode::NONE;
    return safety;
}

bool startupStateSequenceUsesPreloadThenLocomoting() {
    MotionLimiter limiter;
    const SafetyState safety = safeState();

    MotionLimiterOutput out = limiter.update(makeWalkIntent(0.12, 0.0, 0.06), safety);
    if (!expect(out.state == MotionLimiterState::IDLE || out.state == MotionLimiterState::BODY_PRELOAD,
                "initial update should stay idle or enter preload")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    out = limiter.update(makeWalkIntent(0.12, 0.0, 0.06), safety);
    if (!expect(out.state == MotionLimiterState::BODY_PRELOAD,
                "start command should enter BODY_PRELOAD after hysteresis")) {
        return false;
    }
    if (!expect(out.intent.requested_mode == RobotMode::STAND,
                "BODY_PRELOAD should hold gait progression")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(420));
    out = limiter.update(makeWalkIntent(0.12, 0.0, 0.06), safety);

    return expect(out.state == MotionLimiterState::LOCOMOTING,
                  "BODY_PRELOAD should transition to LOCOMOTING by timeout/threshold") &&
           expect(out.intent.requested_mode == RobotMode::WALK,
                  "LOCOMOTING should allow walk mode") &&
           expect(out.intent.speed_mps.value > 0.0,
                  "LOCOMOTING should emit non-zero stride command");
}

bool stopSequenceUsesSettleThenIdle() {
    MotionLimiter limiter;
    const SafetyState safety = safeState();

    MotionLimiterOutput out{};
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
        out = limiter.update(makeWalkIntent(0.14, 0.20, 0.05), safety);
        if (out.state == MotionLimiterState::LOCOMOTING) {
            break;
        }
    }
    if (!expect(out.state == MotionLimiterState::LOCOMOTING,
                "test setup should reach LOCOMOTING before stop")) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    out = limiter.update(makeWalkIntent(0.0, 0.0, 0.00), safety);
    if (out.state != MotionLimiterState::BODY_SETTLE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out = limiter.update(makeWalkIntent(0.0, 0.0, 0.00), safety);
        if (!expect(out.state == MotionLimiterState::BODY_SETTLE,
                    "clear command should enter BODY_SETTLE from locomoting")) {
            return false;
        }
    }

    bool saw_walk_in_settle = false;
    bool saw_stand_in_settle = false;
    for (int i = 0; i < 8; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        out = limiter.update(makeWalkIntent(0.0, 0.0, 0.00), safety);
        saw_walk_in_settle = saw_walk_in_settle || out.intent.requested_mode == RobotMode::WALK;
        saw_stand_in_settle = saw_stand_in_settle || out.intent.requested_mode == RobotMode::STAND;
        if (out.state == MotionLimiterState::IDLE) {
            break;
        }
    }

    return expect(saw_walk_in_settle,
                  "BODY_SETTLE should keep locomotion active while cadence ramps down") &&
           expect(saw_stand_in_settle,
                  "BODY_SETTLE should switch to stand after gait halt") &&
           expect(out.state == MotionLimiterState::IDLE,
                  "BODY_SETTLE should complete into IDLE once body settles");
}

bool hysteresisSuppressesStateChatter() {
    MotionLimiter limiter;
    const SafetyState safety = safeState();

    MotionLimiterOutput out{};
    for (int i = 0; i < 6; ++i) {
        const double speed = (i % 2 == 0) ? 0.05 : 0.01;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        out = limiter.update(makeWalkIntent(speed, 0.0, 0.02), safety);
    }

    return expect(out.state != MotionLimiterState::LOCOMOTING,
                  "brief noisy command toggles should not chatter into locomoting");
}

} // namespace

int main() {
    const bool ok = startupStateSequenceUsesPreloadThenLocomoting() &&
                    stopSequenceUsesSettleThenIdle() &&
                    hysteresisSuppressesStateChatter();
    if (!ok) {
        return 1;
    }
    std::cout << "PASS\n";
    return 0;
}
