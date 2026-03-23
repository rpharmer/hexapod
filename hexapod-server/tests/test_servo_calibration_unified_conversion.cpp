#include "types.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool near(double lhs, double rhs, double eps = 1e-9) {
    return std::abs(lhs - rhs) <= eps;
}

}  // namespace

int main() {
    ServoCalibration calibration{};
    calibration.coxaOffset = AngleRad{0.25};
    calibration.femurOffset = AngleRad{-0.4};
    calibration.tibiaOffset = AngleRad{0.1};
    calibration.coxaSign = 4.0;   // normalized to +1
    calibration.femurSign = -3.0; // normalized to -1
    calibration.tibiaSign = std::numeric_limits<double>::quiet_NaN(); // normalized to +1

    LegState joint_raw{};
    joint_raw.joint_state[0].pos_rad = AngleRad{0.5};
    joint_raw.joint_state[1].pos_rad = AngleRad{-0.25};
    joint_raw.joint_state[2].pos_rad = AngleRad{1.1};

    const LegState servo_raw = calibration.toServoAngles(joint_raw);
    if (!expect(near(servo_raw.joint_state[0].pos_rad.value, 0.75),
                "coxa raw conversion should apply normalized sign and offset") ||
        !expect(near(servo_raw.joint_state[1].pos_rad.value, -0.15),
                "femur raw conversion should apply normalized sign and offset") ||
        !expect(near(servo_raw.joint_state[2].pos_rad.value, 1.2),
                "tibia raw conversion should treat non-finite sign as +1")) {
        return EXIT_FAILURE;
    }

    const LegState joint_raw_from_servo = calibration.toJointAngles(servo_raw);
    if (!expect(near(joint_raw_from_servo.joint_state[0].pos_rad.value, 0.5),
                "coxa raw decode should preserve existing conversion semantics") ||
        !expect(near(joint_raw_from_servo.joint_state[1].pos_rad.value, 0.55),
                "femur raw decode should preserve existing conversion semantics") ||
        !expect(near(joint_raw_from_servo.joint_state[2].pos_rad.value, 1.1),
                "tibia raw decode should preserve existing conversion semantics")) {
        return EXIT_FAILURE;
    }

    LegState joint_state{};
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        joint_state.joint_state[joint].pos_rad = joint_raw.joint_state[joint].pos_rad;
        joint_state.joint_state[joint].vel_radps = AngularRateRadPerSec{1.23};
    }

    const LegState servo_state = calibration.toServoAngles(joint_state);
    if (!expect(near(servo_state.joint_state[0].pos_rad.value,
                     servo_raw.joint_state[0].pos_rad.value),
                "raw/state coxa conversion paths should stay equivalent") ||
        !expect(near(servo_state.joint_state[1].pos_rad.value,
                     servo_raw.joint_state[1].pos_rad.value),
                "raw/state femur conversion paths should stay equivalent") ||
        !expect(near(servo_state.joint_state[2].pos_rad.value,
                     servo_raw.joint_state[2].pos_rad.value),
                "raw/state tibia conversion paths should stay equivalent")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
