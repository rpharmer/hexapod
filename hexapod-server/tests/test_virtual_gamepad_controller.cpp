#include "virtual_gamepad_controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool expectNear(float actual, float expected, float tolerance, const char* message)
{
    return expect(std::fabs(actual - expected) <= tolerance, message);
}

} // namespace

int main()
{
    VirtualGamepadController controller;

    if (!expect(!controller.injectAxis("LX", 20000), "injects should fail before start") ||
        !expect(controller.start(), "virtual controller should start")) {
        return EXIT_FAILURE;
    }

    controller.setRadialDeadzone("L", 8000);
    controller.setRadialDeadzone("R", 8000);
    if (!expect(controller.injectAxis("LX", 4000), "axis inject should succeed after start") ||
        !expect(controller.injectAxis("LY", 0), "axis inject should succeed after start")) {
        return EXIT_FAILURE;
    }

    if (!expectNear(controller.getLeftMag(), 0.0f, 1e-5f, "left stick should be suppressed by deadzone") ||
        !expectNear(controller.getLeftX(), 0.0f, 1e-5f, "left X should be zero inside deadzone") ||
        !expectNear(controller.getLeftY(), 0.0f, 1e-5f, "left Y should be zero inside deadzone")) {
        return EXIT_FAILURE;
    }

    if (!expect(controller.injectAxis("LX", 32767), "max LX inject should succeed") ||
        !expect(controller.injectAxis("LY", 0), "LY inject should succeed")) {
        return EXIT_FAILURE;
    }

    if (!expectNear(controller.getLeftMag(), 1.0f, 1e-5f, "left stick magnitude should normalize to 1.0") ||
        !expectNear(controller.getLeftX(), 1.0f, 1e-5f, "left X should normalize to 1.0") ||
        !expectNear(controller.getLeftY(), 0.0f, 1e-5f, "left Y should normalize to 0") ||
        !expectNear(controller.getLeftAng(), 0.0f, 1e-5f, "left angle should point along +X")) {
        return EXIT_FAILURE;
    }

    if (!expect(controller.injectAxis("RX", 32767), "max RX inject should succeed") ||
        !expect(controller.injectAxis("RY", 0), "RY inject should succeed")) {
        return EXIT_FAILURE;
    }

    if (!expectNear(controller.getRightMag(), 1.0f, 1e-5f, "right stick magnitude should normalize to 1.0") ||
        !expectNear(controller.getRightX(), 1.0f, 1e-5f, "right X should normalize to 1.0") ||
        !expectNear(controller.getRightY(), 0.0f, 1e-5f, "right Y should normalize to 0") ||
        !expectNear(controller.getRightAng(), 0.0f, 1e-5f, "right angle should point along +X")) {
        return EXIT_FAILURE;
    }

    if (!expect(controller.injectAxis("LX", 0), "LX boundary inject should succeed") ||
        !expect(controller.injectAxis("LY", 8000), "LY boundary inject should succeed")) {
        return EXIT_FAILURE;
    }

    if (!expectNear(controller.getLeftMag(), 0.0f, 1e-5f, "left magnitude should be 0 at deadzone boundary") ||
        !expectNear(controller.getLeftX(), 0.0f, 1e-5f, "left X should be 0 at deadzone boundary") ||
        !expectNear(controller.getLeftY(), 0.0f, 1e-5f, "left Y should be 0 at deadzone boundary") ||
        !expectNear(controller.getLeftAng(), std::atan2(1.0f, 0.0f), 1e-5f,
                    "left angle should still be computed at deadzone boundary")) {
        return EXIT_FAILURE;
    }

    const float right_mag_before_unknown = controller.getRightMag();
    const float right_x_before_unknown = controller.getRightX();
    const float right_y_before_unknown = controller.getRightY();
    const float right_ang_before_unknown = controller.getRightAng();

    if (!expect(controller.injectAxis("UNKNOWN", 1234), "unknown axis inject should still succeed while running") ||
        !expectNear(controller.getRightMag(), right_mag_before_unknown, 1e-5f,
                    "unknown axis should not alter right magnitude") ||
        !expectNear(controller.getRightX(), right_x_before_unknown, 1e-5f,
                    "unknown axis should not alter right X") ||
        !expectNear(controller.getRightY(), right_y_before_unknown, 1e-5f,
                    "unknown axis should not alter right Y") ||
        !expectNear(controller.getRightAng(), right_ang_before_unknown, 1e-5f,
                    "unknown axis should not alter right angle")) {
        return EXIT_FAILURE;
    }

    if (!expect(controller.injectAxis("LT", 250), "LT inject should succeed") ||
        !expect(controller.injectAxis("RT", 900), "RT inject should succeed") ||
        !expect(controller.getLeftTrigger() == 250, "left trigger should update") ||
        !expect(controller.getRightTrigger() == 900, "right trigger should update")) {
        return EXIT_FAILURE;
    }

    if (!expect(controller.injectButton("A", 1), "button press inject should succeed") ||
        !expect(controller.injectButton("A", 0), "button release inject should succeed")) {
        return EXIT_FAILURE;
    }

    int axis_events = 0;
    int button_events = 0;
    int unknown_axis_events = 0;
    bool saw_a_press = false;
    bool saw_a_release = false;

    while (auto ev = controller.getQueue().pop()) {
        if (ev->type == ControllerEvent::Type::Axis) {
            ++axis_events;
            if (ev->name == "UNKNOWN" && ev->value == 1234) {
                ++unknown_axis_events;
            }
        } else if (ev->type == ControllerEvent::Type::Button && ev->name == "A") {
            ++button_events;
            if (ev->value == 1) {
                saw_a_press = true;
            }
            if (ev->value == 0) {
                saw_a_release = true;
            }
        }
    }

    if (!expect(axis_events >= 11, "axis events should be queued for every axis inject") ||
        !expect(unknown_axis_events == 1, "unknown axis event should be queued once") ||
        !expect(button_events == 2, "button events should include press and release") ||
        !expect(saw_a_press, "button press should be observed") ||
        !expect(saw_a_release, "button release should be observed")) {
        return EXIT_FAILURE;
    }

    controller.stop();
    if (!expect(!controller.injectButton("B", 1), "injects should fail after stop")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
