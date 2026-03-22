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
    bool saw_a_press = false;
    bool saw_a_release = false;

    while (auto ev = controller.getQueue().pop()) {
        if (ev->type == ControllerEvent::Type::Axis) {
            ++axis_events;
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

    if (!expect(axis_events >= 6, "axis events should be queued for every axis inject") ||
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
