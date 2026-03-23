#pragma once

#include <string>

#include "evdev_gamepad_controller.hpp"

class XboxController : public EvdevGamepadController {
public:
    explicit XboxController(const std::string& devicePath)
        : EvdevGamepadController(devicePath, makeXboxGamepadMapping()) {}
};
