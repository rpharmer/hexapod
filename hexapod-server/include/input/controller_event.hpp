#pragma once

#include <string>

struct ControllerEvent {
    enum class Type {
        Button,
        Axis
    };

    Type type{Type::Button};
    std::string name;
    int value{0};
};

using XboxEvent = ControllerEvent;
