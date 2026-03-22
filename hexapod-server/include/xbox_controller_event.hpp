#pragma once
#include <string>

struct XboxEvent {
    enum class Type {
        Button,
        Axis
    };

    Type type{Type::Button};
    std::string name;
    int value{0};
};
