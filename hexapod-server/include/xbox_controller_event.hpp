#pragma once
#include <string>

struct XboxEvent {
    enum class Type {
        Button
    };

    Type type;
    std::string name;
    int value;
};
