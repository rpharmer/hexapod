#include "evdev_gamepad_controller.hpp"

#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

namespace {

void initializeAxes(std::map<std::string, int>& rawAxis)
{
    rawAxis["LX"] = rawAxis["LY"] = 0;
    rawAxis["RX"] = rawAxis["RY"] = 0;
    rawAxis["LT"] = rawAxis["RT"] = 0;
}

} // namespace

GamepadMapping makeXboxGamepadMapping()
{
    return GamepadMapping{
        {
            {304, "A"}, {305, "B"}, {307, "X"}, {308, "Y"}, {310, "LB"}, {311, "RB"},
            {314, "Back"}, {315, "Start"}, {316, "Guide"}, {317, "LStick"}, {318, "RStick"},
        },
        {
            {0, "LX"}, {1, "LY"}, {2, "LT"}, {5, "RT"}, {3, "RX"}, {4, "RY"}, {16, "DPadX"}, {17, "DPadY"},
        },
    };
}

GamepadMapping makeGenericGamepadMapping()
{
    return makeXboxGamepadMapping();
}

EvdevGamepadController::EvdevGamepadController(std::string devicePath, GamepadMapping mapping)
    : fd(-1),
      device(std::move(devicePath)),
      running(false),
      buttonMap(std::move(mapping.button_map)),
      axisMap(std::move(mapping.axis_map)),
      leftX(0.0f),
      leftY(0.0f),
      leftMag(0.0f),
      leftAng(0.0f),
      rightX(0.0f),
      rightY(0.0f),
      rightMag(0.0f),
      rightAng(0.0f),
      leftTrigger(0),
      rightTrigger(0)
{
    radialDZ["L"] = 8000;
    radialDZ["R"] = 8000;
    initializeAxes(rawAxis);
}

EvdevGamepadController::~EvdevGamepadController()
{
    stop();
}

float EvdevGamepadController::getLeftX() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftX;
}

float EvdevGamepadController::getLeftY() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftY;
}

float EvdevGamepadController::getLeftMag() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftMag;
}

float EvdevGamepadController::getLeftAng() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftAng;
}

float EvdevGamepadController::getRightX() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightX;
}

float EvdevGamepadController::getRightY() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightY;
}

float EvdevGamepadController::getRightMag() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightMag;
}

float EvdevGamepadController::getRightAng() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightAng;
}

int EvdevGamepadController::getLeftTrigger() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftTrigger;
}

int EvdevGamepadController::getRightTrigger() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightTrigger;
}

void EvdevGamepadController::setRadialDeadzone(const std::string& stick, int dz)
{
    radialDZ[stick] = dz;
}

bool EvdevGamepadController::start()
{
    fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << "\n";
        return false;
    }

    running = true;
    worker = std::thread(&EvdevGamepadController::eventLoop, this);
    return true;
}

void EvdevGamepadController::stop()
{
    if (running) {
        running = false;
        if (worker.joinable()) {
            worker.join();
        }
    }

    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

void EvdevGamepadController::eventLoop()
{
    input_event ev;

    while (running) {
        const ssize_t n = read(fd, &ev, sizeof(ev));

        if (n == static_cast<ssize_t>(sizeof(ev))) {
            processInputEvent(ev);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void EvdevGamepadController::updateStick(const std::string& stick)
{
    const std::string ax = (stick == "L") ? "LX" : "RX";
    const std::string ay = (stick == "L") ? "LY" : "RY";

    const int x = rawAxis[ax];
    const int y = rawAxis[ay];

    const double x_d = static_cast<double>(x);
    const double y_d = static_cast<double>(y);
    const float r = static_cast<float>(std::sqrt((x_d * x_d) + (y_d * y_d)));
    const float dz = static_cast<float>(radialDZ[stick]);

    std::lock_guard<std::mutex> lock(stickStateMutex);

    if (r < dz) {
        if (stick == "L") {
            leftX = leftY = 0.0f;
            leftMag = 0.0f;
            leftAng = 0.0f;
        } else {
            rightX = rightY = 0.0f;
            rightMag = 0.0f;
            rightAng = 0.0f;
        }
        return;
    }

    float rNorm = (r - dz) / (32767.0f - dz);
    if (rNorm < 0.0f) {
        rNorm = 0.0f;
    }
    if (rNorm > 1.0f) {
        rNorm = 1.0f;
    }

    const float scale = rNorm / r;

    const float fx = static_cast<float>(x) * scale;
    const float fy = static_cast<float>(y) * scale;

    const float ang = std::atan2(static_cast<float>(y), static_cast<float>(x));

    if (stick == "L") {
        leftX = fx;
        leftY = fy;
        leftMag = rNorm;
        leftAng = ang;
    } else {
        rightX = fx;
        rightY = fy;
        rightMag = rNorm;
        rightAng = ang;
    }
}

void EvdevGamepadController::processLogicalEvent(const ControllerEvent& ev)
{
    if (ev.type == ControllerEvent::Type::Axis) {
        rawAxis[ev.name] = ev.value;

        if (ev.name == "LX" || ev.name == "LY") {
            updateStick("L");
        }

        if (ev.name == "RX" || ev.name == "RY") {
            updateStick("R");
        }

        if (ev.name == "LT" || ev.name == "RT") {
            std::lock_guard<std::mutex> lock(stickStateMutex);
            leftTrigger = rawAxis["LT"];
            rightTrigger = rawAxis["RT"];
        }
    }

    queue.push(ev);
}

void EvdevGamepadController::processInputEvent(const input_event& ev)
{
    if (ev.type == EV_KEY && buttonMap.count(ev.code)) {
        processLogicalEvent(ControllerEvent{ControllerEvent::Type::Button, buttonMap[ev.code], ev.value});
        return;
    }

    if (ev.type == EV_ABS && axisMap.count(ev.code)) {
        processLogicalEvent(ControllerEvent{ControllerEvent::Type::Axis, axisMap[ev.code], ev.value});
    }
}
