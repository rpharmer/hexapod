#include "virtual_gamepad_controller.hpp"

#include <cmath>

namespace {

void initializeAxes(std::map<std::string, int>& rawAxis)
{
    rawAxis["LX"] = rawAxis["LY"] = 0;
    rawAxis["RX"] = rawAxis["RY"] = 0;
    rawAxis["LT"] = rawAxis["RT"] = 0;
}

} // namespace

VirtualGamepadController::VirtualGamepadController()
    : running(false),
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

bool VirtualGamepadController::start()
{
    running = true;
    return true;
}

void VirtualGamepadController::stop()
{
    running = false;
}

float VirtualGamepadController::getLeftX() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftX;
}

float VirtualGamepadController::getLeftY() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftY;
}

float VirtualGamepadController::getLeftMag() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftMag;
}

float VirtualGamepadController::getLeftAng() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftAng;
}

float VirtualGamepadController::getRightX() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightX;
}

float VirtualGamepadController::getRightY() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightY;
}

float VirtualGamepadController::getRightMag() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightMag;
}

float VirtualGamepadController::getRightAng() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightAng;
}

int VirtualGamepadController::getLeftTrigger() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftTrigger;
}

int VirtualGamepadController::getRightTrigger() const
{
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightTrigger;
}

void VirtualGamepadController::setRadialDeadzone(const std::string& stick, int dz)
{
    radialDZ[stick] = dz;
}

void VirtualGamepadController::updateStick(const std::string& stick)
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

void VirtualGamepadController::processLogicalEvent(const ControllerEvent& ev)
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

bool VirtualGamepadController::injectAxis(const std::string& axis, int value)
{
    if (!running.load()) {
        return false;
    }
    processLogicalEvent(ControllerEvent{ControllerEvent::Type::Axis, axis, value});
    return true;
}

bool VirtualGamepadController::injectButton(const std::string& button, int value)
{
    if (!running.load()) {
        return false;
    }
    processLogicalEvent(ControllerEvent{ControllerEvent::Type::Button, button, value});
    return true;
}
