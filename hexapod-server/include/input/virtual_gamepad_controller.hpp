#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>

#include "control_device.hpp"

class VirtualGamepadController : public IControlDevice {
public:
    VirtualGamepadController();

    bool start() override;
    void stop() override;

    EventQueue<ControllerEvent>& getQueue() override { return queue; }

    float getLeftX() const override;
    float getLeftY() const override;
    float getLeftMag() const override;
    float getLeftAng() const override;

    float getRightX() const override;
    float getRightY() const override;
    float getRightMag() const override;
    float getRightAng() const override;

    int getLeftTrigger() const override;
    int getRightTrigger() const override;

    void setRadialDeadzone(const std::string& stick, int dz) override;

    bool injectAxis(const std::string& axis, int value);
    bool injectButton(const std::string& button, int value);

private:
    void processLogicalEvent(const ControllerEvent& ev);
    void updateStick(const std::string& stick);

    std::atomic<bool> running;
    EventQueue<ControllerEvent> queue;

    std::map<std::string, int> rawAxis;
    std::map<std::string, int> radialDZ;

    mutable std::mutex stickStateMutex;

    float leftX;
    float leftY;
    float leftMag;
    float leftAng;

    float rightX;
    float rightY;
    float rightMag;
    float rightAng;

    int leftTrigger;
    int rightTrigger;
};
