#pragma once

#include <atomic>
#include <linux/input.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "control_device.hpp"

struct GamepadMapping {
    std::map<int, std::string> button_map;
    std::map<int, std::string> axis_map;
};

GamepadMapping makeXboxGamepadMapping();
GamepadMapping makeGenericGamepadMapping();

class EvdevGamepadController : public IControlDevice {
public:
    EvdevGamepadController(std::string devicePath, GamepadMapping mapping);
    ~EvdevGamepadController() override;

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

protected:
    void processLogicalEvent(const ControllerEvent& ev);

private:
    void eventLoop();
    void processInputEvent(const input_event& ev);
    void updateStick(const std::string& stick);

    int fd;
    std::string device;
    std::thread worker;
    std::atomic<bool> running;

    EventQueue<ControllerEvent> queue;

    std::map<int, std::string> buttonMap;
    std::map<int, std::string> axisMap;

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
