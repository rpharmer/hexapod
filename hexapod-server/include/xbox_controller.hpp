#pragma once
#include <atomic>
#include <linux/input.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "event_queue.hpp"
#include "xbox_controller_event.hpp"

class XboxController {
public:
    explicit XboxController(const std::string& devicePath);
    ~XboxController();

    bool start();
    void stop();

    EventQueue<XboxEvent>& getQueue() { return queue; }

    // Read current stick states
    int getLeftX() const;
    int getLeftY() const;
    float getLeftMag() const;
    float getLeftAng() const;

    int getRightX() const;
    int getRightY() const;
    float getRightMag() const;
    float getRightAng() const;

    int getLeftTrigger() const;
    int getRightTrigger() const;

    void setRadialDeadzone(const std::string& stick, int dz);

private:
    void eventLoop();
    void processEvent(const input_event& ev);

    void updateStick(const std::string& stick);

    int fd;
    std::string device;
    std::thread worker;
    std::atomic<bool> running;

    EventQueue<XboxEvent> queue;

    std::map<int, std::string> buttonMap;
    std::map<int, std::string> axisMap;

    std::map<std::string, int> rawAxis;
    std::map<std::string, int> radialDZ;

    mutable std::mutex stickStateMutex;

    // Stick state
    int leftX;
    int leftY;
    float leftMag;
    float leftAng;

    int rightX;
    int rightY;
    float rightMag;
    float rightAng;

    int leftTrigger;
    int rightTrigger;
};
