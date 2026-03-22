#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <map>
#include <linux/input.h>
#include "event_queue.hpp"
#include "xbox_controller_event.hpp"

class XboxController {
public:
    XboxController(const std::string& devicePath);
    ~XboxController();

    bool start();
    void stop();

    EventQueue<XboxEvent>& getQueue() { return queue; }

    // Read current stick states
    int getLeftX()  const { return leftX; }
    int getLeftY()  const { return leftY; }
    float getLeftMag() const { return leftMag; }
    float getLeftAng() const { return leftAng; }

    int getRightX() const { return rightX; }
    int getRightY() const { return rightY; }
    float getRightMag() const { return rightMag; }
    float getRightAng() const { return rightAng; }

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

    // Stick state
    int leftX, leftY;
    float leftMag, leftAng;

    int rightX, rightY;
    float rightMag, rightAng;
};
