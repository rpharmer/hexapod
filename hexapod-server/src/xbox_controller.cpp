#include "xbox_controller.hpp"

#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

XboxController::XboxController(const std::string& devicePath)
    : fd(-1),
      device(devicePath),
      running(false),
      leftX(0.0f),
      leftY(0.0f),
      leftMag(0),
      leftAng(0),
      rightX(0.0f),
      rightY(0.0f),
      rightMag(0),
      rightAng(0),
      leftTrigger(0),
      rightTrigger(0) {
    buttonMap = {
        {304, "A"},      {305, "B"},       {307, "X"},       {308, "Y"},
        {310, "LB"},     {311, "RB"},      {314, "Back"},    {315, "Start"},
        {316, "Guide"},  {317, "LStick"},  {318, "RStick"},
    };

    axisMap = {
        {0, "LX"},    {1, "LY"},    {2, "LT"},    {5, "RT"},
        {3, "RX"},    {4, "RY"},    {16, "DPadX"}, {17, "DPadY"},
    };

    radialDZ["L"] = 8000;
    radialDZ["R"] = 8000;

    rawAxis["LX"] = rawAxis["LY"] = 0;
    rawAxis["RX"] = rawAxis["RY"] = 0;
    rawAxis["LT"] = rawAxis["RT"] = 0;
}

XboxController::~XboxController() {
    stop();
}

float XboxController::getLeftX() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftX;
}

float XboxController::getLeftY() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftY;
}

float XboxController::getLeftMag() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftMag;
}

float XboxController::getLeftAng() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftAng;
}

float XboxController::getRightX() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightX;
}

float XboxController::getRightY() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightY;
}

float XboxController::getRightMag() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightMag;
}

float XboxController::getRightAng() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightAng;
}


int XboxController::getLeftTrigger() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return leftTrigger;
}

int XboxController::getRightTrigger() const {
    std::lock_guard<std::mutex> lock(stickStateMutex);
    return rightTrigger;
}

void XboxController::setRadialDeadzone(const std::string& stick, int dz) {
    radialDZ[stick] = dz;
}

bool XboxController::start() {
    fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << "\n";
        return false;
    }

    running = true;
    worker = std::thread(&XboxController::eventLoop, this);
    return true;
}

void XboxController::stop() {
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

void XboxController::eventLoop() {
    input_event ev;

    while (running) {
        ssize_t n = read(fd, &ev, sizeof(ev));

        if (n == static_cast<ssize_t>(sizeof(ev))) {
            processEvent(ev);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}

void XboxController::updateStick(const std::string& stick) {
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
            leftMag = 0;
            leftAng = 0;
        } else {
            rightX = rightY = 0.0f;
            rightMag = 0;
            rightAng = 0;
        }
        return;
    }

    float rNorm = (r - dz) / (32767.0f - dz);
    if (rNorm < 0) {
        rNorm = 0;
    }
    if (rNorm > 1) {
        rNorm = 1;
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

void XboxController::processEvent(const input_event& ev) {
    if (ev.type == EV_KEY && buttonMap.count(ev.code)) {
        XboxEvent e;
        e.type = XboxEvent::Type::Button;
        e.name = buttonMap[ev.code];
        e.value = ev.value;
        queue.push(e);
        return;
    }

    if (ev.type == EV_ABS && axisMap.count(ev.code)) {
        const std::string axis = axisMap[ev.code];
        rawAxis[axis] = ev.value;

        XboxEvent e;
        e.type = XboxEvent::Type::Axis;
        e.name = axis;
        e.value = ev.value;
        queue.push(e);

        if (axis == "LX" || axis == "LY") {
            updateStick("L");
        }

        if (axis == "RX" || axis == "RY") {
            updateStick("R");
        }

        if (axis == "LT" || axis == "RT") {
            std::lock_guard<std::mutex> lock(stickStateMutex);
            leftTrigger = rawAxis["LT"];
            rightTrigger = rawAxis["RT"];
        }
    }
}
