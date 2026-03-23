#pragma once

#include <string>

#include "controller_event.hpp"
#include "event_queue.hpp"

class IControlDevice {
public:
    virtual ~IControlDevice() = default;

    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual EventQueue<ControllerEvent>& getQueue() = 0;

    virtual float getLeftX() const = 0;
    virtual float getLeftY() const = 0;
    virtual float getLeftMag() const = 0;
    virtual float getLeftAng() const = 0;

    virtual float getRightX() const = 0;
    virtual float getRightY() const = 0;
    virtual float getRightMag() const = 0;
    virtual float getRightAng() const = 0;

    virtual int getLeftTrigger() const = 0;
    virtual int getRightTrigger() const = 0;

    virtual void setRadialDeadzone(const std::string& stick, int dz) = 0;
};
