#pragma once

namespace minphys3d::demo {
inline double servoPdRequest(double kp, double kd, double error, double velocity,
                             double proportionalScale, double dampingScale) {
    return proportionalScale * kp * error - dampingScale * kd * velocity;
}
}
