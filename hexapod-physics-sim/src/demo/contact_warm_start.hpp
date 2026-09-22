#pragma once
#include <Eigen/Core>

namespace minphys3d::demo {
// Contact coordinates are not persistent world vectors. Return false when the
// normal reverses (the old constraint no longer represents the same surface).
inline bool transportContactWarmStart(const Eigen::Matrix3d& oldFrame,
                                     const Eigen::Matrix3d& newFrame,
                                     Eigen::Vector3d& impulse,
                                     Eigen::Vector3d& velocity) {
    if (!oldFrame.allFinite() || !newFrame.allFinite() || !impulse.allFinite()
        || !velocity.allFinite() || oldFrame.col(2).dot(newFrame.col(2)) <= 0.0) {
        impulse.setZero();
        velocity.setZero();
        return false;
    }
    const Eigen::Matrix3d transport = newFrame.transpose() * oldFrame;
    impulse = (transport * impulse).eval();
    velocity = (transport * velocity).eval();
    return true;
}
}
