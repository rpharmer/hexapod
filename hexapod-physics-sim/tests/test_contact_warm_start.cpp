#include "demo/contact_warm_start.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    using minphys3d::demo::transportContactWarmStart;
    bool ok = true;
    for (int i=0; i<1000; ++i) {
        const Eigen::Matrix3d oldFrame = Eigen::AngleAxisd(.001*i, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Matrix3d newFrame = oldFrame
            * Eigen::AngleAxisd(.006*i, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Vector3d impulse(.01, -.02, .10), velocity(.3, -.2, .01);
        const Eigen::Vector3d worldImpulse = oldFrame*impulse, worldVelocity = oldFrame*velocity;
        ok &= transportContactWarmStart(oldFrame, newFrame, impulse, velocity);
        ok &= (newFrame*impulse-worldImpulse).norm() < 1e-12;
        ok &= (newFrame*velocity-worldVelocity).norm() < 1e-12;
        ok &= std::abs(impulse.z()-.10) < 1e-12;
        ok &= std::abs(impulse.head<2>().norm()-std::sqrt(.0005)) < 1e-12;
        // Reverse transport and dt rescaling commute before cone projection.
        impulse *= .5;
        ok &= transportContactWarmStart(newFrame, oldFrame, impulse, velocity);
        ok &= (oldFrame*impulse-.5*worldImpulse).norm() < 1e-12;
    }
    const Eigen::Matrix3d oldFrame = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d newFrame = Eigen::AngleAxisd(.01, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d impulse(.01, -.02, .10), velocity(.3, -.2, .01);
    const Eigen::Vector3d oldImpulse = impulse, oldVelocity = velocity;
    ok &= transportContactWarmStart(oldFrame, newFrame, impulse, velocity);
    ok &= (newFrame*impulse-oldImpulse).norm() < 1e-12;
    ok &= (newFrame*velocity-oldVelocity).norm() < 1e-12;
    const Eigen::Matrix3d reversed = Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitX()).toRotationMatrix();
    ok &= !transportContactWarmStart(oldFrame, reversed, impulse, velocity);
    ok &= impulse.isZero(0) && velocity.isZero(0);
    impulse.x() = std::numeric_limits<double>::quiet_NaN();
    ok &= !transportContactWarmStart(oldFrame, newFrame, impulse, velocity);
    ok &= impulse.isZero(0) && velocity.isZero(0);
    if (!ok) std::cerr << "FAIL: rotating contact coordinates must preserve world impulse and velocity\n";
    return ok ? 0 : 1;
}
