#include "leg_kinematics_utils.hpp"

#include <cmath>

namespace kinematics {

Vec3 footInLegFrame(const LegState& joint_angles, const LegGeometry& geometry) {
    const std::array<JointState, kJointsPerLeg>& joints = joint_angles.joint_state;
    const double q1 = joints[COXA].pos_rad.value;
    const double q2 = joints[FEMUR].pos_rad.value;
    const double q3 = joints[TIBIA].pos_rad.value;

    const double rho = geometry.femurLength.value * std::cos(q2) +
                       geometry.tibiaLength.value * std::cos(q2 + q3);
    const double z_leg = geometry.femurLength.value * std::sin(q2) +
                         geometry.tibiaLength.value * std::sin(q2 + q3);
    const double r = geometry.coxaLength.value + rho;
    return Vec3{r * std::cos(q1), r * std::sin(q1), z_leg};
}

Vec3 footInBodyFrame(const LegState& joint_angles, const LegGeometry& geometry) {
    const Vec3 foot_leg = footInLegFrame(joint_angles, geometry);
    const Mat3 body_from_leg = Mat3::rotZ(geometry.mountAngle.value);
    return geometry.bodyCoxaOffset + (body_from_leg * foot_leg);
}

Point2 footPointInBodyPlane(const LegState& joint_angles, const LegGeometry& geometry) {
    const Vec3 foot_body = footInBodyFrame(joint_angles, geometry);
    return Point2{foot_body.x, foot_body.y};
}

}  // namespace kinematics
