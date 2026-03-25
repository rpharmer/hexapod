#include "plane_estimation.hpp"

#include "touch_residuals.hpp"

Vec3 computeFootWorldFromServoAngles(const LegState& servo_angles,
                                     const LegGeometry& leg_geometry,
                                     const BodyPose& body_pose) {
    const Vec3 foot_body =
        computeFootInBodyFromServoAngles(servo_angles, leg_geometry, leg_geometry.servo);
    const Mat3 body_to_world = body_pose.rotationBodyToWorld();
    return body_pose.position + (body_to_world * foot_body);
}

int countContacts(const std::array<bool, kNumLegs>& contacts) {
    int count = 0;
    for (bool contact : contacts) {
        if (contact) {
            ++count;
        }
    }
    return count;
}

double meanContactPlaneHeight(const BaseClearanceSample& sample,
                              const HexapodGeometry& geometry,
                              int& used_contacts) {
    double sum_height = 0.0;
    used_contacts = 0;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!sample.foot_contacts[leg]) {
            continue;
        }

        const Vec3 foot_world = computeFootWorldFromServoAngles(
            sample.servo_angles[leg], geometry.legGeometry[leg], sample.body_pose);
        sum_height += foot_world.z;
        ++used_contacts;
    }

    if (used_contacts == 0) {
        return 0.0;
    }

    return sum_height / static_cast<double>(used_contacts);
}
