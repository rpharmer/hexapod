#include "touch_residuals.hpp"

#include <array>
#include <cmath>
#include <vector>

Vec3 computeFootInBodyFromServoAngles(const LegState& servo_angles,
                                      const LegGeometry& leg_geometry,
                                      const ServoCalibration& calibration) {
    LegGeometry model_leg = leg_geometry;
    model_leg.servo = calibration;

    const LegState joint_angles = model_leg.servo.toJointAngles(servo_angles);
    const std::array<JointState, kJointsPerLeg> joints = joint_angles.joint_state;

    const double q1 = joints[COXA].pos_rad.value;
    const double q2 = joints[FEMUR].pos_rad.value;
    const double q3 = joints[TIBIA].pos_rad.value;

    const double rho =
        model_leg.femurLength.value * std::cos(q2) +
        model_leg.tibiaLength.value * std::cos(q2 + q3);
    const double z_leg =
        model_leg.femurLength.value * std::sin(q2) +
        model_leg.tibiaLength.value * std::sin(q2 + q3);
    const double r = model_leg.coxaLength.value + rho;

    const Vec3 foot_leg{
        r * std::cos(q1),
        r * std::sin(q1),
        z_leg,
    };

    const Mat3 body_from_leg = Mat3::rotZ(model_leg.mountAngle.value);
    return model_leg.bodyCoxaOffset + (body_from_leg * foot_leg);
}

double touchResidualMeters(const CalibrationTouchSample& sample,
                           const LegGeometry& leg_geometry,
                           const ServoCalibration& calibration,
                           double plane_height_m) {
    const Vec3 foot_body = computeFootInBodyFromServoAngles(
        sample.servo_angles, leg_geometry, calibration);
    const Mat3 body_to_world = sample.body_pose.rotationBodyToWorld();
    const Vec3 foot_world = sample.body_pose.position + (body_to_world * foot_body);

    return foot_world.z - plane_height_m;
}

std::vector<const CalibrationTouchSample*> filterContactSamples(
    const std::vector<CalibrationTouchSample>& samples) {
    std::vector<const CalibrationTouchSample*> filtered{};
    filtered.reserve(samples.size());

    for (const CalibrationTouchSample& sample : samples) {
        if (sample.contact) {
            filtered.push_back(&sample);
        }
    }

    return filtered;
}

double computeTouchResidualRms(const std::vector<const CalibrationTouchSample*>& samples,
                               const LegGeometry& leg_geometry,
                               const ServoCalibration& calibration,
                               double plane_height_m) {
    if (samples.empty()) {
        return 0.0;
    }

    double squared_sum = 0.0;
    for (const CalibrationTouchSample* sample : samples) {
        const double residual = touchResidualMeters(
            *sample, leg_geometry, calibration, plane_height_m);
        squared_sum += residual * residual;
    }

    return std::sqrt(squared_sum / static_cast<double>(samples.size()));
}
