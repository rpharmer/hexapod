#include "touch_residuals.hpp"

#include <cmath>
#include <vector>

#include "leg_kinematics_utils.hpp"

Vec3 computeFootInBodyFromServoAngles(const LegState& servo_angles,
                                      const LegGeometry& leg_geometry,
                                      const ServoCalibration& calibration) {
    LegGeometry model_leg = leg_geometry;
    model_leg.servo = calibration;

    const LegState joint_angles = model_leg.servo.toJointAngles(servo_angles);
    return kinematics::footInBodyFrame(joint_angles, model_leg);
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
