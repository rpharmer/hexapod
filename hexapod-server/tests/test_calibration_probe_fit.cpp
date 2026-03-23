#include "calibration_probe.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include "geometry_config.hpp"
#include "leg_fk.hpp"

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

LegRawState makeJointState(double coxa, double femur, double tibia) {
    LegRawState joint{};
    joint.joint_raw_state[COXA].pos_rad = AngleRad{coxa};
    joint.joint_raw_state[FEMUR].pos_rad = AngleRad{femur};
    joint.joint_raw_state[TIBIA].pos_rad = AngleRad{tibia};
    return joint;
}

CalibrationTouchSample makeTouchSample(const LegGeometry& leg,
                                       const ServoCalibration& true_cal,
                                       const LegRawState& joint_true) {
    LegFK fk{};
    CalibrationTouchSample sample{};
    sample.contact = true;
    sample.servo_angles = true_cal.toServoAngles(joint_true);

    const Vec3 foot_body = fk.footInBodyFrame(joint_true, leg).pos_body_m;
    sample.body_pose.position = Vec3{0.0, 0.0, -foot_body.z};
    sample.body_pose.roll = AngleRad{0.0};
    sample.body_pose.pitch = AngleRad{0.0};
    sample.body_pose.yaw = AngleRad{0.0};
    return sample;
}

BodyPose makeLevelPoseAtHeight(double z_m) {
    BodyPose pose{};
    pose.position = Vec3{0.0, 0.0, z_m};
    pose.roll = AngleRad{0.0};
    pose.pitch = AngleRad{0.0};
    pose.yaw = AngleRad{0.0};
    return pose;
}

BaseClearanceSample makeBaseClearanceSample(const HexapodGeometry& geometry,
                                            const BodyPose& pose,
                                            const std::array<bool, kNumLegs>& contacts) {
    LegFK fk{};
    BaseClearanceSample sample{};
    sample.body_pose = pose;
    sample.foot_contacts = contacts;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const LegGeometry& leg_geometry = geometry.legGeometry[leg];
        const Vec3 mount = leg_geometry.bodyCoxaOffset;
        const double q1 = std::atan2(-mount.y, -mount.x) - leg_geometry.mountAngle.value;
        const LegRawState joint = makeJointState(q1, -0.72, 1.05);
        sample.servo_angles[leg] = leg_geometry.servo.toServoAngles(joint);

        if (contacts[leg]) {
            const Vec3 foot_body = fk.footInBodyFrame(joint, leg_geometry).pos_body_m;
            sample.body_pose.position.z = -foot_body.z;
        }
    }

    return sample;
}

double meanContactHeight(const BaseClearanceSample& sample,
                         const HexapodGeometry& geometry) {
    LegFK fk{};
    double sum = 0.0;
    int count = 0;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!sample.foot_contacts[leg]) {
            continue;
        }
        const LegRawState joint =
            geometry.legGeometry[leg].servo.toJointAngles(sample.servo_angles[leg]);
        const Vec3 foot_body =
            fk.footInBodyFrame(joint, geometry.legGeometry[leg]).pos_body_m;
        sum += sample.body_pose.position.z + foot_body.z;
        ++count;
    }

    return count > 0 ? (sum / static_cast<double>(count)) : 0.0;
}

bool test_fit_improves_surface_touch_error() {
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    LegGeometry leg = geometry.legGeometry[0];

    ServoCalibration true_cal = leg.servo;

    std::vector<LegRawState> joint_states{
        makeJointState(0.1, -0.75, 1.0),
        makeJointState(0.25, -0.68, 0.93),
        makeJointState(-0.2, -0.82, 1.08),
        makeJointState(0.35, -0.7, 1.02),
        makeJointState(-0.05, -0.77, 0.98),
        makeJointState(0.15, -0.73, 1.05),
    };

    std::vector<CalibrationTouchSample> samples{};
    samples.reserve(joint_states.size());
    for (const LegRawState& state : joint_states) {
        samples.push_back(makeTouchSample(leg, true_cal, state));
    }

    leg.servo.coxaOffset += AngleRad{0.12};
    leg.servo.femurOffset -= AngleRad{0.08};
    leg.servo.tibiaOffset += AngleRad{0.06};

    CalibrationFitOptions options{};
    options.max_iterations = 60;
    options.learning_rate = 0.15;

    const CalibrationLegFitResult result =
        fitServoCalibrationFromTouches(leg, samples, options);

    const double initial_err =
        std::abs(result.initial_calibration.coxaOffset.value - true_cal.coxaOffset.value) +
        std::abs(result.initial_calibration.femurOffset.value - true_cal.femurOffset.value) +
        std::abs(result.initial_calibration.tibiaOffset.value - true_cal.tibiaOffset.value);

    const double fitted_err =
        std::abs(result.fitted_calibration.coxaOffset.value - true_cal.coxaOffset.value) +
        std::abs(result.fitted_calibration.femurOffset.value - true_cal.femurOffset.value) +
        std::abs(result.fitted_calibration.tibiaOffset.value - true_cal.tibiaOffset.value);

    if (!expect(result.success, "fit should report success")) {
        return false;
    }
    if (!expect(result.used_samples == static_cast<int>(samples.size()),
                "fit should use all contact samples")) {
        return false;
    }
    if (!expect(result.rms_error_after_m < result.rms_error_before_m,
                "rms should improve")) {
        return false;
    }
    if (!expect(fitted_err < initial_err,
                "fitted calibration should be closer to ground truth")) {
        return false;
    }

    return true;
}

bool test_requires_minimum_contact_samples() {
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    const LegGeometry leg = geometry.legGeometry[1];

    std::vector<CalibrationTouchSample> samples(2);
    for (CalibrationTouchSample& sample : samples) {
        sample.contact = true;
    }

    const CalibrationLegFitResult result =
        fitServoCalibrationFromTouches(leg, samples);

    if (!expect(!result.success, "fit should fail with fewer than 3 contacts")) {
        return false;
    }
    if (!expect(result.used_samples == 2, "used sample count should match contacts")) {
        return false;
    }
    return true;
}

bool test_estimate_to_bottom_from_contact_loss_transition() {
    HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();

    const std::array<bool, kNumLegs> all_contact{true, true, true, true, true, true};
    const std::array<bool, kNumLegs> no_contact{false, false, false, false, false, false};

    BaseClearanceSample before =
        makeBaseClearanceSample(geometry, makeLevelPoseAtHeight(0.06), all_contact);
    const double plane_height = meanContactHeight(before, geometry);

    BaseClearanceSample after = makeBaseClearanceSample(
        geometry,
        makeLevelPoseAtHeight(geometry.toBottom.value + plane_height),
        no_contact);

    std::vector<BaseClearanceSample> samples{};
    samples.push_back(before);
    samples.push_back(after);

    const BaseClearanceEstimateResult result =
        estimateToBottomFromSynchronousLift(geometry, samples);

    if (!expect(result.success, "to-bottom estimate should succeed")) {
        return false;
    }
    if (!expect(result.transition_index == 1,
                "transition index should point to the first all-off-ground sample")) {
        return false;
    }
    if (!expect(std::abs(result.estimated_to_bottom_m.value - geometry.toBottom.value) < 1e-4,
                "estimated to-bottom should match geometry")) {
        return false;
    }

    return true;
}

bool test_estimate_to_bottom_requires_clear_transition() {
    const HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    const std::array<bool, kNumLegs> all_contact{true, true, true, true, true, true};
    std::vector<BaseClearanceSample> samples{};
    samples.push_back(
        makeBaseClearanceSample(geometry, makeLevelPoseAtHeight(0.06), all_contact));
    samples.push_back(
        makeBaseClearanceSample(geometry, makeLevelPoseAtHeight(0.055), all_contact));

    const BaseClearanceEstimateResult result =
        estimateToBottomFromSynchronousLift(geometry, samples);

    if (!expect(!result.success, "to-bottom estimate should fail without a loss-of-contact transition")) {
        return false;
    }

    return true;
}

} // namespace

int main() {
    if (!test_fit_improves_surface_touch_error()) {
        return 1;
    }
    if (!test_requires_minimum_contact_samples()) {
        return 1;
    }
    if (!test_estimate_to_bottom_from_contact_loss_transition()) {
        return 1;
    }
    if (!test_estimate_to_bottom_requires_clear_transition()) {
        return 1;
    }

    std::cout << "Calibration probe fit tests passed\n";
    return 0;
}
