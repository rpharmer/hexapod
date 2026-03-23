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

LegState makeJointState(double coxa, double femur, double tibia) {
    LegState joint{};
    joint.joint_state[COXA].pos_rad = AngleRad{coxa};
    joint.joint_state[FEMUR].pos_rad = AngleRad{femur};
    joint.joint_state[TIBIA].pos_rad = AngleRad{tibia};
    return joint;
}

CalibrationTouchSample makeTouchSample(const LegGeometry& leg,
                                       const ServoCalibration& true_cal,
                                       const LegState& joint_true) {
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
        const LegState joint = makeJointState(q1, -0.72, 1.05);
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
        const LegState joint =
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

    std::vector<LegState> joint_states{
        makeJointState(0.1, -0.75, 1.0),
        makeJointState(0.25, -0.68, 0.93),
        makeJointState(-0.2, -0.82, 1.08),
        makeJointState(0.35, -0.7, 1.02),
        makeJointState(-0.05, -0.77, 0.98),
        makeJointState(0.15, -0.73, 1.05),
    };

    std::vector<CalibrationTouchSample> samples{};
    samples.reserve(joint_states.size());
    for (const LegState& state : joint_states) {
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

ServoDynamicsSample makeDynamicsSample(double t,
                                       const LegState& command,
                                       const LegState& measured) {
    ServoDynamicsSample sample{};
    sample.time_s = t;
    sample.command_servo_angles = command;
    sample.measured_servo_angles = measured;
    return sample;
}

bool test_fit_servo_dynamics_recovers_tau_and_vmax() {
    constexpr double dt = 0.01;
    constexpr double tau = 0.12;
    constexpr double vmax = 4.0;

    std::vector<ServoDynamicsSample> samples{};
    samples.reserve(220);

    LegState measured = makeJointState(0.0, 0.0, 0.0);
    for (int i = 0; i < 220; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double q_cmd = (i < 100) ? 0.95 : -0.85;
        LegState command = makeJointState(q_cmd, q_cmd, q_cmd);

        for (int j = 0; j < kJointsPerLeg; ++j) {
            const double q_prev = measured.joint_state[j].pos_rad.value;
            const double err = q_cmd - q_prev;
            const double dq = std::clamp(err / tau, -vmax, vmax);
            measured.joint_state[j].pos_rad = AngleRad{q_prev + dq * dt};
        }

        samples.push_back(makeDynamicsSample(t, command, measured));
    }

    ServoDynamicsFitOptions options{};
    options.min_samples = 20;
    const LegServoDynamicsFitResult fit = fitLegServoDynamicsFromSamples(samples, options);

    for (int j = 0; j < kJointsPerLeg; ++j) {
        if (!expect(fit.positive_direction[j].success, "positive-direction dynamics fit should succeed")) {
            return false;
        }
        if (!expect(fit.negative_direction[j].success, "negative-direction dynamics fit should succeed")) {
            return false;
        }
        if (!expect(std::abs(fit.positive_direction[j].tau_s - tau) < 0.03,
                    "positive-direction tau should match synthetic model")) {
            return false;
        }
        if (!expect(std::abs(fit.negative_direction[j].tau_s - tau) < 0.03,
                    "negative-direction tau should match synthetic model")) {
            return false;
        }
        if (!expect(std::abs(fit.positive_direction[j].vmax_radps - vmax) < 0.35,
                    "positive-direction vmax should match synthetic model")) {
            return false;
        }
        if (!expect(std::abs(fit.negative_direction[j].vmax_radps - vmax) < 0.35,
                    "negative-direction vmax should match synthetic model")) {
            return false;
        }
    }

    return true;
}

ProbeDestinationSample makeDestinationSample(double t, double x, bool contact) {
    ProbeDestinationSample sample{};
    sample.time_s = t;
    sample.foot_position_body = Vec3{x, 0.0, 0.0};
    sample.target_position_body = Vec3{0.0, 0.0, 0.0};
    sample.foot_contact = contact;
    return sample;
}

bool test_destination_requires_contact_and_position() {
    std::vector<ProbeDestinationSample> samples{};
    samples.push_back(makeDestinationSample(0.00, 0.030, false));
    samples.push_back(makeDestinationSample(0.05, 0.015, true));
    samples.push_back(makeDestinationSample(0.10, 0.002, true));
    samples.push_back(makeDestinationSample(0.15, 0.001, true));

    ProbeDestinationOptions options{};
    options.position_tolerance_m = 0.003;
    options.contact_debounce_samples = 2;
    options.timeout_s = 0.5;
    const ProbeDestinationResult result =
        evaluateProbeDestinationReached(samples, options);

    if (!expect(result.reached, "destination should be reached with in-position + debounced contact")) {
        return false;
    }
    if (!expect(result.status == ProbeDestinationStatus::Reached,
                "status should be reached")) {
        return false;
    }

    return true;
}

bool test_destination_detects_early_contact() {
    std::vector<ProbeDestinationSample> samples{};
    samples.push_back(makeDestinationSample(0.00, 0.030, false));
    samples.push_back(makeDestinationSample(0.05, 0.025, true));
    samples.push_back(makeDestinationSample(0.10, 0.020, true));
    samples.push_back(makeDestinationSample(0.20, 0.005, true));

    ProbeDestinationOptions options{};
    options.position_tolerance_m = 0.003;
    options.contact_debounce_samples = 2;
    options.timeout_s = 0.5;
    const ProbeDestinationResult result =
        evaluateProbeDestinationReached(samples, options);

    if (!expect(!result.reached, "early contact should not report reached")) {
        return false;
    }
    if (!expect(result.status == ProbeDestinationStatus::EarlyContact,
                "status should classify early contact")) {
        return false;
    }
    return true;
}

bool test_destination_timeout_without_contact() {
    std::vector<ProbeDestinationSample> samples{};
    samples.push_back(makeDestinationSample(0.00, 0.030, false));
    samples.push_back(makeDestinationSample(0.25, 0.002, false));
    samples.push_back(makeDestinationSample(0.55, 0.001, false));

    ProbeDestinationOptions options{};
    options.position_tolerance_m = 0.003;
    options.contact_debounce_samples = 2;
    options.timeout_s = 0.5;
    const ProbeDestinationResult result =
        evaluateProbeDestinationReached(samples, options);

    if (!expect(!result.reached, "timeout without contact should not be reached")) {
        return false;
    }
    if (!expect(result.status == ProbeDestinationStatus::TimeoutNoContact,
                "status should classify timeout with geometric reach but no contact")) {
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
    if (!test_fit_servo_dynamics_recovers_tau_and_vmax()) {
        return 1;
    }
    if (!test_destination_requires_contact_and_position()) {
        return 1;
    }
    if (!test_destination_detects_early_contact()) {
        return 1;
    }
    if (!test_destination_timeout_without_contact()) {
        return 1;
    }

    std::cout << "Calibration probe fit tests passed\n";
    return 0;
}
