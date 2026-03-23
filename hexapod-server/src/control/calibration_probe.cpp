#include "calibration_probe.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

Vec3 computeFootInBodyFromServoAngles(const LegRawState& servo_angles,
                                      const LegGeometry& leg_geometry,
                                      const ServoCalibration& calibration) {
    LegGeometry model_leg = leg_geometry;
    model_leg.servo = calibration;

    const LegRawState joint_angles = model_leg.servo.toJointAngles(servo_angles);
    const std::array<JointRawState, kJointsPerLeg> joints = joint_angles.joint_raw_state;

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

double computeRms(const std::vector<const CalibrationTouchSample*>& samples,
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

double clampedDelta(double candidate, double limit_abs) {
    return std::clamp(candidate, -limit_abs, limit_abs);
}

double legJoint(const LegRawState& leg_state, int joint) {
    return leg_state.joint_raw_state[joint].pos_rad.value;
}

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 computeFootWorldFromServoAngles(const LegRawState& servo_angles,
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

SingleJointDynamicsFit fitSingleJointDirection(
    const std::vector<ServoDynamicsSample>& samples,
    int joint,
    bool positive_direction,
    const ServoDynamicsFitOptions& options) {
    SingleJointDynamicsFit fit{};
    if (samples.size() < 2) {
        return fit;
    }

    std::vector<std::pair<double, double>> err_and_velocity{};
    err_and_velocity.reserve(samples.size() - 1);
    double vmax_observed = 0.0;

    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].time_s - samples[i - 1].time_s;
        if (dt < options.min_dt_s) {
            continue;
        }

        const double measured_prev = legJoint(samples[i - 1].measured_servo_angles, joint);
        const double measured_now = legJoint(samples[i].measured_servo_angles, joint);
        const double commanded_now = legJoint(samples[i].command_servo_angles, joint);
        const double velocity = (measured_now - measured_prev) / dt;
        const double err = commanded_now - measured_prev;

        if (std::abs(err) < options.min_err_rad) {
            continue;
        }

        if (positive_direction && velocity <= 0.0) {
            continue;
        }
        if (!positive_direction && velocity >= 0.0) {
            continue;
        }

        vmax_observed = std::max(vmax_observed, std::abs(velocity));
        err_and_velocity.emplace_back(err, velocity);
    }

    if (static_cast<int>(err_and_velocity.size()) < options.min_samples ||
        vmax_observed <= 0.0) {
        return fit;
    }

    const double unsat_v = options.unsaturated_velocity_fraction * vmax_observed;
    double err_sq_sum = 0.0;
    double err_vel_sum = 0.0;
    int used = 0;

    for (const auto& [err, vel] : err_and_velocity) {
        if (std::abs(vel) >= unsat_v) {
            continue;
        }
        err_sq_sum += err * err;
        err_vel_sum += err * vel;
        ++used;
    }

    if (used < options.min_samples || err_sq_sum <= 1e-12) {
        return fit;
    }

    const double k = err_vel_sum / err_sq_sum; // dq = k * err
    if (!std::isfinite(k) || std::abs(k) <= 1e-9) {
        return fit;
    }

    fit.success = true;
    fit.tau_s = 1.0 / std::abs(k);
    fit.vmax_radps = vmax_observed;
    fit.used_samples = used;
    return fit;
}

bool hasDebouncedContact(const std::vector<ProbeDestinationSample>& samples,
                         std::size_t index,
                         int debounce_samples) {
    if (debounce_samples <= 1) {
        return samples[index].foot_contact;
    }
    if (index + 1 < static_cast<std::size_t>(debounce_samples)) {
        return false;
    }
    for (int k = 0; k < debounce_samples; ++k) {
        if (!samples[index - static_cast<std::size_t>(k)].foot_contact) {
            return false;
        }
    }
    return true;
}

} // namespace

CalibrationLegFitResult fitServoCalibrationFromTouches(
    const LegGeometry& leg_geometry,
    const std::vector<CalibrationTouchSample>& samples,
    const CalibrationFitOptions& options) {
    CalibrationLegFitResult result{};

    const std::vector<const CalibrationTouchSample*> contact_samples =
        filterContactSamples(samples);
    result.used_samples = static_cast<int>(contact_samples.size());
    result.initial_calibration = leg_geometry.servo;
    result.fitted_calibration = leg_geometry.servo;

    if (contact_samples.size() < 3) {
        return result;
    }

    ServoCalibration current = leg_geometry.servo;
    const ServoCalibration base = leg_geometry.servo;
    double current_rms = computeRms(
        contact_samples, leg_geometry, current, options.plane_height_m);

    for (int iter = 0; iter < options.max_iterations; ++iter) {
        std::array<double, kJointsPerLeg> grad{};

        auto finiteDiff = [&](int joint_index, ServoCalibration plus,
                              ServoCalibration minus) {
            switch (joint_index) {
                case COXA:
                    plus.coxaOffset += AngleRad{options.finite_diff_step_rad};
                    minus.coxaOffset -= AngleRad{options.finite_diff_step_rad};
                    break;
                case FEMUR:
                    plus.femurOffset += AngleRad{options.finite_diff_step_rad};
                    minus.femurOffset -= AngleRad{options.finite_diff_step_rad};
                    break;
                case TIBIA:
                    plus.tibiaOffset += AngleRad{options.finite_diff_step_rad};
                    minus.tibiaOffset -= AngleRad{options.finite_diff_step_rad};
                    break;
                default:
                    break;
            }

            const double rms_plus = computeRms(
                contact_samples, leg_geometry, plus, options.plane_height_m);
            const double rms_minus = computeRms(
                contact_samples, leg_geometry, minus, options.plane_height_m);

            return (rms_plus - rms_minus) / (2.0 * options.finite_diff_step_rad);
        };

        grad[COXA] = finiteDiff(COXA, current, current);
        grad[FEMUR] = finiteDiff(FEMUR, current, current);
        grad[TIBIA] = finiteDiff(TIBIA, current, current);

        ServoCalibration next = current;
        next.coxaOffset -= AngleRad{options.learning_rate * grad[COXA]};
        next.femurOffset -= AngleRad{options.learning_rate * grad[FEMUR]};
        next.tibiaOffset -= AngleRad{options.learning_rate * grad[TIBIA]};

        next.coxaOffset =
            base.coxaOffset +
            AngleRad{clampedDelta(next.coxaOffset.value - base.coxaOffset.value,
                                  options.max_offset_delta_rad)};
        next.femurOffset =
            base.femurOffset +
            AngleRad{clampedDelta(next.femurOffset.value - base.femurOffset.value,
                                  options.max_offset_delta_rad)};
        next.tibiaOffset =
            base.tibiaOffset +
            AngleRad{clampedDelta(next.tibiaOffset.value - base.tibiaOffset.value,
                                  options.max_offset_delta_rad)};

        const double next_rms = computeRms(
            contact_samples, leg_geometry, next, options.plane_height_m);
        if (next_rms + options.improvement_epsilon >= current_rms) {
            break;
        }

        current = next;
        current_rms = next_rms;
    }

    result.fitted_calibration = current;
    result.rms_error_before_m = computeRms(
        contact_samples, leg_geometry, result.initial_calibration,
        options.plane_height_m);
    result.rms_error_after_m = computeRms(
        contact_samples, leg_geometry, result.fitted_calibration,
        options.plane_height_m);
    result.coxa_delta = AngleRad{result.fitted_calibration.coxaOffset.value -
                                 result.initial_calibration.coxaOffset.value};
    result.femur_delta = AngleRad{result.fitted_calibration.femurOffset.value -
                                  result.initial_calibration.femurOffset.value};
    result.tibia_delta = AngleRad{result.fitted_calibration.tibiaOffset.value -
                                  result.initial_calibration.tibiaOffset.value};
    result.success = result.rms_error_after_m < result.rms_error_before_m;

    return result;
}

BaseClearanceEstimateResult estimateToBottomFromSynchronousLift(
    const HexapodGeometry& geometry,
    const std::vector<BaseClearanceSample>& samples,
    const BaseClearanceEstimateOptions& options) {
    BaseClearanceEstimateResult result{};
    if (samples.size() < 2) {
        return result;
    }

    const int min_contacts_before_loss = std::max(
        1, static_cast<int>(std::ceil(options.min_contact_fraction_before_loss *
                                      static_cast<double>(kNumLegs))));

    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        const BaseClearanceSample& before = samples[i];
        const BaseClearanceSample& after = samples[i + 1];

        const int contacts_before = countContacts(before.foot_contacts);
        const int contacts_after = countContacts(after.foot_contacts);
        if (contacts_before < min_contacts_before_loss || contacts_after != 0) {
            continue;
        }

        int used_contacts = 0;
        const double plane_height_m = meanContactPlaneHeight(before, geometry, used_contacts);
        if (used_contacts < min_contacts_before_loss) {
            continue;
        }

        result.success = true;
        result.transition_index = static_cast<int>(i + 1);
        result.contact_legs_before_loss = contacts_before;
        result.ground_plane_height_m = plane_height_m;
        result.estimated_to_bottom_m =
            LengthM{after.body_pose.position.z - plane_height_m};
        return result;
    }

    return result;
}

LegServoDynamicsFitResult fitLegServoDynamicsFromSamples(
    const std::vector<ServoDynamicsSample>& samples,
    const ServoDynamicsFitOptions& options) {
    LegServoDynamicsFitResult result{};
    for (int joint = 0; joint < kJointsPerLeg; ++joint) {
        result.positive_direction[joint] =
            fitSingleJointDirection(samples, joint, true, options);
        result.negative_direction[joint] =
            fitSingleJointDirection(samples, joint, false, options);
    }
    return result;
}

ProbeDestinationResult evaluateProbeDestinationReached(
    const std::vector<ProbeDestinationSample>& samples,
    const ProbeDestinationOptions& options) {
    ProbeDestinationResult result{};
    if (samples.empty()) {
        return result;
    }

    const double start_time = samples.front().time_s;
    const double timeout_time = start_time + options.timeout_s;
    double contact_rise_time = -1.0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const ProbeDestinationSample& sample = samples[i];
        const bool debounced = hasDebouncedContact(samples, i, options.contact_debounce_samples);
        const bool in_position =
            norm(sample.foot_position_body - sample.target_position_body) <=
            options.position_tolerance_m;

        if (debounced && contact_rise_time < 0.0) {
            contact_rise_time = sample.time_s;
        }

        if (debounced && !in_position) {
            result.status = ProbeDestinationStatus::EarlyContact;
            result.decision_time_s = sample.time_s;
            result.contact_rise_time_s = contact_rise_time;
            result.contact_debounced = true;
            result.in_position = false;
            return result;
        }

        if (debounced && in_position) {
            result.status = ProbeDestinationStatus::Reached;
            result.reached = true;
            result.decision_time_s = sample.time_s;
            result.contact_rise_time_s = contact_rise_time;
            result.contact_debounced = true;
            result.in_position = true;
            return result;
        }

        if (sample.time_s >= timeout_time) {
            result.status = in_position ? ProbeDestinationStatus::TimeoutNoContact
                                        : ProbeDestinationStatus::TimeoutNoProgress;
            result.decision_time_s = sample.time_s;
            result.contact_rise_time_s = contact_rise_time;
            result.contact_debounced = debounced;
            result.in_position = in_position;
            return result;
        }
    }

    const ProbeDestinationSample& last = samples.back();
    result.status = ProbeDestinationStatus::NotReached;
    result.decision_time_s = last.time_s;
    result.contact_rise_time_s = contact_rise_time;
    result.contact_debounced = false;
    result.in_position =
        norm(last.foot_position_body - last.target_position_body) <=
        options.position_tolerance_m;
    return result;
}
