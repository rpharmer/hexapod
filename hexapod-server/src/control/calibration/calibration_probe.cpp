#include "calibration_probe.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "plane_estimation.hpp"
#include "probe_contact_logic.hpp"
#include "servo_dynamics_fit.hpp"
#include "touch_residuals.hpp"

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
    double current_rms = computeTouchResidualRms(
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

            const double rms_plus = computeTouchResidualRms(
                contact_samples, leg_geometry, plus, options.plane_height_m);
            const double rms_minus = computeTouchResidualRms(
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

        const double next_rms = computeTouchResidualRms(
            contact_samples, leg_geometry, next, options.plane_height_m);
        if (next_rms + options.improvement_epsilon >= current_rms) {
            break;
        }

        current = next;
        current_rms = next_rms;
    }

    result.fitted_calibration = current;
    result.rms_error_before_m = computeTouchResidualRms(
        contact_samples, leg_geometry, result.initial_calibration,
        options.plane_height_m);
    result.rms_error_after_m = computeTouchResidualRms(
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
            vecNorm(sample.foot_position_body - sample.target_position_body) <=
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
        vecNorm(last.foot_position_body - last.target_position_body) <=
        options.position_tolerance_m;
    return result;
}
