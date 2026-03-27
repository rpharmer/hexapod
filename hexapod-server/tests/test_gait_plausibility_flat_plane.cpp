#include "body_controller.hpp"
#include "gait_scheduler.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class FlatPlaneFootContactEstimator {
public:
    std::array<bool, kNumLegs> update(const std::array<Vec3, kNumLegs>& foot_world_positions,
                                      double /*dt_s*/) {
        std::array<bool, kNumLegs> contacts{};

        double min_z = foot_world_positions[0].z;
        for (int leg = 1; leg < kNumLegs; ++leg) {
            min_z = std::min(min_z, foot_world_positions[leg].z);
        }

        for (int leg = 0; leg < kNumLegs; ++leg) {
            contacts[leg] = (foot_world_positions[leg].z <= min_z + kContactBandM);
        }

        return contacts;
    }

private:
    static constexpr double kContactBandM = 0.006;
};

bool walkOnFlatPlaneMaintainsPlausibleGroundMotion() {
    GaitScheduler gait_scheduler;
    BodyController body_controller;
    FlatPlaneFootContactEstimator contact_estimator;

    RobotState estimated{};
    estimated.foot_contacts = {true, true, true, true, true, true};
    for (auto& leg : estimated.leg_states) {
        leg.joint_state[FEMUR].pos_rad = AngleRad{-0.6};
        leg.joint_state[TIBIA].pos_rad = AngleRad{-0.8};
    }

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.speed_mps = LinearRateMps{0.12};
    walk.heading_rad = AngleRad{0.0};
    walk.body_pose_setpoint.body_trans_mps = VelocityMps3{walk.speed_mps.value, 0.0, 0.0};
    walk.body_pose_setpoint.orientation_rad = EulerAnglesRad3{0.0, 0.0, 0.0};
    walk.body_pose_setpoint.angular_velocity_radps = AngularVelocityRadPerSec3{0.0, 0.0, 0.0};

    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;

    std::array<std::array<double, kNumLegs>, kNumLegs> pair_distance_baseline{};
    std::array<std::array<bool, kNumLegs>, kNumLegs> pair_contact_active{};
    bool observed_pair_baseline = false;
    bool observed_estimated_contact = false;
    int contact_agreement_samples = 0;
    int contact_agreement_matches = 0;
    int directional_samples = 0;
    int opposite_heading_samples = 0;
    int low_lateral_drift_samples = 0;

    Vec3 body_position_world{0.0, 0.0, 0.20};
    TimePointUs previous_targets_ts{};
    std::array<Vec3, kNumLegs> previous_body_targets{};
    bool have_previous_body_targets = false;

    for (int i = 0; i < 180; ++i) {
        walk.timestamp_us = now_us();
        const GaitState gait = gait_scheduler.update(estimated, walk, safety);
        const LegTargets targets = body_controller.update(estimated, walk, gait, safety);

        double dt_s = 0.005;
        if (!previous_targets_ts.isZero() && targets.timestamp_us.value > previous_targets_ts.value) {
            dt_s = static_cast<double>(targets.timestamp_us.value - previous_targets_ts.value) * 1e-6;
        }
        previous_targets_ts = targets.timestamp_us;

        body_position_world.x += walk.speed_mps.value * dt_s;

        std::array<Vec3, kNumLegs> foot_world_positions{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            foot_world_positions[leg] = body_position_world + static_cast<Vec3>(targets.feet[leg].pos_body_m);
        }

        const std::array<bool, kNumLegs> estimated_contacts = contact_estimator.update(foot_world_positions, dt_s);
        const std::array<bool, kNumLegs>& contacts = gait.in_stance;

        for (int leg = 0; leg < kNumLegs; ++leg) {
            ++contact_agreement_samples;
            if (estimated_contacts[leg] == contacts[leg]) {
                ++contact_agreement_matches;
            }
            if (estimated_contacts[leg]) {
                observed_estimated_contact = true;
            }
        }

        for (int a = 0; a < kNumLegs; ++a) {
            if (!contacts[a]) {
                continue;
            }
            for (int b = a + 1; b < kNumLegs; ++b) {
                if (!contacts[b]) {
                    continue;
                }

                const Vec3 planar_delta = foot_world_positions[b] - foot_world_positions[a];
                const double distance = std::sqrt(planar_delta.x * planar_delta.x +
                                                  planar_delta.y * planar_delta.y);
                if (!pair_contact_active[a][b]) {
                    pair_distance_baseline[a][b] = distance;
                    pair_contact_active[a][b] = true;
                    observed_pair_baseline = true;
                } else {
                    const double error = std::abs(distance - pair_distance_baseline[a][b]);
                    if (!expect(error < 0.015,
                                "distance between grounded legs should stay nearly constant")) {
                        return false;
                    }
                }
            }
        }


        for (int a = 0; a < kNumLegs; ++a) {
            for (int b = a + 1; b < kNumLegs; ++b) {
                if (!(contacts[a] && contacts[b])) {
                    pair_contact_active[a][b] = false;
                }
            }
        }

        if (have_previous_body_targets && dt_s > 1e-6) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!contacts[leg]) {
                    continue;
                }

                const Vec3 planar_velocity =
                    (static_cast<Vec3>(targets.feet[leg].pos_body_m) - previous_body_targets[leg]) * (1.0 / dt_s);
                const double speed = std::sqrt(planar_velocity.x * planar_velocity.x +
                                               planar_velocity.y * planar_velocity.y);
                if (speed < 1e-3) {
                    continue;
                }

                ++directional_samples;
                if (planar_velocity.x < 0.0) {
                    ++opposite_heading_samples;
                }
                if (std::abs(planar_velocity.y) <= std::abs(planar_velocity.x) * 0.35) {
                    ++low_lateral_drift_samples;
                }
            }
        }

        for (int leg = 0; leg < kNumLegs; ++leg) {
            previous_body_targets[leg] = static_cast<Vec3>(targets.feet[leg].pos_body_m);
        }
        have_previous_body_targets = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const double opposite_ratio =
        (directional_samples > 0) ? static_cast<double>(opposite_heading_samples) / directional_samples : 0.0;
    const double low_lateral_ratio =
        (directional_samples > 0) ? static_cast<double>(low_lateral_drift_samples) / directional_samples : 0.0;
    const double contact_agreement_ratio =
        (contact_agreement_samples > 0) ? static_cast<double>(contact_agreement_matches) / contact_agreement_samples : 0.0;

    return expect(observed_estimated_contact, "flat-plane contact estimator should detect contacts") &&
           expect(contact_agreement_ratio > 0.55,
                  "flat-plane contact estimator should broadly agree with gait stance phases") &&
           expect(observed_pair_baseline, "contact windows should provide leg-pair baselines") &&
           expect(directional_samples > 30, "simulation should provide sufficient grounded direction samples") &&
           expect(opposite_ratio > 0.70,
                  "most grounded feet should translate opposite the forward heading while not turning") &&
           expect(low_lateral_ratio > 0.80,
                  "grounded feet should mostly move along a shared heading with low lateral drift");
}

} // namespace

int main() {
    if (!walkOnFlatPlaneMaintainsPlausibleGroundMotion()) {
        return EXIT_FAILURE;
    }

    std::cout << "PASS: flat-plane gait plausibility" << std::endl;
    return EXIT_SUCCESS;
}
