#include "body_controller.hpp"
#include "leg_ik.hpp"
#include "motion_intent_utils.hpp"

#include <cmath>
#include <iostream>
#include <limits>

int main() {
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.leg_enabled.fill(true);
    auto intent = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, .14);
    GaitState gait{};
    gait.in_stance.fill(true);
    gait.duty_factor = .5;
    gait.stride_phase_rate_hz = FrequencyHz{1};
    gait.swing_height_m = .03;
    BodyController controller, unseeded;
    RobotState measured{};
    auto feet = controller.update(measured, intent, gait, safety, BodyTwist{});
    // A loaded support target sits below the actual planted contact point.
    for (auto& foot : feet.feet) foot.pos_body_m.z = -.122;
    LegIK ik(defaultHexapodGeometry());
    measured.leg_states = ik.solve(measured, feet, safety).leg_states;
    measured.valid = true;
    measured.has_body_twist_state = true;
    measured.body_twist_state.body_trans_m.z = .14;
    measured.foot_contacts.fill(true);
    controller.update(measured, intent, gait, safety, BodyTwist{});
    intent.requested_mode = RobotMode::WALK;
    gait.in_stance.fill(false);
    gait.phase.fill(.75); // Mid-swing, full clearance.
    measured.foot_contacts.fill(false);
    auto lifted = controller.update(measured, intent, gait, safety, BodyTwist{});
    auto original = unseeded.update(measured, intent, gait, safety, BodyTwist{});
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const auto p = lifted.feet[leg].pos_body_m;
        const auto q = original.feet[leg].pos_body_m;
        if (std::abs(p.z + .14 - (.018 + .03)) > 1e-6
            || std::abs(p.x - q.x) > 1e-9 || std::abs(p.y - q.y) > 1e-9) {
            std::cerr << "contact-relative apex/unchanged XY failed, leg " << leg
                      << ": world z=" << p.z + .14 << '\n';
            return 1;
        }
    }
    for (const double phase : {.5, 1.0}) {
        gait.phase.fill(phase);
        lifted = controller.update(measured, intent, gait, safety, BodyTwist{});
        const auto endpoint = unseeded.update(measured, intent, gait, safety, BodyTwist{});
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (std::abs(lifted.feet[leg].pos_body_m.z - endpoint.feet[leg].pos_body_m.z) > 1e-9) {
                std::cerr << "correction must vanish at swing endpoints, phase=" << phase
                          << " leg=" << leg << " delta="
                          << lifted.feet[leg].pos_body_m.z - endpoint.feet[leg].pos_body_m.z << '\n';
                return 1;
            }
        }
    }
    gait.phase.fill(.75);
    // The reference is fixed in the world throughout swing, including chassis
    // motion. Airborne FK must not be mistaken for a new contact measurement.
    measured.body_twist_state.body_trans_m.z = .135;
    measured.body_twist_state.twist_pos_rad.x = .02;
    lifted = controller.update(measured, intent, gait, safety, BodyTwist{});
    for (const auto& foot : lifted.feet) {
        const double world_z = .135 + (Mat3::rotX(.02) * foot.pos_body_m).z;
        if (std::abs(world_z - .048) > 1e-6) {
            std::cerr << "swing reference moved with the chassis\n";
            return 1;
        }
    }
    measured.body_twist_state.body_trans_m.z = .14;
    measured.body_twist_state.twist_pos_rad.x = 0;
    // Reset must discard the old world's support reference.
    controller.reset();
    lifted = controller.update(measured, intent, gait, safety, BodyTwist{});
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (std::abs(lifted.feet[leg].pos_body_m.z - original.feet[leg].pos_body_m.z) > 1e-9) {
            std::cerr << "reset retained a stale contact height\n";
            return 1;
        }
    }
    intent.requested_mode = RobotMode::STAND;
    measured.foot_contacts.fill(true);
    controller.update(measured, intent, gait, safety, BodyTwist{});
    measured.valid = false;
    controller.update(measured, intent, gait, safety, BodyTwist{});
    measured.valid = true;
    measured.foot_contacts.fill(false);
    intent.requested_mode = RobotMode::WALK;
    lifted = controller.update(measured, intent, gait, safety, BodyTwist{});
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (std::abs(lifted.feet[leg].pos_body_m.z - original.feet[leg].pos_body_m.z) > 1e-9) {
            std::cerr << "invalid estimate retained a stale contact height\n";
            return 1;
        }
    }
    for (const bool invalid_pose : {true, false}) {
        BodyController fresh;
        intent.requested_mode = RobotMode::STAND;
        measured.foot_contacts.fill(true);
        fresh.update(measured, intent, gait, safety, BodyTwist{});
        const auto valid_measurement = measured;
        if (invalid_pose) {
            measured.body_twist_state.twist_pos_rad.z = std::numeric_limits<double>::quiet_NaN();
        } else {
            for (auto& leg : measured.leg_states)
                leg.joint_state[0].pos_rad = AngleRad{std::numeric_limits<double>::quiet_NaN()};
        }
        fresh.update(measured, intent, gait, safety, BodyTwist{});
        measured = valid_measurement;
        measured.foot_contacts.fill(false);
        intent.requested_mode = RobotMode::WALK;
        const auto after_invalid = fresh.update(measured, intent, gait, safety, BodyTwist{});
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (std::abs(after_invalid.feet[leg].pos_body_m.z - original.feet[leg].pos_body_m.z) > 1e-9) {
                std::cerr << "non-finite feedback retained a stale contact height\n";
                return 1;
            }
        }
    }
    std::cout << "measured-contact swing clearance and reset passed\n";
}
