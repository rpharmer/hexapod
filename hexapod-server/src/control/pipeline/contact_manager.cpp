#include "contact_manager.hpp"

#include <algorithm>

namespace {

double swingProgress(const GaitState& gait,
                     const RuntimeGaitPolicy& policy,
                     int leg) {
    const double duty = std::clamp(policy.per_leg[leg].duty_cycle, 0.05, 0.95);
    const double phase = clamp01(gait.phase[leg]);
    return clamp01((phase - duty) / std::max(1.0 - duty, 1e-6));
}

} // namespace

ContactManagerOutput ContactManager::update(const RobotState& estimated,
                                            const GaitState& scheduled_gait,
                                            const RuntimeGaitPolicy& policy) {
    ContactManagerOutput output{};
    output.managed_gait = scheduled_gait;
    output.managed_policy = policy;

    int degraded_legs = 0;

    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegContactState& leg_state = leg_states_[leg];
        const bool contact = estimated.foot_contacts[leg];
        const bool scheduled_stance = scheduled_gait.in_stance[leg];
        const double swing_alpha = swingProgress(scheduled_gait, policy, leg);

        const bool touchdown_event =
            (!scheduled_stance) &&
            (!leg_state.previous_contact) &&
            contact &&
            (swing_alpha >= 0.20);

        if (touchdown_event) {
            leg_state.touchdown_latched = true;
            leg_state.missed_touchdown_count = 0;
        }

        const bool slip_event = scheduled_stance && (!contact);
        if (slip_event) {
            ++leg_state.slip_count;
            leg_state.stance_confidence = std::max(0.0, leg_state.stance_confidence - 0.20);
        } else if (scheduled_stance && contact) {
            leg_state.stance_confidence = std::min(1.0, leg_state.stance_confidence + 0.10);
        } else {
            leg_state.stance_confidence = std::max(0.0, leg_state.stance_confidence - 0.05);
        }

        const bool missed_touchdown =
            scheduled_stance &&
            (!contact) &&
            leg_state.previous_scheduled_stance;
        if (missed_touchdown) {
            ++leg_state.missed_touchdown_count;
        } else if (contact) {
            leg_state.missed_touchdown_count = 0;
        }

        const bool touchdown_override =
            leg_state.touchdown_latched &&
            (!scheduled_stance);
        if (touchdown_override) {
            output.managed_gait.in_stance[leg] = true;
            leg_state.touchdown_latched = false;
        }

        const bool foot_search_active = leg_state.missed_touchdown_count >= 2;
        if (foot_search_active) {
            output.managed_policy.per_leg[leg].swing_height_m =
                LengthM{std::min(0.08, output.managed_policy.per_leg[leg].swing_height_m.value + 0.01)};
            output.managed_policy.per_leg[leg].step_length_m =
                LengthM{std::max(0.02, output.managed_policy.per_leg[leg].step_length_m.value * 0.9)};
        }

        if ((leg_state.slip_count >= 2) || foot_search_active || (leg_state.stance_confidence < 0.4)) {
            ++degraded_legs;
        }

        output.stance_confidence[leg] = leg_state.stance_confidence;
        output.touchdown_detected[leg] = touchdown_event;
        output.slip_detected[leg] = slip_event;

        leg_state.previous_contact = contact;
        leg_state.previous_scheduled_stance = scheduled_stance;
    }

    if (degraded_legs >= 2) {
        output.derating_requested = true;
        output.managed_policy.cadence_hz =
            FrequencyHz{std::max(0.15, output.managed_policy.cadence_hz.value * 0.80)};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            output.managed_policy.per_leg[leg].step_length_m =
                LengthM{std::max(0.02, output.managed_policy.per_leg[leg].step_length_m.value * 0.85)};
        }
    }

    return output;
}
