#include "contact_manager.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

int g_failures = 0;

bool expect(const bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << '\n';
        ++g_failures;
        return false;
    }
    return true;
}

RuntimeGaitPolicy makePolicy() {
    RuntimeGaitPolicy policy{};
    policy.cadence_hz = FrequencyHz{1.0};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        policy.per_leg[leg].duty_cycle = 0.6;
        policy.per_leg[leg].step_length_m = LengthM{0.08};
        policy.per_leg[leg].swing_height_m = LengthM{0.03};
    }
    return policy;
}

RobotState makeStateWithContactMask(const std::array<bool, kNumLegs>& contacts) {
    RobotState state{};
    state.foot_contacts = contacts;
    return state;
}

GaitState makeScheduledGait(const std::array<bool, kNumLegs>& in_stance, const double phase = 0.55) {
    GaitState gait{};
    gait.in_stance = in_stance;
    gait.phase.fill(phase);
    return gait;
}

void expectFiniteAndBounded(const ContactManagerOutput& output,
                            const ContactManagerOutput& previous,
                            const std::string& label) {
    const double cadence_rate = std::abs(output.managed_policy.cadence_hz.value - previous.managed_policy.cadence_hz.value);
    expect(std::isfinite(output.managed_policy.cadence_hz.value), label + ": cadence must be finite");
    expect(output.managed_policy.cadence_hz.value >= 0.0, label + ": cadence should not be negative");
    expect(cadence_rate <= 0.25, label + ": cadence update rate should remain bounded");

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double step = output.managed_policy.per_leg[leg].step_length_m.value;
        const double swing = output.managed_policy.per_leg[leg].swing_height_m.value;
        const double stance_conf = output.stance_confidence[leg];
        const double step_rate = std::abs(step - previous.managed_policy.per_leg[leg].step_length_m.value);
        const double swing_rate = std::abs(swing - previous.managed_policy.per_leg[leg].swing_height_m.value);

        expect(std::isfinite(step), label + ": step length must be finite for leg " + std::to_string(leg));
        expect(std::isfinite(swing), label + ": swing height must be finite for leg " + std::to_string(leg));
        expect(std::isfinite(stance_conf), label + ": stance confidence must be finite for leg " + std::to_string(leg));

        expect(step >= 0.02 && step <= 0.08, label + ": step length should stay within controller limits for leg " + std::to_string(leg));
        expect(swing >= 0.0 && swing <= 0.08, label + ": swing height should stay within controller limits for leg " + std::to_string(leg));
        expect(stance_conf >= 0.0 && stance_conf <= 1.0, label + ": stance confidence should stay normalized for leg " + std::to_string(leg));

        expect(step_rate <= 0.03, label + ": step-length update rate should remain bounded for leg " + std::to_string(leg));
        expect(swing_rate <= 0.011, label + ": swing-height update rate should remain bounded for leg " + std::to_string(leg));
    }
}

void testTouchdownAndDerating() {
    ContactManager manager;
    RuntimeGaitPolicy policy = makePolicy();
    GaitState scheduled = makeScheduledGait({false, false, false, false, false, false}, 0.8);
    RobotState state = makeStateWithContactMask({false, false, false, false, false, false});

    (void)manager.update(state, scheduled, policy);
    state.foot_contacts[0] = true;
    const ContactManagerOutput touchdown = manager.update(state, scheduled, policy);

    expect(touchdown.touchdown_detected[0], "contact rise in swing should trigger touchdown detection");
    expect(touchdown.managed_gait.in_stance[0], "touchdown should complete swing and force stance");

    scheduled = makeScheduledGait({true, true, true, true, true, true}, 0.55);
    state = makeStateWithContactMask({false, false, true, true, true, true});
    ContactManagerOutput degraded = manager.update(state, scheduled, policy);
    degraded = manager.update(state, scheduled, degraded.managed_policy);
    degraded = manager.update(state, scheduled, degraded.managed_policy);

    expect(degraded.derating_requested, "persistent stance-contact loss should request gait derating");
    expect(degraded.managed_policy.cadence_hz.value < policy.cadence_hz.value, "derating should reduce cadence");
    expect(degraded.managed_policy.per_leg[0].swing_height_m.value > policy.per_leg[0].swing_height_m.value,
           "missed touchdown should activate foot-search swing lift");
}

void testSingleLegDropoutTimeline() {
    ContactManager manager;
    RuntimeGaitPolicy policy = makePolicy();

    GaitState scheduled = makeScheduledGait({true, true, true, true, true, true}, 0.55);
    ContactManagerOutput prev{};
    prev.managed_policy = policy;

    for (int t = 0; t < 12; ++t) {
        std::array<bool, kNumLegs> contacts = {true, true, true, true, true, true};
        if (t >= 3 && t <= 8) {
            contacts[2] = false;
        }
        const ContactManagerOutput out = manager.update(makeStateWithContactMask(contacts), scheduled, policy);
        expectFiniteAndBounded(out, prev, "single-leg-dropout t=" + std::to_string(t));

        if (t >= 3 && t <= 8) {
            expect(out.slip_detected[2], "single-leg dropout should flag slip on the dropped leg");
            expect(!out.derating_requested, "single-leg dropout alone should not trigger global derating");
        }
        prev = out;
        policy = out.managed_policy;
    }
}

void testAlternatingDropoutsTimeline() {
    ContactManager manager;
    RuntimeGaitPolicy policy = makePolicy();
    const GaitState scheduled = makeScheduledGait({true, true, true, true, true, true}, 0.55);

    ContactManagerOutput prev{};
    prev.managed_policy = policy;
    bool saw_derating = false;

    for (int t = 0; t < 10; ++t) {
        std::array<bool, kNumLegs> contacts = {true, true, true, true, true, true};
        const bool drop_group_a = (t % 2) == 0;
        contacts[0] = !drop_group_a;
        contacts[3] = !drop_group_a;
        contacts[1] = drop_group_a;
        contacts[4] = drop_group_a;

        const ContactManagerOutput out = manager.update(makeStateWithContactMask(contacts), scheduled, policy);
        expectFiniteAndBounded(out, prev, "alternating-dropout t=" + std::to_string(t));
        saw_derating = saw_derating || out.derating_requested;

        expect(out.slip_detected[0] || out.slip_detected[1] || out.slip_detected[3] || out.slip_detected[4],
               "alternating dropouts should keep slip flags active on one leg group");
        prev = out;
        policy = out.managed_policy;
    }

    expect(saw_derating, "alternating multi-leg dropouts should eventually engage gait derating fallback");
}

void testShortAllFalseBurstTimeline() {
    ContactManager manager;
    RuntimeGaitPolicy policy = makePolicy();
    const GaitState scheduled = makeScheduledGait({true, true, true, true, true, true}, 0.55);

    ContactManagerOutput prev{};
    prev.managed_policy = policy;
    bool derating_on_burst = false;
    bool derating_cleared_after_recovery = false;

    for (int t = 0; t < 16; ++t) {
        std::array<bool, kNumLegs> contacts = {true, true, true, true, true, true};
        if (t == 4 || t == 5) {
            contacts.fill(false);
        }
        const ContactManagerOutput out = manager.update(makeStateWithContactMask(contacts), scheduled, policy);
        expectFiniteAndBounded(out, prev, "all-false-burst t=" + std::to_string(t));

        if (t == 4 || t == 5) {
            derating_on_burst = derating_on_burst || out.derating_requested;
        }
        if (t >= 12 && !out.derating_requested) {
            derating_cleared_after_recovery = true;
        }

        prev = out;
        policy = out.managed_policy;
    }

    expect(derating_on_burst, "short all-false burst should trigger fallback derating/suppression");
    expect(derating_cleared_after_recovery,
           "fallback derating should clear after the healthy-contact recovery window");
}

void testDeratingRecoveryAfterHealthyWindow() {
    ContactManager manager;
    RuntimeGaitPolicy policy = makePolicy();
    const GaitState scheduled = makeScheduledGait({true, true, true, true, true, true}, 0.55);

    bool saw_derating = false;
    bool recovered = false;

    for (int t = 0; t < 20; ++t) {
        std::array<bool, kNumLegs> contacts = {true, true, true, true, true, true};
        if (t >= 2 && t <= 4) {
            contacts[0] = false;
            contacts[1] = false;
        }

        const ContactManagerOutput out = manager.update(makeStateWithContactMask(contacts), scheduled, policy);
        if (t >= 2 && t <= 4) {
            saw_derating = saw_derating || out.derating_requested;
        }
        if (t >= 12 && !out.derating_requested) {
            recovered = true;
        }
        policy = out.managed_policy;
    }

    expect(saw_derating, "multi-leg stance loss should engage derating before recovery");
    expect(recovered, "derating should clear after sustained healthy contact");
}

} // namespace

int main() {
    testTouchdownAndDerating();
    testSingleLegDropoutTimeline();
    testAlternatingDropoutsTimeline();
    testShortAllFalseBurstTimeline();
    testDeratingRecoveryAfterHealthyWindow();

    if (g_failures != 0) {
        std::cerr << g_failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
