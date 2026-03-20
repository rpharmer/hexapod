#include "sim_hardware_bridge.hpp"

#include <cmath>

SimHardwareBridge::SimHardwareBridge(SimHardwareFaultToggles fault_toggles,
                                     DurationSec read_cycle_period,
                                     DurationSec response_time_constant)
    : fault_toggles_(fault_toggles),
      read_cycle_period_(read_cycle_period),
      response_time_constant_(response_time_constant) {}

bool SimHardwareBridge::init() {
    state_ = RawHardwareState{};
    commanded_targets_ = JointTargets{};

    state_.voltage = fault_toggles_.low_voltage ? fault_toggles_.low_voltage_value
                                                : fault_toggles_.nominal_voltage;
    state_.current = fault_toggles_.high_current ? fault_toggles_.high_current_value
                                                 : fault_toggles_.nominal_current;
    state_.bus_ok = !fault_toggles_.drop_bus;
    if (fault_toggles_.forced_contacts.has_value()) {
        state_.foot_contacts = *fault_toggles_.forced_contacts;
    }

    state_.timestamp_us = now_us();
    initialized_ = true;
    return true;
}

bool SimHardwareBridge::read(RawHardwareState& out) {
    if (!initialized_) {
        return false;
    }

    const double tau_s = response_time_constant_.value;
    const double dt_s = read_cycle_period_.value;
    const double alpha = (tau_s <= 0.0) ? 1.0 : clamp01(1.0 - std::exp(-dt_s / tau_s));

    for (int leg = 0; leg < kNumLegs; ++leg) {
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            AngleRad& simulated = state_.leg_states[leg].joint_raw_state[joint].pos_rad;
            const AngleRad target = commanded_targets_.leg_raw_states[leg].joint_raw_state[joint].pos_rad;
            simulated.value += alpha * (target.value - simulated.value);
        }
    }

    state_.bus_ok = !fault_toggles_.drop_bus;
    state_.voltage = fault_toggles_.low_voltage ? fault_toggles_.low_voltage_value
                                                : fault_toggles_.nominal_voltage;
    state_.current = fault_toggles_.high_current ? fault_toggles_.high_current_value
                                                 : fault_toggles_.nominal_current;
    if (fault_toggles_.forced_contacts.has_value()) {
        state_.foot_contacts = *fault_toggles_.forced_contacts;
    }

    state_.timestamp_us = now_us();
    out = state_;
    return true;
}

bool SimHardwareBridge::write(const JointTargets& in) {
    if (!initialized_) {
        return false;
    }

    commanded_targets_ = in;
    return !fault_toggles_.drop_bus;
}

double SimHardwareBridge::clamp01(double value) {
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}
