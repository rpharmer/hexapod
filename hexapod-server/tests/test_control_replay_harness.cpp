#include "control_pipeline.hpp"
#include "estimator.hpp"
#include "safety_supervisor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>
#include <vector>

namespace {

struct ReplayFrame {
    RobotState raw{};
    MotionIntent intent{};
    DurationSec dt{0.02};
    bool bus_ok{true};
    SafetySupervisor::FreshnessInputs freshness{};
};

struct OutputEnvelope {
    double min_joint_rad{std::numeric_limits<double>::infinity()};
    double max_joint_rad{-std::numeric_limits<double>::infinity()};
    double max_envelope_speed_normalized{0.0};
    double max_envelope_yaw_normalized{0.0};
    double max_envelope_roll_pitch_rad{0.0};
    std::size_t walk_steps{0};
    std::size_t safe_idle_steps{0};
    std::size_t fault_steps{0};
    std::size_t inhibited_steps{0};
    std::size_t torque_cut_steps{0};
    FaultCode final_fault{FaultCode::NONE};
};

struct ReplayScenario {
    std::string_view name;
    std::vector<ReplayFrame> frames;
    OutputEnvelope golden;
};

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool near(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <= tolerance;
}

RobotState makeRaw(uint64_t timestamp_us,
                   uint64_t sample_id,
                   bool bus_ok,
                   bool contacts = true,
                   float voltage = 12.4f,
                   float current = 1.2f) {
    RobotState raw{};
    raw.timestamp_us = TimePointUs{timestamp_us};
    raw.sample_id = sample_id;
    raw.bus_ok = bus_ok;
    raw.valid = true;
    raw.voltage = voltage;
    raw.current = current;
    raw.foot_contacts.fill(contacts);
    raw.has_valid_flag = true;
    raw.has_power_state = true;
    return raw;
}

MotionIntent makeIntent(uint64_t timestamp_us,
                        uint64_t sample_id,
                        RobotMode mode,
                        double speed_mps,
                        double heading_rad) {
    MotionIntent intent{};
    intent.timestamp_us = TimePointUs{timestamp_us};
    intent.sample_id = sample_id;
    intent.requested_mode = mode;
    intent.speed_mps = LinearRateMps{speed_mps};
    intent.heading_rad = AngleRad{heading_rad};
    return intent;
}

OutputEnvelope replayScenario(const ReplayScenario& scenario) {
    SimpleEstimator estimator;
    SafetySupervisor safety;
    ControlPipeline pipeline;

    OutputEnvelope envelope{};
    uint64_t loop_counter = 1;

    for (const ReplayFrame& frame : scenario.frames) {
        const RobotState est = estimator.update(frame.raw);
        const SafetyState safety_state = safety.evaluate(frame.raw, est, frame.intent, frame.freshness);
        const PipelineStepResult result = pipeline.runStep(
            est,
            frame.intent,
            safety_state,
            frame.dt,
            frame.bus_ok,
            loop_counter++);

        for (const auto& leg : result.joint_targets.leg_states) {
            for (const auto& joint : leg.joint_state) {
                envelope.min_joint_rad = std::min(envelope.min_joint_rad, joint.pos_rad.value);
                envelope.max_joint_rad = std::max(envelope.max_joint_rad, joint.pos_rad.value);
            }
        }

        envelope.max_envelope_speed_normalized =
            std::max(envelope.max_envelope_speed_normalized,
                     result.status.dynamic_gait.envelope_max_speed_normalized);
        envelope.max_envelope_yaw_normalized =
            std::max(envelope.max_envelope_yaw_normalized,
                     result.status.dynamic_gait.envelope_max_yaw_normalized);
        envelope.max_envelope_roll_pitch_rad =
            std::max(envelope.max_envelope_roll_pitch_rad,
                     result.status.dynamic_gait.envelope_max_roll_pitch_rad);

        switch (result.status.active_mode) {
            case RobotMode::WALK:
                ++envelope.walk_steps;
                break;
            case RobotMode::SAFE_IDLE:
                ++envelope.safe_idle_steps;
                break;
            case RobotMode::FAULT:
                ++envelope.fault_steps;
                break;
            default:
                break;
        }
        if (safety_state.inhibit_motion) {
            ++envelope.inhibited_steps;
        }
        if (safety_state.torque_cut) {
            ++envelope.torque_cut_steps;
        }
        envelope.final_fault = safety_state.active_fault;
    }

    return envelope;
}

bool compareEnvelope(const ReplayScenario& scenario,
                    const OutputEnvelope& actual,
                    const OutputEnvelope& golden) {
    constexpr double kJointTolerance = 1e-5;
    constexpr double kEnvelopeTolerance = 1e-6;

    const bool ok = expect(near(actual.min_joint_rad, golden.min_joint_rad, kJointTolerance),
                  std::string(scenario.name) + ": min joint envelope mismatch") &&
           expect(near(actual.max_joint_rad, golden.max_joint_rad, kJointTolerance),
                  std::string(scenario.name) + ": max joint envelope mismatch") &&
           expect(near(actual.max_envelope_speed_normalized,
                       golden.max_envelope_speed_normalized,
                       kEnvelopeTolerance),
                  std::string(scenario.name) + ": speed envelope mismatch") &&
           expect(near(actual.max_envelope_yaw_normalized,
                       golden.max_envelope_yaw_normalized,
                       kEnvelopeTolerance),
                  std::string(scenario.name) + ": yaw envelope mismatch") &&
           expect(near(actual.max_envelope_roll_pitch_rad,
                       golden.max_envelope_roll_pitch_rad,
                       kEnvelopeTolerance),
                  std::string(scenario.name) + ": roll/pitch envelope mismatch") &&
           expect(actual.walk_steps == golden.walk_steps,
                  std::string(scenario.name) + ": walk step count mismatch") &&
           expect(actual.safe_idle_steps == golden.safe_idle_steps,
                  std::string(scenario.name) + ": safe-idle step count mismatch") &&
           expect(actual.fault_steps == golden.fault_steps,
                  std::string(scenario.name) + ": fault step count mismatch") &&
           expect(actual.inhibited_steps == golden.inhibited_steps,
                  std::string(scenario.name) + ": inhibited step count mismatch") &&
           expect(actual.torque_cut_steps == golden.torque_cut_steps,
                  std::string(scenario.name) + ": torque-cut step count mismatch") &&
           expect(actual.final_fault == golden.final_fault,
                  std::string(scenario.name) + ": final fault mismatch");
    if (!ok) {
        std::cerr << "actual envelope for " << scenario.name
                  << ": min_joint=" << actual.min_joint_rad
                  << ", max_joint=" << actual.max_joint_rad
                  << ", speed_max=" << actual.max_envelope_speed_normalized
                  << ", yaw_max=" << actual.max_envelope_yaw_normalized
                  << ", roll_pitch_max=" << actual.max_envelope_roll_pitch_rad
                  << ", walk=" << actual.walk_steps
                  << ", safe_idle=" << actual.safe_idle_steps
                  << ", fault=" << actual.fault_steps
                  << ", inhibited=" << actual.inhibited_steps
                  << ", torque_cut=" << actual.torque_cut_steps
                  << ", final_fault=" << static_cast<int>(actual.final_fault)
                  << '\n';
    }
    return ok;
}

std::vector<ReplayScenario> makeScenarios() {
    std::vector<ReplayScenario> scenarios;

    {
        ReplayScenario nominal{};
        nominal.name = "nominal_walk_replay";
        nominal.frames = {
            ReplayFrame{makeRaw(1'000'000, 1, true, true),
                        makeIntent(1'000'000, 1, RobotMode::WALK, 0.15, 0.0),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(1'020'000, 2, true, true),
                        makeIntent(1'020'000, 2, RobotMode::WALK, 0.20, 0.10),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(1'040'000, 3, true, true),
                        makeIntent(1'040'000, 3, RobotMode::WALK, 0.22, -0.12),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(1'060'000, 4, true, true),
                        makeIntent(1'060'000, 4, RobotMode::WALK, 0.18, 0.05),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
        };
        nominal.golden = OutputEnvelope{
            .min_joint_rad = -1.4486232791552935,
            .max_joint_rad = 1.4486232791552935,
            .max_envelope_speed_normalized = 1.0,
            .max_envelope_yaw_normalized = 1.0,
            .max_envelope_roll_pitch_rad = 0.13962634015954636,
            .walk_steps = 4,
            .safe_idle_steps = 0,
            .fault_steps = 0,
            .inhibited_steps = 0,
            .torque_cut_steps = 0,
            .final_fault = FaultCode::NONE,
        };
        scenarios.push_back(std::move(nominal));
    }

    {
        ReplayScenario fault_recovery{};
        fault_recovery.name = "bus_fault_and_safe_idle_recovery_replay";
        fault_recovery.frames = {
            ReplayFrame{makeRaw(2'000'000, 10, true, true),
                        makeIntent(2'000'000, 10, RobotMode::WALK, 0.12, 0.0),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(2'020'000, 11, false, true),
                        makeIntent(2'020'000, 11, RobotMode::WALK, 0.12, 0.0),
                        DurationSec{0.02},
                        false,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(2'040'000, 12, false, true),
                        makeIntent(2'040'000, 12, RobotMode::WALK, 0.10, 0.0),
                        DurationSec{0.02},
                        false,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(2'060'000, 13, true, true),
                        makeIntent(2'060'000, 13, RobotMode::SAFE_IDLE, 0.0, 0.0),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(2'080'000, 14, true, true),
                        makeIntent(2'080'000, 14, RobotMode::SAFE_IDLE, 0.0, 0.0),
                        DurationSec{0.52},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
            ReplayFrame{makeRaw(2'600'000, 15, true, true),
                        makeIntent(2'600'000, 15, RobotMode::WALK, 0.10, 0.0),
                        DurationSec{0.02},
                        true,
                        SafetySupervisor::FreshnessInputs{true, true}},
        };
        fault_recovery.golden = OutputEnvelope{
            .min_joint_rad = -1.4486232791552935,
            .max_joint_rad = 1.4486232791552935,
            .max_envelope_speed_normalized = 1.0,
            .max_envelope_yaw_normalized = 1.0,
            .max_envelope_roll_pitch_rad = 0.13962634015954636,
            .walk_steps = 1,
            .safe_idle_steps = 0,
            .fault_steps = 5,
            .inhibited_steps = 5,
            .torque_cut_steps = 5,
            .final_fault = FaultCode::TIP_OVER,
        };
        scenarios.push_back(std::move(fault_recovery));
    }

    return scenarios;
}

} // namespace

int main() {
    std::vector<ReplayScenario> scenarios = makeScenarios();

    for (const ReplayScenario& scenario : scenarios) {
        const OutputEnvelope actual = replayScenario(scenario);
        if (!compareEnvelope(scenario, actual, scenario.golden)) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
