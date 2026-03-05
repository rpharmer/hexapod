#include "robot_control.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

RobotControl::RobotControl(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator)
    : hw_(std::move(hw)), estimator_(std::move(estimator)) {}

RobotControl::~RobotControl() {
    stop();
}

bool RobotControl::init() {
    if (!hw_ || !estimator_) {
        std::cerr << "Missing components\n";
        return false;
    }
    if (!hw_->init()) {
        std::cerr << "Hardware init failed\n";
        return false;
    }

    //MotionIntent initial{};
    //initial.requested_mode = RobotMode::SAFE_IDLE;
    //initial.timestamp_us = now_us();

    //motion_intent_.write(initial);
    //status_.write(ControlStatus{});
    //safety_state_.write(SafetyState{});
    joint_targets_.write(JointTargets{});

    return true;
}

void RobotControl::start() {
    if (running_.exchange(true)) return;

    bus_thread_ = std::thread(&RobotControl::busLoop, this);
    estimator_thread_ = std::thread(&RobotControl::estimatorLoop, this);
    //control_thread_ = std::thread(&RobotControl::controlLoop, this);
    //safety_thread_ = std::thread(&RobotControl::safetyLoop, this);
    //diag_thread_ = std::thread(&RobotControl::diagnosticsLoop, this);
}

void RobotControl::stop() {
    if (!running_.exchange(false)) return;

    joinThread(bus_thread_);
    joinThread(estimator_thread_);
    //joinThread(control_thread_);
    //joinThread(safety_thread_);
    //joinThread(diag_thread_);
}

void RobotControl::setMotionIntent(const MotionIntent& intent) {
    motion_intent_.write(intent);
}

/*ControlStatus RobotControl::getStatus() const {
    return status_.read();
}*/

void RobotControl::busLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        RawHardwareState raw{};
        /*if (!hw_->read(raw)) {
            raw.bus_ok = false;
        }*/
        raw_state_.write(raw);

        const JointTargets cmd = joint_targets_.read();
        (void)hw_->write(cmd);

        sleepUntil(cycle_start, 2000us); // 500 Hz
    }
}

void RobotControl::estimatorLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const RawHardwareState raw = raw_state_.read();
        const EstimatedState est = estimator_->update(raw);
        estimated_state_.write(est);

        sleepUntil(cycle_start, 2000us); // 500 Hz
    }
}

void RobotControl::controlLoop() {
    //uint64_t loop_counter = 0;

    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const EstimatedState est = estimated_state_.read();
        /*const MotionIntent intent = motion_intent_.read();
        const SafetyState safety_state = safety_state_.read();

        RobotMode active_mode = intent.requested_mode;
        if (safety_state.active_fault != FaultCode::NONE) {
            active_mode = RobotMode::FAULT;
        }

        const GaitState gait_state = gait_.update(est, intent, safety_state);
        const LegTargets leg_targets = body_.update(est, intent, gait_state, safety_state);
        const JointTargets joint_targets = ik_.solve(est, leg_targets, safety_state);

        joint_targets_.write(joint_targets);

        ControlStatus st{};
        st.active_mode = active_mode;
        st.estimator_valid = est.valid;
        st.bus_ok = raw_state_.read().bus_ok;
        st.active_fault = safety_state.active_fault;
        st.loop_counter = ++loop_counter;
        status_.write(st);*/

        (void)est;

        sleepUntil(cycle_start, 4000us); // 250 Hz
    }
}

/*void RobotControl::safetyLoop() {
    while (running_.load()) {
        const auto cycle_start = Clock::now();

        const RawHardwareState raw = raw_state_.read();
        const EstimatedState est = estimated_state_.read();
        const MotionIntent intent = motion_intent_.read();

        const SafetyState s = safety_.evaluate(raw, est, intent);
        safety_state_.write(s);

        sleepUntil(cycle_start, 2000us); // 500 Hz
    }
}*/

/*void RobotControl::diagnosticsLoop() {
    while (running_.load()) {
        const auto st = status_.read();
        std::cout
            << "[diag] mode=" << toString(st.active_mode)
            << " est=" << (st.estimator_valid ? "ok" : "bad")
            << " bus=" << (st.bus_ok ? "ok" : "bad")
            << " fault=" << toString(st.active_fault)
            << " loops=" << st.loop_counter
            << "\n";

        std::this_thread::sleep_for(500ms);
    }
}*/

void RobotControl::joinThread(std::thread& t) {
    if (t.joinable()) t.join();
}

void RobotControl::sleepUntil(const Clock::time_point& start,
                              std::chrono::microseconds period) {
    std::this_thread::sleep_until(start + period);
}

/*const char* RobotControl::toString(RobotMode mode) {
    switch (mode) {
        case RobotMode::SAFE_IDLE: return "SAFE_IDLE";
        case RobotMode::HOMING: return "HOMING";
        case RobotMode::STAND: return "STAND";
        case RobotMode::WALK: return "WALK";
        case RobotMode::FAULT: return "FAULT";
    }
    return "UNKNOWN";
}*/

/*const char* RobotControl::toString(FaultCode code) {
    switch (code) {
        case FaultCode::NONE: return "NONE";
        case FaultCode::BUS_TIMEOUT: return "BUS_TIMEOUT";
        case FaultCode::ESTOP: return "ESTOP";
        case FaultCode::TIP_OVER: return "TIP_OVER";
        case FaultCode::ESTIMATOR_INVALID: return "ESTIMATOR_INVALID";
        case FaultCode::MOTOR_FAULT: return "MOTOR_FAULT";
        case FaultCode::JOINT_LIMIT: return "JOINT_LIMIT";
        case FaultCode::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
    }
    return "UNKNOWN";
}*/