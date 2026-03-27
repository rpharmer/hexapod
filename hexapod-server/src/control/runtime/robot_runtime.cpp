#include "robot_runtime.hpp"

#include "geometry_config.hpp"

#include <algorithm>
#include <cmath>

RobotRuntime::RobotRuntime(std::unique_ptr<IHardwareBridge> hw,
                           std::unique_ptr<IEstimator> estimator,
                           std::shared_ptr<logging::AsyncLogger> logger,
                           control_config::ControlConfig config,
                           std::unique_ptr<telemetry::ITelemetryPublisher> telemetry_publisher,
                           std::unique_ptr<hardware::IImuUnit> imu)
    : hw_(std::move(hw)),
      imu_(std::move(imu)),
      estimator_(std::move(estimator)),
      logger_(std::move(logger)),
      config_(config),
      telemetry_publisher_(std::move(telemetry_publisher)),
      pipeline_(config_.gait),
      safety_(config_.safety),
      freshness_policy_(config_.freshness),
      freshness_gate_(freshness_policy_),
      timing_metrics_(config_, control_dt_sum_us_, control_jitter_max_us_),
      diagnostics_reporter_(logger_, freshness_policy_) {}

namespace {

telemetry::TelemetryPublishCounters readTelemetryCounters(
    const std::unique_ptr<telemetry::ITelemetryPublisher>& telemetry_publisher)
{
    return telemetry_publisher ? telemetry_publisher->counters() : telemetry::TelemetryPublishCounters{};
}

constexpr double kJointTargetSlewLimitRadPerSec = 18.0;

uint8_t missionStateCode(autonomy::MissionState state) {
    return static_cast<uint8_t>(state);
}

} // namespace

bool RobotRuntime::init() {
    if (!hw_ || !estimator_) {
        if (logger_) {
            LOG_ERROR(logger_, "Missing components");
        }
        return false;
    }
    if (!hw_->init()) {
        if (logger_) {
            LOG_ERROR(logger_, "Hardware Bridge init failed");
        }
        return false;
    }
    if (!imu_) {
        imu_ = hardware::makeNoopImuUnit();
    }
    (void)imu_->init();
    diagnostics_reporter_.setRuntimeImuReadsEnabled(config_.runtime_imu.enable_reads);
    if (logger_) {
        LOG_INFO(logger_, "Runtime.Imu.EnableReads=", config_.runtime_imu.enable_reads);
    }
    if (!config_.runtime_imu.enable_reads && logger_) {
        LOG_WARN(logger_, "IMU reads disabled by Runtime.Imu.EnableReads=false");
    }

    if (!telemetry_publisher_) {
        telemetry_publisher_ = telemetry::makeNoopTelemetryPublisher();
    }

    MotionIntent initial{};
    initial.requested_mode = RobotMode::SAFE_IDLE;
    initial.timestamp_us = now_us();
    initial.sample_id = 1;

    motion_intent_.write(initial);
    status_.write(ControlStatus{});
    safety_state_.write(SafetyState{});
    joint_targets_.write(JointTargets{});
    control_loop_counter_.store(0);
    control_dt_sum_us_.store(0);
    control_jitter_max_us_.store(0);
    stale_intent_count_.store(0);
    stale_estimator_count_.store(0);
    raw_sample_seq_.store(0);
    intent_sample_seq_.store(1);
    has_last_written_joint_targets_ = false;
    last_written_joint_targets_ = JointTargets{};
    last_joint_write_timestamp_ = TimePointUs{};
    freshness_gate_.reset();
    timing_metrics_.reset();
    next_telemetry_publish_at_ = TimePointUs{};
    next_geometry_refresh_at_ = TimePointUs{};
    last_autonomy_step_output_.reset();
    last_autonomy_mission_event_ = autonomy::MissionEvent{};
    last_autonomy_recovery_decision_ = autonomy::RecoveryDecision{};
    autonomy_step_ok_ = false;
    autonomy_blocked_signal_.store(false);
    autonomy_no_progress_signal_.store(false);
    autonomy_waypoint_reached_event_.store(false);

    if (config_.autonomy.enabled) {
        autonomy_stack_ = std::make_unique<autonomy::AutonomyStack>(autonomy::AutonomyStackConfig{
            .no_progress_timeout_ms = config_.autonomy.no_progress_timeout_ms,
            .recovery_retry_budget = config_.autonomy.recovery_retry_budget,
            .traversability_policy = autonomy::TraversabilityPolicyConfig{
                .occupancy_risk_weight = config_.autonomy.traversability.occupancy_risk_weight,
                .gradient_risk_weight = config_.autonomy.traversability.gradient_risk_weight,
                .obstacle_near_risk_weight = config_.autonomy.traversability.obstacle_near_risk_weight,
                .obstacle_mid_risk_weight = config_.autonomy.traversability.obstacle_mid_risk_weight,
                .obstacle_far_risk_weight = config_.autonomy.traversability.obstacle_far_risk_weight,
                .slope_high_risk_weight = config_.autonomy.traversability.slope_high_risk_weight,
                .confidence_unknown_penalty = config_.autonomy.traversability.confidence_unknown_penalty,
                .confidence_cost_weight = config_.autonomy.traversability.confidence_cost_weight,
                .risk_block_threshold = config_.autonomy.traversability.risk_block_threshold,
                .confidence_block_threshold = config_.autonomy.traversability.confidence_block_threshold,
            },
        });
        if (!autonomy_stack_->init() || !autonomy_stack_->start()) {
            if (logger_) {
                LOG_ERROR(logger_, "AutonomyStack init/start failed");
            }
            autonomy_stack_.reset();
            return false;
        }
        if (logger_) {
            LOG_INFO(logger_,
                     "Runtime.Autonomy.Enabled=true, NoProgressTimeoutMs=",
                     config_.autonomy.no_progress_timeout_ms,
                     ", RecoveryRetryBudget=",
                     config_.autonomy.recovery_retry_budget);
        }
    } else {
        autonomy_stack_.reset();
    }

    return true;
}

void RobotRuntime::stop() {
    if (autonomy_stack_) {
        autonomy_stack_->stop();
    }
}

void RobotRuntime::startTelemetry() {
    if (!telemetry_publisher_ || !config_.telemetry.enabled) {
        return;
    }

    telemetry_publisher_->publishGeometry(geometry_config::activeHexapodGeometry());

    const TimePointUs now = now_us();
    next_telemetry_publish_at_ = now;
    next_geometry_refresh_at_ = TimePointUs{
        now.value + static_cast<uint64_t>(config_.telemetry.geometry_refresh_period.count()) * 1000ULL};
}

void RobotRuntime::busStep() {
    RobotState raw{};
    if (!hw_->read(raw)) {
        raw.bus_ok = false;
    }
    if (config_.runtime_imu.enable_reads) {
        hardware::ImuSample imu_sample{};
        if (imu_ && imu_->read(imu_sample) && imu_sample.valid) {
            raw.body_pose_state.orientation_rad = imu_sample.orientation_rad;
            raw.body_pose_state.angular_velocity_radps = imu_sample.angular_velocity_radps;
            raw.has_body_pose_state = true;
        }
    }
    if (raw.sample_id == 0) {
        raw.sample_id = raw_sample_seq_.fetch_add(1) + 1;
    }
    raw_state_.write(raw);

    JointTargets cmd = joint_targets_.read();
    const TimePointUs now = now_us();
    if (has_last_written_joint_targets_ && !last_joint_write_timestamp_.isZero() &&
        now.value > last_joint_write_timestamp_.value) {
        const double dt_s = static_cast<double>(now.value - last_joint_write_timestamp_.value) / 1'000'000.0;
        const double max_step = kJointTargetSlewLimitRadPerSec * dt_s;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double previous = last_written_joint_targets_.leg_states[leg].joint_state[joint].pos_rad.value;
                const double requested = cmd.leg_states[leg].joint_state[joint].pos_rad.value;
                const double delta = requested - previous;
                cmd.leg_states[leg].joint_state[joint].pos_rad.value =
                    previous + std::clamp(delta, -max_step, max_step);
            }
        }
    }
    (void)hw_->write(cmd);
    last_written_joint_targets_ = cmd;
    has_last_written_joint_targets_ = true;
    last_joint_write_timestamp_ = now;
}

void RobotRuntime::estimatorStep() {
    const RobotState raw = raw_state_.read();
    const RobotState est = estimator_->update(raw);
    estimated_state_.write(est);
}

void RobotRuntime::controlStep() {
    const TimePointUs now = now_us();
    timing_metrics_.update(now);

    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const SafetyState safety_state = safety_state_.read();
    const bool bus_ok = raw_state_.read().bus_ok;

    const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
        RuntimeFreshnessGate::EvaluationMode::StrictControl, now, est, intent);
    freshness_gate_.recordStrictMetrics(freshness, stale_intent_count_, stale_estimator_count_);

    const uint64_t loop_counter = control_loop_counter_.fetch_add(1) + 1;
    const RuntimeFreshnessGate::Decision decision =
        freshness_gate_.computeControlDecision(freshness, bus_ok, loop_counter);
    if (!decision.allow_pipeline) {
        RuntimeFreshnessGate::maybeLogReject(logger_, freshness, est, intent);
        ControlStatus runtime_status = decision.status;
        applyAutonomyStatus(runtime_status);
        status_.write(runtime_status);
        joint_targets_.write(decision.joint_targets);
        diagnostics_reporter_.recordControlOutputs(decision.joint_targets, runtime_status, now, nullptr);
        maybePublishTelemetry(now);
        return;
    }

    const MotionIntent effective_intent = resolveAutonomyMotionIntent(intent, safety_state, now);

    const DurationSec control_loop_dt{
        static_cast<double>(config_.loop_timing.control_loop_period.count()) / 1'000'000.0};
    const PipelineStepResult result = pipeline_.runStep(
        est,
        effective_intent,
        safety_state,
        control_loop_dt,
        bus_ok,
        loop_counter);

    joint_targets_.write(result.joint_targets);
    ControlStatus runtime_status = result.status;
    applyAutonomyStatus(runtime_status);
    status_.write(runtime_status);
    diagnostics_reporter_.recordControlOutputs(result.joint_targets, runtime_status, now, &result.leg_targets);

    maybePublishTelemetry(now);
}

void RobotRuntime::maybePublishTelemetry(const TimePointUs& now) {
    if (!telemetry_publisher_ || !config_.telemetry.enabled) {
        return;
    }

    if (now.value < next_telemetry_publish_at_.value) {
        return;
    }

    const uint64_t publish_period_us =
        static_cast<uint64_t>(config_.telemetry.publish_period.count()) * 1000ULL;
    next_telemetry_publish_at_ = TimePointUs{now.value + publish_period_us};

    if (now.value >= next_geometry_refresh_at_.value) {
        telemetry_publisher_->publishGeometry(geometry_config::activeHexapodGeometry());
        const uint64_t geometry_refresh_period_us =
            static_cast<uint64_t>(config_.telemetry.geometry_refresh_period.count()) * 1000ULL;
        next_geometry_refresh_at_ = TimePointUs{now.value + geometry_refresh_period_us};
    }

    telemetry::ControlStepTelemetry telemetry_sample{};
    telemetry_sample.estimated_state = estimated_state_.read();
    telemetry_sample.motion_intent = motion_intent_.read();
    telemetry_sample.joint_targets = joint_targets_.read();
    telemetry_sample.status = status_.read();
    telemetry_sample.timestamp_us = now;
    telemetry_sample.imu_reads_enabled = config_.runtime_imu.enable_reads;
    if (last_autonomy_step_output_.has_value()) {
        const auto& localization = last_autonomy_step_output_->localization_estimate;
        if (localization.valid) {
            telemetry_sample.autonomy_debug.localization_pose = telemetry::ControlStepTelemetry::AutonomyDebugPose{
                .x_m = localization.x_m,
                .y_m = localization.y_m,
                .yaw_rad = localization.yaw_rad,
                .frame_id = localization.frame_id,
            };
        }
    }
    telemetry_publisher_->publishControlStep(telemetry_sample);
}

void RobotRuntime::safetyStep() {
    const RobotState raw = raw_state_.read();
    const RobotState est = estimated_state_.read();
    const MotionIntent intent = motion_intent_.read();
    const TimePointUs now = now_us();
    const FreshnessPolicy::Evaluation freshness = freshness_gate_.evaluate(
        RuntimeFreshnessGate::EvaluationMode::SafetyLenient, now, est, intent);

    const SafetySupervisor::FreshnessInputs freshness_inputs{
        freshness.estimator.valid,
        freshness.intent.valid};
    const SafetyState s = safety_.evaluate(raw, est, intent, freshness_inputs);
    safety_state_.write(s);
}

void RobotRuntime::diagnosticsStep() {
    const auto st = status_.read();
    const auto bridge_result = hw_ ? hw_->last_bridge_result() : std::nullopt;
    const uint64_t loops = control_loop_counter_.load();
    const LoopTimingRollingMetrics loop_timing_metrics = timing_metrics_.rollingMetrics();
    diagnostics_reporter_.recordVisualizerTelemetry(readTelemetryCounters(telemetry_publisher_), now_us());
    diagnostics_reporter_.report(st,
                                 bridge_result,
                                 loops,
                                 timing_metrics_.averageControlDtUs(loops),
                                 control_jitter_max_us_.load(),
                                 loop_timing_metrics,
                                 stale_intent_count_.load(),
                                 stale_estimator_count_.load());
}

void RobotRuntime::setMotionIntent(const MotionIntent& intent) {
    MotionIntent stamped_intent = intent;
    if (stamped_intent.timestamp_us.isZero()) {
        stamped_intent.timestamp_us = now_us();
    }
    if (stamped_intent.sample_id == 0) {
        stamped_intent.sample_id = intent_sample_seq_.fetch_add(1) + 1;
    }
    motion_intent_.write(stamped_intent);
}

MotionIntent RobotRuntime::resolveAutonomyMotionIntent(const MotionIntent& base_intent,
                                                       const SafetyState& safety_state,
                                                       const TimePointUs& now) {
    last_autonomy_step_output_.reset();
    autonomy_step_ok_ = false;
    if (!autonomy_stack_) {
        return base_intent;
    }

    autonomy::AutonomyStepInput step_input{};
    step_input.now_ms = now.value / 1000ULL;
    step_input.estop = safety_state.torque_cut;
    step_input.hold = safety_state.inhibit_motion && !step_input.estop;
    step_input.blocked = autonomy_blocked_signal_.load() || autonomy_no_progress_signal_.load();
    step_input.waypoint_reached = autonomy_waypoint_reached_event_.exchange(false);
    step_input.map_slice_input = autonomy::MapSliceInput{
        .has_occupancy = true,
        .has_elevation = false,
        .has_risk_confidence = false,
        .occupancy = 0.0,
    };

    const RobotState estimated = estimated_state_.read();
    if (estimated.valid) {
        step_input.has_estimator_state = true;
        step_input.estimator_state = estimated;
    }

    autonomy::AutonomyStepOutput step_output{};
    if (!autonomy_stack_->step(step_input, &step_output)) {
        if (logger_) {
            LOG_WARN(logger_, "AutonomyStack step failed, using direct MotionIntent");
        }
        return base_intent;
    }
    autonomy_step_ok_ = true;
    last_autonomy_step_output_ = step_output;
    last_autonomy_mission_event_ = step_output.mission_event;
    last_autonomy_recovery_decision_ = step_output.recovery_decision;

    MotionIntent intent = base_intent;
    intent.timestamp_us = now;
    intent.sample_id = intent_sample_seq_.fetch_add(1) + 1;

    if (!step_output.motion_decision.allow_motion || !step_output.locomotion_command.sent) {
        intent.requested_mode = RobotMode::SAFE_IDLE;
        intent.speed_mps = LinearRateMps{0.0};
        return intent;
    }

    intent.requested_mode = RobotMode::WALK;
    const double target_x = step_output.locomotion_command.target.x_m;
    const double target_y = step_output.locomotion_command.target.y_m;
    intent.heading_rad = AngleRad{std::atan2(target_y, target_x)};
    const double distance = std::sqrt((target_x * target_x) + (target_y * target_y));
    const double capped_speed = std::min(distance, config_.gait.fallback_speed_mag.value);
    intent.speed_mps = LinearRateMps{capped_speed};
    return intent;
}

void RobotRuntime::applyAutonomyStatus(ControlStatus& status) const {
    if (!autonomy_stack_) {
        status.autonomy = ControlStatus::AutonomyTelemetry{};
        return;
    }

    status.autonomy.enabled = true;
    status.autonomy.step_ok = autonomy_step_ok_;
    status.autonomy.blocked = autonomy_blocked_signal_.load();
    status.autonomy.no_progress = autonomy_no_progress_signal_.load();
    status.autonomy.recovery_active = last_autonomy_recovery_decision_.recovery_active;
    status.autonomy.motion_allowed =
        last_autonomy_step_output_.has_value() && last_autonomy_step_output_->motion_decision.allow_motion;
    status.autonomy.locomotion_sent =
        last_autonomy_step_output_.has_value() && last_autonomy_step_output_->locomotion_command.sent;
    status.autonomy.mission_state = missionStateCode(last_autonomy_mission_event_.state);
    status.autonomy.mission_completed_waypoints = last_autonomy_mission_event_.progress.completed_waypoints;
    status.autonomy.mission_total_waypoints = last_autonomy_mission_event_.progress.total_waypoints;
    status.autonomy.mission_loaded = !last_autonomy_mission_event_.progress.mission_id.empty() &&
                                     last_autonomy_mission_event_.state != autonomy::MissionState::Idle;
    status.autonomy.mission_running = last_autonomy_mission_event_.state == autonomy::MissionState::Exec ||
                                      last_autonomy_mission_event_.state == autonomy::MissionState::Paused;
}

bool RobotRuntime::loadAutonomyMission(const autonomy::WaypointMission& mission) {
    if (!autonomy_stack_) {
        return false;
    }

    const auto event = autonomy_stack_->loadMission(mission);
    last_autonomy_mission_event_ = event;
    last_autonomy_recovery_decision_ = autonomy::RecoveryDecision{};
    return event.accepted;
}

bool RobotRuntime::startAutonomyMission() {
    if (!autonomy_stack_) {
        return false;
    }

    const auto event = autonomy_stack_->startMission();
    last_autonomy_mission_event_ = event;
    return event.accepted;
}

void RobotRuntime::setAutonomyBlocked(bool blocked) {
    autonomy_blocked_signal_.store(blocked);
}

void RobotRuntime::setAutonomyNoProgress(bool no_progress) {
    autonomy_no_progress_signal_.store(no_progress);
}

void RobotRuntime::signalAutonomyWaypointReached() {
    autonomy_waypoint_reached_event_.store(true);
}

std::optional<autonomy::AutonomyStepOutput> RobotRuntime::lastAutonomyStepOutput() const {
    return last_autonomy_step_output_;
}

void RobotRuntime::setMotionIntentForTest(const MotionIntent& intent) {
    motion_intent_.write(intent);
}

bool RobotRuntime::setSimFaultToggles(const SimHardwareFaultToggles& toggles) {
    auto* sim_hw = dynamic_cast<SimHardwareBridge*>(hw_.get());
    if (sim_hw == nullptr) {
        return false;
    }

    sim_hw->setFaultToggles(toggles);
    return true;
}

ControlStatus RobotRuntime::getStatus() const {
    return status_.read();
}

void RobotRuntime::setAutonomyBlockedForTest(bool blocked) {
    setAutonomyBlocked(blocked);
}

void RobotRuntime::setAutonomyNoProgressForTest(bool no_progress) {
    setAutonomyNoProgress(no_progress);
}

void RobotRuntime::signalAutonomyWaypointReachedForTest() {
    signalAutonomyWaypointReached();
}

bool RobotRuntime::loadAutonomyMissionForTest(const autonomy::WaypointMission& mission) {
    return loadAutonomyMission(mission);
}

bool RobotRuntime::startAutonomyMissionForTest() {
    return startAutonomyMission();
}

std::optional<autonomy::AutonomyStepOutput> RobotRuntime::lastAutonomyStepOutputForTest() const {
    return lastAutonomyStepOutput();
}
