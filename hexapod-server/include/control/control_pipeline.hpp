#pragma once

#include "body_controller.hpp"
#include "control_config.hpp"
#include "contact_manager.hpp"
#include "gait_policy_planner.hpp"
#include "gait_scheduler.hpp"
#include "leg_ik.hpp"
#include "motion_limiter.hpp"
#include "types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

enum class PipelineStage : std::size_t {
    Estimator = 0,
    Limiter = 1,
    Gait = 2,
    Body = 3,
    Ik = 4,
    Count = 5
};

struct StageTimingEnvelope {
    uint64_t sample_count{0};
    uint64_t p95_us{0};
    uint64_t p99_us{0};
    uint64_t budget_p95_us{0};
    uint64_t budget_p99_us{0};
    bool p95_within_budget{true};
    bool p99_within_budget{true};
};

struct PipelineTimingSnapshot {
    StageTimingEnvelope estimator{};
    StageTimingEnvelope limiter{};
    StageTimingEnvelope gait{};
    StageTimingEnvelope body{};
    StageTimingEnvelope ik{};
};

struct PipelineStepResult {
    LegTargets leg_targets{};
    JointTargets joint_targets{};
    ControlStatus status{};
};

class ControlPipeline {
public:
    explicit ControlPipeline(control_config::ControlConfig config = {});

    PipelineStepResult runStep(const RobotState& estimated,
                               const MotionIntent& intent,
                               const SafetyState& safety_state,
                               const DurationSec& loop_dt,
                               bool bus_ok,
                               uint64_t loop_counter);
    void recordStageDuration(PipelineStage stage, uint64_t duration_us);
    [[nodiscard]] PipelineTimingSnapshot timingSnapshot() const;

private:
    static constexpr std::size_t kMaxTimingWindow = 1024;
    struct StageTimingWindow {
        std::array<uint64_t, kMaxTimingWindow> samples_us{};
        std::size_t write_index{0};
        std::size_t sample_count{0};
    };

    [[nodiscard]] uint64_t stageBudgetP95Us(PipelineStage stage) const;
    [[nodiscard]] uint64_t stageBudgetP99Us(PipelineStage stage) const;
    [[nodiscard]] StageTimingEnvelope stageEnvelopeLocked(PipelineStage stage) const;
    [[nodiscard]] uint64_t percentileLocked(const StageTimingWindow& window, double quantile) const;

    control_config::ControlConfig config_{};
    GaitPolicyPlanner planner_;
    MotionLimiter motion_limiter_;
    GaitScheduler gait_;
    ContactManager contact_manager_;
    BodyController body_;
    LegIK ik_;
    std::size_t timing_window_size_{0};
    bool contact_manager_bypassed_{false};
    mutable std::mutex timing_mutex_{};
    std::array<StageTimingWindow, static_cast<std::size_t>(PipelineStage::Count)> stage_timings_{};
};
