#pragma once

#include "process_resource_monitoring.hpp"

#include <array>
#include <cstddef>

namespace runtime_resource_monitoring {

inline constexpr std::size_t kMaxSections = 18;
inline constexpr std::size_t kTopSectionsToReport = 8;

enum class Section : std::size_t {
    BusRead = 0,
    BusWrite,
    EstimatorUpdate,
    ControlStep,
    ControlResolveIntent,
    ControlFusionConsistency,
    ControlFreshnessGate,
    ControlPipeline,
    ControlPipelineLocoCommand,
    ControlPipelineGait,
    ControlPipelineStability,
    ControlPipelineBody,
    ControlPipelineIK,
    TelemetryPublish,
    ReplayWrite,
    SafetyEvaluate,
    DiagnosticsSampleResources,
    DiagnosticsReport,
    Count,
};

static_assert(static_cast<std::size_t>(Section::Count) <= kMaxSections,
              "runtime resource labels must fit in the fixed-size profiler");

using Profiler = resource_monitoring::SectionProfiler<kMaxSections>;
using SectionSummary = resource_monitoring::ResourceSectionSummary<kMaxSections>;

inline constexpr std::array<const char*, kMaxSections> kSectionLabels = {
    "bus.read",
    "bus.write",
    "estimator.update",
    "control.step",
    "control.resolve_intent",
    "control.fusion_consistency",
    "control.freshness_gate",
    "control.pipeline",
    "control.pipeline.loco_cmd",
    "control.pipeline.gait",
    "control.pipeline.stability",
    "control.pipeline.body",
    "control.pipeline.ik",
    "control.telemetry_publish",
    "control.replay_write",
    "safety.evaluate",
    "diagnostics.sample_resources",
    "diagnostics.report",
};

inline constexpr std::size_t toIndex(Section section) {
    return static_cast<std::size_t>(section);
}

} // namespace runtime_resource_monitoring
