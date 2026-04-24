#pragma once

#include "process_resource_monitoring.hpp"

#include <array>
#include <cstddef>

namespace minphys3d::world_resource_monitoring {

inline constexpr std::size_t kMaxSections = 18;
inline constexpr std::size_t kTopSectionsToReport = 8;

enum class Section : std::size_t {
    Step = 0,
    IntegrateForces,
    UpdateBroadphaseProxies,
    GenerateContacts,
    BuildManifolds,
    BuildIslands,
    WarmStartContacts,
    WarmStartJoints,
    SolveIslands,
    RelaxationPass,
    ApplySplitStabilization,
    ResolveTOI,
    IntegrateOrientation,
    SolveJointPositions,
    PositionalCorrection,
    UpdateSleeping,
    ClearAccumulators,
    ClampBodyVelocities,
    Count,
};

static_assert(static_cast<std::size_t>(Section::Count) <= kMaxSections,
              "world resource labels must fit in the fixed-size profiler");

using Profiler = resource_monitoring::SectionProfiler<kMaxSections>;
using SectionSummary = resource_monitoring::ResourceSectionSummary<kMaxSections>;

inline constexpr std::array<const char*, kMaxSections> kSectionLabels = {
    "world.step",
    "world.integrate_forces",
    "world.update_broadphase",
    "world.generate_contacts",
    "world.build_manifolds",
    "world.build_islands",
    "world.warm_start_contacts",
    "world.warm_start_joints",
    "world.solve_islands",
    "world.relaxation_pass",
    "world.apply_split_stabilization",
    "world.resolve_toi",
    "world.integrate_orientation",
    "world.solve_joint_positions",
    "world.positional_correction",
    "world.update_sleeping",
    "world.clear_accumulators",
    "world.clamp_body_velocities",
};

inline constexpr std::size_t toIndex(Section section) {
    return static_cast<std::size_t>(section);
}

} // namespace minphys3d::world_resource_monitoring
