#pragma once

#include "process_resource_monitoring.hpp"

#include <array>
#include <cstddef>

namespace minphys3d::world_resource_monitoring {

inline constexpr std::size_t kMaxSections = 30;
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
    SolveJointPositionsServo,
    PositionalCorrection,
    UpdateSleeping,
    ClearAccumulators,
    ClampBodyVelocities,
    /// `SolveIslands` inner breakdown (nesting; self-time of `world.solve_islands` is only uncaptured overhead).
    SolveIslandsIslandOrder,
    SolveIslandsContactsForward,
    SolveIslandsContactsShock,
    SolveIslandsDistanceJoints,
    SolveIslandsHingeJoints,
    SolveIslandsBallSocketJoints,
    SolveIslandsFixedJoints,
    SolveIslandsPrismaticJoints,
    SolveIslandsServoJoints,
    GetServoJointAngle,
    SolveServoJoint,
    Count,
};

static_assert(static_cast<std::size_t>(Section::Count) == kMaxSections,
              "world resource labels and Section::Count must match kMaxSections");

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
    "world.solve_joint_positions.servo",
    "world.positional_correction",
    "world.update_sleeping",
    "world.clear_accumulators",
    "world.clamp_body_velocities",
    "world.solve_islands.island_order",
    "world.solve_islands.contacts_forward",
    "world.solve_islands.contacts_shock",
    "world.solve_islands.joints_distance",
    "world.solve_islands.joints_hinge",
    "world.solve_islands.joints_ball_socket",
    "world.solve_islands.joints_fixed",
    "world.solve_islands.joints_prismatic",
    "world.solve_islands.joints_servo",
    "world.get_servo_joint_angle",
    "world.solve_servo_joint",
};

inline constexpr std::size_t toIndex(Section section) {
    return static_cast<std::size_t>(section);
}

} // namespace minphys3d::world_resource_monitoring
