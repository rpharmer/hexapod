#pragma once

#include "local_map.hpp"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

inline constexpr double kDefaultLocalPlannerReplanPeriodS = 0.25;
inline constexpr double kDefaultLocalPlannerSearchHorizonM = 1.2;
inline constexpr int kDefaultLocalPlannerSearchNodeBudget = 4000;
inline constexpr int kDefaultLocalPlannerMaxOutputWaypoints = 6;
inline constexpr int kDefaultLocalPlannerSegmentCellHorizon = 12;
inline constexpr double kDefaultLocalPlannerBlockedTimeoutS = 2.0;

struct LocalPlannerConfig {
    /** Parsed for tuning compatibility; not yet applied by `NavigationManager` / `AStarLocalPlanner`. */
    double replan_period_s{kDefaultLocalPlannerReplanPeriodS};
    double search_horizon_m{kDefaultLocalPlannerSearchHorizonM};
    int search_node_budget{kDefaultLocalPlannerSearchNodeBudget};
    int max_output_waypoints{kDefaultLocalPlannerMaxOutputWaypoints};
    int segment_cell_horizon{kDefaultLocalPlannerSegmentCellHorizon};
    double blocked_timeout_s{kDefaultLocalPlannerBlockedTimeoutS};
};

enum class LocalPlanStatus {
    Ready,
    GoalReached,
    Blocked,
    MapUnavailable,
};

enum class PlannerBlockReason {
    None,
    StartOccupied,
    GoalOccupied,
    NoPath,
    SearchBudgetExceeded,
};

struct LocalPlanRequest {
    NavPose2d current_pose{};
    NavPose2d goal_pose{};
    LocalMapSnapshot map{};
};

struct LocalPlanResult {
    LocalPlanStatus status{LocalPlanStatus::Blocked};
    PlannerBlockReason block_reason{PlannerBlockReason::None};
    std::vector<NavPose2d> waypoints{};
    double path_length_m{0.0};
    std::size_t expanded_nodes{0};
};

class ILocalPlanner {
public:
    virtual ~ILocalPlanner() = default;

    virtual LocalPlanResult plan(const LocalPlanRequest& request) const = 0;
};

class AStarLocalPlanner final : public ILocalPlanner {
public:
    explicit AStarLocalPlanner(LocalPlannerConfig config = {});

    [[nodiscard]] LocalPlanResult plan(const LocalPlanRequest& request) const override;

private:
    [[nodiscard]] bool lineOfSight(const LocalOccupancyGrid& grid,
                                   int from_x,
                                   int from_y,
                                   int to_x,
                                   int to_y) const;
    [[nodiscard]] NavPose2d projectGoalIntoHorizon(const LocalPlanRequest& request) const;
    [[nodiscard]] std::vector<NavPose2d> simplifyPath(const LocalOccupancyGrid& grid,
                                                      const std::vector<std::pair<int, int>>& cells,
                                                      const NavPose2d& terminal_goal) const;

    LocalPlannerConfig config_{};
};
