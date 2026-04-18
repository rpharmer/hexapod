#include "local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace {

struct SearchNode {
    int cell_x{0};
    int cell_y{0};
    double g_cost{0.0};
    double f_cost{0.0};
};

struct SearchNodeCompare {
    bool operator()(const SearchNode& lhs, const SearchNode& rhs) const {
        return lhs.f_cost > rhs.f_cost;
    }
};

double heuristicCost(const int from_x, const int from_y, const int to_x, const int to_y) {
    return std::hypot(static_cast<double>(to_x - from_x), static_cast<double>(to_y - from_y));
}

bool hasObservedOccupiedSpace(const LocalOccupancyGrid& grid) {
    return std::any_of(grid.cells.begin(), grid.cells.end(), [](const LocalMapCellState state) {
        return state == LocalMapCellState::Occupied;
    });
}

bool traversable(const LocalMapCellState state, const bool allow_unknown_traversal) {
    if (state == LocalMapCellState::Occupied) {
        return false;
    }
    if (state == LocalMapCellState::Free) {
        return true;
    }
    return allow_unknown_traversal;
}

double pathLength(const std::vector<NavPose2d>& waypoints) {
    double total = 0.0;
    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        total += std::hypot(waypoints[i].x_m - waypoints[i - 1].x_m,
                            waypoints[i].y_m - waypoints[i - 1].y_m);
    }
    return total;
}

int clampToCellRange(const int value, const int limit) {
    return std::max(0, std::min(limit - 1, value));
}

} // namespace

AStarLocalPlanner::AStarLocalPlanner(LocalPlannerConfig config)
    : config_(std::move(config)) {}

LocalPlanResult AStarLocalPlanner::plan(const LocalPlanRequest& request) const {
    LocalPlanResult out{};
    if (!request.map.has_observations || !request.map.fresh) {
        out.status = LocalPlanStatus::MapUnavailable;
        return out;
    }

    const LocalOccupancyGrid& grid = request.map.inflated;
    if (grid.empty()) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::NoPath;
        return out;
    }
    const bool allow_unknown_traversal = !hasObservedOccupiedSpace(grid);

    const NavPose2d planning_goal = projectGoalIntoHorizon(request);
    if (std::hypot(planning_goal.x_m - request.current_pose.x_m,
                   planning_goal.y_m - request.current_pose.y_m) <= grid.resolution_m) {
        out.status = LocalPlanStatus::GoalReached;
        return out;
    }

    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    if (!grid.worldToCell(request.current_pose.x_m, request.current_pose.y_m, start_x, start_y)) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::NoPath;
        return out;
    }
    if (!grid.worldToCell(planning_goal.x_m, planning_goal.y_m, goal_x, goal_y)) {
        goal_x = clampToCellRange(goal_x, grid.width_cells);
        goal_y = clampToCellRange(goal_y, grid.height_cells);
    }

    if (!traversable(grid.stateAtCell(start_x, start_y), allow_unknown_traversal)) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::StartOccupied;
        return out;
    }
    if (!traversable(grid.stateAtCell(goal_x, goal_y), allow_unknown_traversal)) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::GoalOccupied;
        return out;
    }

    const int width = grid.width_cells;
    const std::size_t cell_count = static_cast<std::size_t>(width * grid.height_cells);
    const auto index_of = [width](const int x, const int y) {
        return static_cast<std::size_t>(y * width + x);
    };

    std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::priority_queue<SearchNode, std::vector<SearchNode>, SearchNodeCompare> open;

    const std::size_t start_index = index_of(start_x, start_y);
    g_score[start_index] = 0.0;
    open.push(SearchNode{start_x, start_y, 0.0, heuristicCost(start_x, start_y, goal_x, goal_y)});

    const std::vector<std::pair<int, int>> neighbors = {
        {-1, -1}, {0, -1}, {1, -1},
        {-1, 0},            {1, 0},
        {-1, 1},  {0, 1},   {1, 1},
    };

    bool found = false;
    std::size_t expanded = 0;
    while (!open.empty()) {
        const SearchNode current = open.top();
        open.pop();
        ++expanded;
        if (expanded > static_cast<std::size_t>(std::max(1, config_.search_node_budget))) {
            out.status = LocalPlanStatus::Blocked;
            out.block_reason = PlannerBlockReason::SearchBudgetExceeded;
            out.expanded_nodes = expanded;
            return out;
        }

        if (current.cell_x == goal_x && current.cell_y == goal_y) {
            found = true;
            break;
        }

        const std::size_t current_index = index_of(current.cell_x, current.cell_y);
        if (current.g_cost > g_score[current_index] + 1e-9) {
            continue;
        }

        for (const auto& [dx, dy] : neighbors) {
            const int nx = current.cell_x + dx;
            const int ny = current.cell_y + dy;
            if (!grid.containsCell(nx, ny) ||
                !traversable(grid.stateAtCell(nx, ny), allow_unknown_traversal)) {
                continue;
            }

            const double step_cost = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
            const double tentative = current.g_cost + step_cost;
            const std::size_t next_index = index_of(nx, ny);
            if (tentative + 1e-9 >= g_score[next_index]) {
                continue;
            }

            g_score[next_index] = tentative;
            parent[next_index] = static_cast<int>(current_index);
            open.push(SearchNode{nx, ny, tentative, tentative + heuristicCost(nx, ny, goal_x, goal_y)});
        }
    }

    out.expanded_nodes = expanded;
    if (!found) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::NoPath;
        return out;
    }

    std::vector<std::pair<int, int>> path_cells{};
    int cursor = static_cast<int>(index_of(goal_x, goal_y));
    while (cursor >= 0) {
        const int cell_x = cursor % width;
        const int cell_y = cursor / width;
        path_cells.emplace_back(cell_x, cell_y);
        if (cursor == static_cast<int>(start_index)) {
            break;
        }
        cursor = parent[static_cast<std::size_t>(cursor)];
    }
    std::reverse(path_cells.begin(), path_cells.end());
    if (path_cells.empty()) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::NoPath;
        return out;
    }

    const int segment_cells = std::max(2, config_.segment_cell_horizon);
    if (static_cast<int>(path_cells.size()) > segment_cells) {
        path_cells.resize(static_cast<std::size_t>(segment_cells));
    }

    out.waypoints = simplifyPath(grid, path_cells, request.goal_pose);
    if (out.waypoints.empty()) {
        out.status = LocalPlanStatus::Blocked;
        out.block_reason = PlannerBlockReason::NoPath;
        return out;
    }

    out.path_length_m = pathLength(out.waypoints);
    out.status = LocalPlanStatus::Ready;
    return out;
}

bool AStarLocalPlanner::lineOfSight(const LocalOccupancyGrid& grid,
                                    const int from_x,
                                    const int from_y,
                                    const int to_x,
                                    const int to_y) const {
    const bool allow_unknown_traversal = !hasObservedOccupiedSpace(grid);
    int x0 = from_x;
    int y0 = from_y;
    const int x1 = to_x;
    const int y1 = to_y;
    const int dx = std::abs(x1 - x0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!grid.containsCell(x0, y0) ||
            !traversable(grid.stateAtCell(x0, y0), allow_unknown_traversal)) {
            return false;
        }
        if (x0 == x1 && y0 == y1) {
            return true;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

NavPose2d AStarLocalPlanner::projectGoalIntoHorizon(const LocalPlanRequest& request) const {
    const double dx = request.goal_pose.x_m - request.current_pose.x_m;
    const double dy = request.goal_pose.y_m - request.current_pose.y_m;
    const double distance = std::hypot(dx, dy);
    if (distance <= config_.search_horizon_m || distance <= 1e-9) {
        return request.goal_pose;
    }

    const double scale = config_.search_horizon_m / distance;
    return NavPose2d{
        request.current_pose.x_m + dx * scale,
        request.current_pose.y_m + dy * scale,
        request.goal_pose.yaw_rad,
    };
}

std::vector<NavPose2d> AStarLocalPlanner::simplifyPath(const LocalOccupancyGrid& grid,
                                                       const std::vector<std::pair<int, int>>& cells,
                                                       const NavPose2d& terminal_goal) const {
    if (cells.empty()) {
        return {};
    }

    std::vector<std::pair<int, int>> simplified{};
    simplified.push_back(cells.front());
    std::size_t anchor = 0;
    while (anchor + 1 < cells.size()) {
        std::size_t farthest = anchor + 1;
        while (farthest + 1 < cells.size() &&
               lineOfSight(grid,
                           cells[anchor].first,
                           cells[anchor].second,
                           cells[farthest + 1].first,
                           cells[farthest + 1].second)) {
            ++farthest;
        }
        simplified.push_back(cells[farthest]);
        anchor = farthest;
    }

    if (static_cast<int>(simplified.size()) > config_.max_output_waypoints) {
        simplified.resize(static_cast<std::size_t>(config_.max_output_waypoints));
        if (simplified.back() != cells.back()) {
            simplified.back() = cells.back();
        }
    }

    std::vector<NavPose2d> waypoints{};
    waypoints.reserve(simplified.size());
    for (std::size_t i = 0; i < simplified.size(); ++i) {
        NavPose2d waypoint = grid.cellCenterPose(simplified[i].first, simplified[i].second);
        if (i + 1 < simplified.size()) {
            const NavPose2d next = grid.cellCenterPose(simplified[i + 1].first, simplified[i + 1].second);
            waypoint.yaw_rad = std::atan2(next.y_m - waypoint.y_m, next.x_m - waypoint.x_m);
        } else {
            waypoint.yaw_rad = terminal_goal.yaw_rad;
        }
        waypoints.push_back(waypoint);
    }

    return waypoints;
}
