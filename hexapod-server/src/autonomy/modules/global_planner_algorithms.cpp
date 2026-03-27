#include "global_planner_algorithms.hpp"

#include "kinematics/math_types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace autonomy::global_planner::algorithms {
namespace {

constexpr int kGridSize = 23;
constexpr int kGridHalf = kGridSize / 2;
constexpr double kGridResolutionM = 0.25;

struct GridNode {
    int x{0};
    int y{0};
    double g{0.0};
    double f{0.0};
};

double heuristicCost(int from_x, int from_y, int to_x, int to_y) {
    return std::hypot(static_cast<double>(to_x - from_x), static_cast<double>(to_y - from_y));
}

double normalizedDot(double ax, double ay, double bx, double by) {
    const double a_norm = std::hypot(ax, ay);
    const double b_norm = std::hypot(bx, by);
    if (a_norm <= std::numeric_limits<double>::epsilon() || b_norm <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }
    return ((ax * bx) + (ay * by)) / (a_norm * b_norm);
}

double cellRiskProxy(int cell_x,
                     int cell_y,
                     int target_x,
                     int target_y,
                     double traversability_cost,
                     double traversability_risk) {
    const double world_x = static_cast<double>(cell_x - kGridHalf) * kGridResolutionM;
    const double world_y = static_cast<double>(cell_y - kGridHalf) * kGridResolutionM;
    const double target_world_x = static_cast<double>(target_x - kGridHalf) * kGridResolutionM;
    const double target_world_y = static_cast<double>(target_y - kGridHalf) * kGridResolutionM;
    const double corridor_alignment = std::max(0.0, normalizedDot(world_x, world_y, target_world_x, target_world_y));
    const double lateral_term = std::abs(normalizedDot(world_x, world_y, target_world_y, -target_world_x));
    const double radial_bias = std::min(1.0, std::hypot(world_x, world_y) / std::max(1.0, std::hypot(target_world_x, target_world_y)));

    return ::clamp01(traversability_risk * (0.55 + 0.35 * corridor_alignment) +
                     traversability_cost * (0.35 + 0.25 * radial_bias) +
                     0.2 * lateral_term);
}

std::vector<Waypoint> gridPathToRoute(const std::vector<std::pair<int, int>>& cell_path,
                                      const Waypoint& target,
                                      double traversability_cost,
                                      double traversability_risk) {
    std::vector<Waypoint> route;
    route.reserve(cell_path.size());
    for (const auto& [cell_x, cell_y] : cell_path) {
        if (cell_x == kGridHalf && cell_y == kGridHalf) {
            continue;
        }
        const double risk_proxy = cellRiskProxy(cell_x,
                                                cell_y,
                                                cell_path.back().first,
                                                cell_path.back().second,
                                                traversability_cost,
                                                traversability_risk);
        if (risk_proxy >= 0.985) {
            continue;
        }
        route.push_back(Waypoint{
            .frame_id = target.frame_id,
            .x_m = static_cast<double>(cell_x - kGridHalf) * kGridResolutionM,
            .y_m = static_cast<double>(cell_y - kGridHalf) * kGridResolutionM,
            .yaw_rad = target.yaw_rad,
        });
    }
    return route;
}

std::vector<Waypoint> directFallbackRoute(const Waypoint& target) {
    constexpr int kFallbackSteps = 3;
    std::vector<Waypoint> route;
    route.reserve(kFallbackSteps);
    for (int i = 1; i <= kFallbackSteps; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(kFallbackSteps);
        route.push_back(Waypoint{
            .frame_id = target.frame_id,
            .x_m = target.x_m * alpha,
            .y_m = target.y_m * alpha,
            .yaw_rad = target.yaw_rad,
        });
    }
    return route;
}

} // namespace

std::vector<Waypoint> buildConstraintAwareRoute(const Waypoint& target,
                                                double traversability_cost,
                                                double traversability_risk) {
    const int start_x = kGridHalf;
    const int start_y = kGridHalf;
    const int goal_x = std::clamp(start_x + static_cast<int>(std::lround(target.x_m / kGridResolutionM)), 0, kGridSize - 1);
    const int goal_y = std::clamp(start_y + static_cast<int>(std::lround(target.y_m / kGridResolutionM)), 0, kGridSize - 1);
    if (goal_x == start_x && goal_y == start_y) {
        return directFallbackRoute(target);
    }

    std::array<std::array<double, kGridSize>, kGridSize> g_score{};
    std::array<std::array<bool, kGridSize>, kGridSize> closed{};
    std::array<std::array<std::pair<int, int>, kGridSize>, kGridSize> parent{};
    for (int y = 0; y < kGridSize; ++y) {
        for (int x = 0; x < kGridSize; ++x) {
            g_score[y][x] = std::numeric_limits<double>::infinity();
            closed[y][x] = false;
            parent[y][x] = {-1, -1};
        }
    }

    auto compare = [](const GridNode& a, const GridNode& b) { return a.f > b.f; };
    std::priority_queue<GridNode, std::vector<GridNode>, decltype(compare)> open(compare);
    g_score[start_y][start_x] = 0.0;
    open.push(GridNode{start_x, start_y, 0.0, heuristicCost(start_x, start_y, goal_x, goal_y)});

    constexpr int kNeighbors[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    bool found_goal = false;
    while (!open.empty()) {
        const GridNode node = open.top();
        open.pop();
        if (closed[node.y][node.x]) {
            continue;
        }
        closed[node.y][node.x] = true;
        if (node.x == goal_x && node.y == goal_y) {
            found_goal = true;
            break;
        }
        for (const auto& delta : kNeighbors) {
            const int nx = node.x + delta[0];
            const int ny = node.y + delta[1];
            if (nx < 0 || ny < 0 || nx >= kGridSize || ny >= kGridSize || closed[ny][nx]) {
                continue;
            }
            const double step_distance = std::hypot(static_cast<double>(delta[0]), static_cast<double>(delta[1])) * kGridResolutionM;
            const double risk_proxy = cellRiskProxy(nx, ny, goal_x, goal_y, traversability_cost, traversability_risk);
            if (risk_proxy >= 0.995) {
                continue;
            }
            const double candidate_g = node.g + step_distance * (1.0 + traversability_cost + risk_proxy * 1.5);
            if (candidate_g + 1e-9 < g_score[ny][nx]) {
                g_score[ny][nx] = candidate_g;
                parent[ny][nx] = {node.x, node.y};
                const double h = heuristicCost(nx, ny, goal_x, goal_y) * (1.0 + traversability_risk);
                open.push(GridNode{nx, ny, candidate_g, candidate_g + h});
            }
        }
    }

    if (!found_goal) {
        return {};
    }

    std::vector<std::pair<int, int>> cell_path;
    for (int cx = goal_x, cy = goal_y; cx >= 0 && cy >= 0;) {
        cell_path.emplace_back(cx, cy);
        if (cx == start_x && cy == start_y) {
            break;
        }
        const auto [px, py] = parent[cy][cx];
        cx = px;
        cy = py;
    }
    std::reverse(cell_path.begin(), cell_path.end());
    auto route = gridPathToRoute(cell_path, target, traversability_cost, traversability_risk);
    if (route.empty()) {
        route = directFallbackRoute(target);
    }
    return route;
}

} // namespace autonomy::global_planner::algorithms
