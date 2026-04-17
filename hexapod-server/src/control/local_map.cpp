#include "local_map.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

double ageSeconds(const TimePointUs now, const TimePointUs then) {
    if (then.isZero() || now.value <= then.value) {
        return 0.0;
    }
    return static_cast<double>(now.value - then.value) * 1e-6;
}

} // namespace

void SyntheticLocalMapObservationSource::setStaticSamples(std::vector<LocalMapObservationSample> samples) {
    const std::lock_guard<std::mutex> lock(mutex_);
    static_samples_ = std::move(samples);
}

void SyntheticLocalMapObservationSource::queueObservation(LocalMapObservation observation) {
    const std::lock_guard<std::mutex> lock(mutex_);
    queued_observations_.push_back(std::move(observation));
    std::sort(queued_observations_.begin(),
              queued_observations_.end(),
              [](const LocalMapObservation& lhs, const LocalMapObservation& rhs) {
                  return lhs.timestamp_us.value < rhs.timestamp_us.value;
              });
}

LocalMapObservation SyntheticLocalMapObservationSource::collect(const NavPose2d& /*pose*/, const TimePointUs now) {
    const std::lock_guard<std::mutex> lock(mutex_);
    LocalMapObservation merged{};
    merged.timestamp_us = now;
    merged.samples = static_samples_;

    auto it = queued_observations_.begin();
    while (it != queued_observations_.end()) {
        if (!it->timestamp_us.isZero() && it->timestamp_us.value > now.value) {
            break;
        }
        if (!it->timestamp_us.isZero()) {
            merged.timestamp_us = it->timestamp_us;
        }
        merged.samples.insert(merged.samples.end(), it->samples.begin(), it->samples.end());
        it = queued_observations_.erase(it);
    }

    return merged;
}

bool LocalOccupancyGrid::containsCell(const int cell_x, const int cell_y) const {
    return cell_x >= 0 && cell_y >= 0 && cell_x < width_cells && cell_y < height_cells;
}

std::size_t LocalOccupancyGrid::indexOf(const int cell_x, const int cell_y) const {
    return static_cast<std::size_t>(cell_y * width_cells + cell_x);
}

LocalMapCellState LocalOccupancyGrid::stateAtCell(const int cell_x, const int cell_y) const {
    if (!containsCell(cell_x, cell_y)) {
        return LocalMapCellState::Unknown;
    }
    return cells[indexOf(cell_x, cell_y)];
}

bool LocalOccupancyGrid::worldToCell(const double world_x_m,
                                     const double world_y_m,
                                     int& out_cell_x,
                                     int& out_cell_y) const {
    if (width_cells <= 0 || height_cells <= 0 || resolution_m <= 0.0) {
        return false;
    }

    const double half_w = static_cast<double>(width_cells - 1) * 0.5;
    const double half_h = static_cast<double>(height_cells - 1) * 0.5;
    const double dx_cells = (world_x_m - center_pose.x_m) / resolution_m;
    const double dy_cells = (world_y_m - center_pose.y_m) / resolution_m;
    out_cell_x = static_cast<int>(std::llround(dx_cells + half_w));
    out_cell_y = static_cast<int>(std::llround(dy_cells + half_h));
    return containsCell(out_cell_x, out_cell_y);
}

NavPose2d LocalOccupancyGrid::cellCenterPose(const int cell_x, const int cell_y) const {
    const double half_w = static_cast<double>(width_cells - 1) * 0.5;
    const double half_h = static_cast<double>(height_cells - 1) * 0.5;
    const double local_x = (static_cast<double>(cell_x) - half_w) * resolution_m;
    const double local_y = (static_cast<double>(cell_y) - half_h) * resolution_m;
    return NavPose2d{center_pose.x_m + local_x, center_pose.y_m + local_y, center_pose.yaw_rad};
}

LocalMapBuilder::LocalMapBuilder(LocalMapConfig config)
    : config_(std::move(config)),
      cells_(static_cast<std::size_t>(config_.width_cells * config_.height_cells)) {}

void LocalMapBuilder::reset() {
    has_center_pose_ = false;
    center_pose_ = NavPose2d{};
    has_observations_ = false;
    last_observation_timestamp_ = TimePointUs{};
    cells_.assign(static_cast<std::size_t>(config_.width_cells * config_.height_cells), CellRecord{});
}

void LocalMapBuilder::update(const NavPose2d& pose,
                             const TimePointUs now,
                             const std::vector<LocalMapObservation>& observations) {
    recenterToPose(pose);

    LocalOccupancyGrid addressing{};
    addressing.width_cells = config_.width_cells;
    addressing.height_cells = config_.height_cells;
    addressing.resolution_m = config_.resolution_m;
    addressing.center_pose = center_pose_;

    for (const LocalMapObservation& observation : observations) {
        if (observation.timestamp_us.isZero()) {
            continue;
        }
        has_observations_ = true;
        if (observation.timestamp_us.value > last_observation_timestamp_.value) {
            last_observation_timestamp_ = observation.timestamp_us;
        }

        for (const LocalMapObservationSample& sample : observation.samples) {
            int cell_x = 0;
            int cell_y = 0;
            if (!addressing.worldToCell(sample.x_m, sample.y_m, cell_x, cell_y)) {
                continue;
            }
            CellRecord& cell = cells_[indexOf(cell_x, cell_y)];
            cell.state = sample.state;
            cell.last_update_us = observation.timestamp_us;
        }
    }

    (void)now;
}

LocalMapSnapshot LocalMapBuilder::snapshot(const TimePointUs now) const {
    const LocalOccupancyGrid raw = buildRawGrid(now);
    const LocalOccupancyGrid inflated = buildInflatedGrid(raw);

    LocalMapSnapshot out{};
    out.raw = raw;
    out.inflated = inflated;
    out.last_observation_timestamp = last_observation_timestamp_;
    out.has_observations = has_observations_;
    out.fresh = has_observations_ &&
                ageSeconds(now, last_observation_timestamp_) <= config_.observation_timeout_s;
    out.nearest_obstacle_distance_m = nearestObstacleDistance(inflated);
    return out;
}

bool LocalMapBuilder::containsCell(const int cell_x, const int cell_y) const {
    return cell_x >= 0 && cell_y >= 0 &&
           cell_x < config_.width_cells && cell_y < config_.height_cells;
}

std::size_t LocalMapBuilder::indexOf(const int cell_x, const int cell_y) const {
    return static_cast<std::size_t>(cell_y * config_.width_cells + cell_x);
}

void LocalMapBuilder::recenterToPose(const NavPose2d& pose) {
    if (!has_center_pose_) {
        center_pose_ = pose;
        has_center_pose_ = true;
        return;
    }

    if (config_.resolution_m <= 0.0) {
        center_pose_ = pose;
        return;
    }

    const int shift_x = static_cast<int>(std::llround((pose.x_m - center_pose_.x_m) / config_.resolution_m));
    const int shift_y = static_cast<int>(std::llround((pose.y_m - center_pose_.y_m) / config_.resolution_m));
    center_pose_ = pose;
    if (shift_x == 0 && shift_y == 0) {
        return;
    }

    std::vector<CellRecord> shifted(cells_.size());
    for (int y = 0; y < config_.height_cells; ++y) {
        for (int x = 0; x < config_.width_cells; ++x) {
            const int src_x = x + shift_x;
            const int src_y = y + shift_y;
            if (!containsCell(src_x, src_y)) {
                continue;
            }
            shifted[indexOf(x, y)] = cells_[indexOf(src_x, src_y)];
        }
    }
    cells_.swap(shifted);
}

void LocalMapBuilder::decayStaleCells(const TimePointUs now) const {
    for (CellRecord& cell : cells_) {
        if (cell.state == LocalMapCellState::Unknown || cell.last_update_us.isZero()) {
            continue;
        }
        if (ageSeconds(now, cell.last_update_us) > config_.observation_decay_s) {
            cell.state = LocalMapCellState::Unknown;
            cell.last_update_us = TimePointUs{};
        }
    }
}

LocalOccupancyGrid LocalMapBuilder::buildRawGrid(const TimePointUs now) const {
    decayStaleCells(now);

    LocalOccupancyGrid grid{};
    grid.width_cells = config_.width_cells;
    grid.height_cells = config_.height_cells;
    grid.resolution_m = config_.resolution_m;
    grid.center_pose = center_pose_;
    grid.cells.resize(cells_.size(), LocalMapCellState::Unknown);
    for (std::size_t i = 0; i < cells_.size(); ++i) {
        grid.cells[i] = cells_[i].state;
    }
    return grid;
}

LocalOccupancyGrid LocalMapBuilder::buildInflatedGrid(const LocalOccupancyGrid& raw) const {
    LocalOccupancyGrid inflated = raw;
    const double inflation_radius_m = config_.obstacle_inflation_radius_m + config_.safety_margin_m;
    if (raw.empty() || raw.resolution_m <= 0.0 || inflation_radius_m <= 0.0) {
        return inflated;
    }

    const int inflation_radius_cells = static_cast<int>(std::ceil(inflation_radius_m / raw.resolution_m));
    for (int y = 0; y < raw.height_cells; ++y) {
        for (int x = 0; x < raw.width_cells; ++x) {
            if (raw.stateAtCell(x, y) != LocalMapCellState::Occupied) {
                continue;
            }
            for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
                for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (!inflated.containsCell(nx, ny)) {
                        continue;
                    }
                    const double distance_m =
                        std::hypot(static_cast<double>(dx) * raw.resolution_m,
                                   static_cast<double>(dy) * raw.resolution_m);
                    if (distance_m <= inflation_radius_m + 1e-9) {
                        inflated.cells[inflated.indexOf(nx, ny)] = LocalMapCellState::Occupied;
                    }
                }
            }
        }
    }

    return inflated;
}

double LocalMapBuilder::nearestObstacleDistance(const LocalOccupancyGrid& inflated) const {
    if (inflated.empty()) {
        return -1.0;
    }

    double best_distance = -1.0;
    for (int y = 0; y < inflated.height_cells; ++y) {
        for (int x = 0; x < inflated.width_cells; ++x) {
            if (inflated.stateAtCell(x, y) != LocalMapCellState::Occupied) {
                continue;
            }
            const NavPose2d cell_pose = inflated.cellCenterPose(x, y);
            const double distance = std::hypot(cell_pose.x_m - inflated.center_pose.x_m,
                                               cell_pose.y_m - inflated.center_pose.y_m);
            if (best_distance < 0.0 || distance < best_distance) {
                best_distance = distance;
            }
        }
    }
    return best_distance;
}
