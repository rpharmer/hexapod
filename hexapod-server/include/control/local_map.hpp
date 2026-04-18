#pragma once

#include "nav_command.hpp"
#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

inline constexpr int kDefaultLocalMapWidthCells = 41;
inline constexpr int kDefaultLocalMapHeightCells = 41;
inline constexpr double kDefaultLocalMapResolutionM = 0.05;
inline constexpr double kDefaultLocalMapObstacleInflationRadiusM = 0.12;
inline constexpr double kDefaultLocalMapSafetyMarginM = 0.02;
inline constexpr double kDefaultLocalMapObservationTimeoutS = 0.5;
inline constexpr double kDefaultLocalMapObservationDecayS = 1.5;

enum class LocalMapCellState : std::uint8_t {
    Unknown = 0,
    Free = 1,
    Occupied = 2,
};

enum class LocalMapObservationFreshnessClass : std::uint8_t {
    Primary = 0,
    Auxiliary = 1,
};

struct LocalMapConfig {
    int width_cells{kDefaultLocalMapWidthCells};
    int height_cells{kDefaultLocalMapHeightCells};
    double resolution_m{kDefaultLocalMapResolutionM};
    double obstacle_inflation_radius_m{kDefaultLocalMapObstacleInflationRadiusM};
    double safety_margin_m{kDefaultLocalMapSafetyMarginM};
    double observation_timeout_s{kDefaultLocalMapObservationTimeoutS};
    double observation_decay_s{kDefaultLocalMapObservationDecayS};
};

struct LocalMapObservationSample {
    double x_m{};
    double y_m{};
    LocalMapCellState state{LocalMapCellState::Occupied};
};

struct LocalMapObservation {
    TimePointUs timestamp_us{};
    std::vector<LocalMapObservationSample> samples{};
    LocalMapObservationFreshnessClass freshness_class{LocalMapObservationFreshnessClass::Primary};
};

class ILocalMapObservationSource {
public:
    virtual ~ILocalMapObservationSource() = default;

    virtual LocalMapObservation collect(const NavPose2d& pose, const RobotState& est, TimePointUs now) = 0;
};

class SyntheticLocalMapObservationSource final : public ILocalMapObservationSource {
public:
    void setStaticSamples(std::vector<LocalMapObservationSample> samples);
    void queueObservation(LocalMapObservation observation);

    [[nodiscard]] LocalMapObservation collect(const NavPose2d& pose, const RobotState& est, TimePointUs now) override;

private:
    mutable std::mutex mutex_{};
    std::vector<LocalMapObservationSample> static_samples_{};
    std::vector<LocalMapObservation> queued_observations_{};
};

struct LocalOccupancyGrid {
    int width_cells{0};
    int height_cells{0};
    double resolution_m{0.0};
    NavPose2d center_pose{};
    std::vector<LocalMapCellState> cells{};

    [[nodiscard]] bool empty() const { return width_cells <= 0 || height_cells <= 0 || cells.empty(); }
    [[nodiscard]] bool containsCell(int cell_x, int cell_y) const;
    [[nodiscard]] std::size_t indexOf(int cell_x, int cell_y) const;
    [[nodiscard]] LocalMapCellState stateAtCell(int cell_x, int cell_y) const;
    [[nodiscard]] bool worldToCell(double world_x_m, double world_y_m, int& out_cell_x, int& out_cell_y) const;
    [[nodiscard]] NavPose2d cellCenterPose(int cell_x, int cell_y) const;
};

struct LocalMapSnapshot {
    LocalOccupancyGrid raw{};
    LocalOccupancyGrid inflated{};
    TimePointUs last_observation_timestamp{};
    TimePointUs last_primary_observation_timestamp{};
    bool has_observations{false};
    bool has_primary_observations{false};
    bool fresh{false};
    double nearest_obstacle_distance_m{-1.0};
};

class LocalMapBuilder {
public:
    explicit LocalMapBuilder(LocalMapConfig config = {});

    void reset();
    void update(const NavPose2d& pose, TimePointUs now, const std::vector<LocalMapObservation>& observations);

    [[nodiscard]] LocalMapSnapshot snapshot(TimePointUs now) const;
    [[nodiscard]] const LocalMapConfig& config() const { return config_; }

private:
    struct CellRecord {
        LocalMapCellState state{LocalMapCellState::Unknown};
        TimePointUs last_update_us{};
    };

    [[nodiscard]] bool containsCell(int cell_x, int cell_y) const;
    [[nodiscard]] std::size_t indexOf(int cell_x, int cell_y) const;
    void recenterToPose(const NavPose2d& pose);
    void decayStaleCells(TimePointUs now) const;
    [[nodiscard]] LocalOccupancyGrid buildRawGrid(TimePointUs now) const;
    [[nodiscard]] LocalOccupancyGrid buildInflatedGrid(const LocalOccupancyGrid& raw) const;
    [[nodiscard]] double nearestObstacleDistance(const LocalOccupancyGrid& inflated) const;

    LocalMapConfig config_{};
    NavPose2d center_pose_{};
    bool has_center_pose_{false};
    mutable std::vector<CellRecord> cells_{};
    TimePointUs last_observation_timestamp_{};
    TimePointUs last_primary_observation_timestamp_{};
    bool has_observations_{false};
    bool has_primary_observations_{false};
};
