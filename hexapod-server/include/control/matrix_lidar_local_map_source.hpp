#pragma once

#include "local_map.hpp"
#include "types.hpp"

/// Projects `RobotState::matrix_lidar` range cells into planar world (XY) occupancy samples for `LocalMapBuilder`.
class MatrixLidarLocalMapObservationSource final : public ILocalMapObservationSource {
public:
    MatrixLidarLocalMapObservationSource() = default;

    [[nodiscard]] LocalMapObservation collect(const NavPose2d& pose, const RobotState& est, TimePointUs now) override;
};
