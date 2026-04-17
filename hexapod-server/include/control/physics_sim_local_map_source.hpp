#pragma once

#include "local_map.hpp"
#include "physics_sim_bridge.hpp"

class PhysicsSimLocalMapObservationSource final : public ILocalMapObservationSource {
public:
    explicit PhysicsSimLocalMapObservationSource(
        const IPhysicsSimObstacleFootprintProvider& provider,
        double sample_spacing_m = 0.05);

    [[nodiscard]] LocalMapObservation collect(const NavPose2d& pose, TimePointUs now) override;

private:
    const IPhysicsSimObstacleFootprintProvider& provider_;
    double sample_spacing_m_{0.05};
};
