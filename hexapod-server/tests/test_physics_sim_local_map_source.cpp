#include "physics_sim_local_map_source.hpp"
#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class FakeObstacleProvider final : public IPhysicsSimObstacleFootprintProvider {
public:
    std::vector<PhysicsSimObstacleFootprint> latestObstacleFootprints() const override {
        return footprints;
    }

    std::vector<PhysicsSimObstacleFootprint> footprints{};
};

bool hasNearbySample(const LocalMapObservation& observation, double x_m, double y_m, double tolerance_m) {
    return std::any_of(observation.samples.begin(),
                       observation.samples.end(),
                       [&](const LocalMapObservationSample& sample) {
                           return std::hypot(sample.x_m - x_m, sample.y_m - y_m) <= tolerance_m;
                       });
}

} // namespace

int main() {
    constexpr double kHalfPi = 1.5707963267948966;
    FakeObstacleProvider provider;
    provider.footprints.push_back(PhysicsSimObstacleFootprint{1.0, 2.0, 0.10, 0.20, 0.0});
    provider.footprints.push_back(PhysicsSimObstacleFootprint{0.0, 0.0, 0.10, 0.10, kHalfPi});

    PhysicsSimLocalMapObservationSource source(provider, 0.10);
    RobotState est{};
    const LocalMapObservation observation = source.collect(NavPose2d{}, est, TimePointUs{123456});

    if (!expect(observation.timestamp_us.value == 123456, "source should stamp observation with collection time")) {
        return EXIT_FAILURE;
    }
    if (!expect(observation.samples.size() >= 18, "source should rasterize obstacle footprints into occupied samples")) {
        return EXIT_FAILURE;
    }
    if (!expect(hasNearbySample(observation, 1.0, 2.0, 0.02), "axis-aligned obstacle should include center sample")) {
        return EXIT_FAILURE;
    }
    if (!expect(hasNearbySample(observation, 0.90, 1.80, 0.03),
                "axis-aligned obstacle should cover its lower-left corner")) {
        return EXIT_FAILURE;
    }
    if (!expect(hasNearbySample(observation, -0.10, 0.0, 0.03),
                "rotated obstacle should extend along world X after rasterization")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
