#include "estimator.hpp"
#include "matrix_lidar_local_map_source.hpp"
#include "motion_intent_utils.hpp"
#include "navigation_manager.hpp"
#include "physics_sim_protocol.hpp"
#include "robot_runtime.hpp"
#include "sim_hardware_bridge.hpp"
#include "types.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

class ScriptedEstimator final : public IEstimator {
public:
    void enqueue(const RobotState& sample) {
        queue_.push_back(sample);
    }

    RobotState update(const RobotState& raw) override {
        if (!queue_.empty()) {
            last_ = queue_.front();
            queue_.erase(queue_.begin());
            return last_;
        }
        return raw;
    }

private:
    std::vector<RobotState> queue_{};
    RobotState last_{};
};

RobotState makeEstimatorSample(const NavPose2d& pose, const TimePointUs timestamp_us, const uint64_t sample_id) {
    RobotState est{};
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.x = pose.x_m;
    est.body_twist_state.body_trans_m.y = pose.y_m;
    est.body_twist_state.twist_pos_rad.z = pose.yaw_rad;
    est.sample_id = sample_id;
    est.timestamp_us = timestamp_us;
    return est;
}

} // namespace

int main() {
    auto bridge = std::make_unique<SimHardwareBridge>();
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* estimator_raw = estimator.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg);

    if (!expect(runtime.init(), "runtime should initialize")) {
        return EXIT_FAILURE;
    }

    auto source = std::make_shared<SyntheticLocalMapObservationSource>();
    auto navigation_manager = std::make_unique<NavigationManager>(cfg.local_map, cfg.local_planner);
    navigation_manager->addObservationSource(source);
    navigation_manager->startNavigateToPose(
        makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07),
        NavPose2d{0.25, 0.0, 0.0});
    runtime.setNavigationManager(std::move(navigation_manager));

    runtime.setMotionIntent(makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07));

    for (uint64_t i = 1; i <= 40; ++i) {
        estimator_raw->enqueue(makeEstimatorSample(NavPose2d{}, TimePointUs{1'000'000 + i * 20'000}, i));
        runtime.busStep();
        runtime.estimatorStep();
        runtime.safetyStep();
        runtime.controlStep();
    }

    if (!expect(runtime.navigationManager() != nullptr, "runtime should retain navigation manager")) {
        return EXIT_FAILURE;
    }
    if (!expect(runtime.navigationManager()->monitor().replan_count >= 1,
                "runtime control path should trigger local planning")) {
        return EXIT_FAILURE;
    }
    if (!expect(runtime.navigationManager()->monitor().lifecycle == NavigationLifecycleState::Running ||
                    runtime.navigationManager()->monitor().lifecycle == NavigationLifecycleState::Completed,
                "runtime should keep the navigation task live through control steps")) {
        return EXIT_FAILURE;
    }
    if (!expect(runtime.getStatus().loop_counter > 0,
                "runtime control step should still produce status updates while navigation is active")) {
        return EXIT_FAILURE;
    }

    {
        auto bridge2 = std::make_unique<SimHardwareBridge>();
        auto estimator2 = std::make_unique<ScriptedEstimator>();
        auto* est2_raw = estimator2.get();

        control_config::ControlConfig cfg2{};
        cfg2.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
        cfg2.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
        cfg2.local_map.observation_timeout_s = 0.06;

        RobotRuntime rt2(std::move(bridge2), std::move(estimator2), nullptr, cfg2);
        if (!expect(rt2.init(), "second runtime should initialize")) {
            return EXIT_FAILURE;
        }

        auto nav2 = std::make_unique<NavigationManager>(cfg2.local_map, cfg2.local_planner);
        nav2->addObservationSource(std::make_shared<MatrixLidarLocalMapObservationSource>());
        nav2->startNavigateToPose(makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07),
                                  NavPose2d{0.8, 0.0, 0.0});
        rt2.setNavigationManager(std::move(nav2));
        rt2.setMotionIntent(makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07));

        const TimePointUs lidar_ts = now_us();
        RobotState frozen = makeEstimatorSample(NavPose2d{}, lidar_ts, 42);
        frozen.has_matrix_lidar = true;
        frozen.matrix_lidar.valid = true;
        frozen.matrix_lidar.model = static_cast<std::uint8_t>(physics_sim::MatrixLidarModel::Matrix64x8);
        frozen.matrix_lidar.cols = 64;
        frozen.matrix_lidar.rows = 8;
        frozen.matrix_lidar.ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);
        frozen.matrix_lidar.timestamp_us = lidar_ts;

        for (int i = 0; i < 8; ++i) {
            est2_raw->enqueue(frozen);
            rt2.busStep();
            rt2.estimatorStep();
            rt2.safetyStep();
            rt2.controlStep();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        bool saw_map_unavailable = false;
        for (int i = 0; i < 40; ++i) {
            est2_raw->enqueue(frozen);
            rt2.busStep();
            rt2.estimatorStep();
            rt2.safetyStep();
            rt2.controlStep();
            if (rt2.navigationManager()->monitor().lifecycle == NavigationLifecycleState::MapUnavailable) {
                saw_map_unavailable = true;
                break;
            }
        }

        if (!expect(saw_map_unavailable,
                    "duplicate sample_id must not block navigation ticks; stale LiDAR should surface as "
                    "MapUnavailable")) {
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
