#include "navigation_manager.hpp"

#include "motion_intent_utils.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace {

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

void fillState(RobotState& est, const NavPose2d& pose, const TimePointUs now, const uint64_t sample_id) {
    est.valid = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m.x = pose.x_m;
    est.body_twist_state.body_trans_m.y = pose.y_m;
    est.body_twist_state.twist_pos_rad.z = pose.yaw_rad;
    est.sample_id = sample_id;
    est.timestamp_us = now;
}

void stepPose(NavPose2d& pose, const MotionIntent& intent, const double dt_s) {
    const PlanarMotionCommand cmd = planarMotionCommand(intent);
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    const double world_vx = -cmd.vx_mps;
    const double world_vy = cmd.vy_mps;
    pose.x_m += (c * world_vx - s * world_vy) * dt_s;
    pose.y_m += (s * world_vx + c * world_vy) * dt_s;
    pose.yaw_rad = navWrapAngleRad(pose.yaw_rad + cmd.yaw_rate_radps * dt_s);
}

} // namespace

int main() {
    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.07);
    MotionIntent walk = makeMotionIntent(RobotMode::WALK, GaitType::TRIPOD, 0.07);

    auto source = std::make_shared<SyntheticLocalMapObservationSource>();
    NavigationManager nav(LocalMapConfig{}, LocalPlannerConfig{});
    nav.addObservationSource(source);
    nav.startNavigateToPose(walk, NavPose2d{0.35, 0.0, 0.0});

    RobotState est{};
    NavPose2d sim{};
    const double dt_s = 0.02;
    MotionIntent last = stand;
    for (uint64_t i = 1; i < 4000; ++i) {
        const TimePointUs now{1'000'000 + static_cast<uint64_t>(i * 20'000)};
        fillState(est, sim, now, i);
        last = nav.mergeIntent(stand, est, now);
        if (!nav.active()) {
            break;
        }
        stepPose(sim, last, dt_s);
    }
    if (!expect(nav.monitor().lifecycle == NavigationLifecycleState::Completed,
                "map-aware navigation should complete on empty observed map")) {
        return EXIT_FAILURE;
    }
    if (!expect(std::abs(sim.x_m - 0.35) < 0.08, "sim pose should approach navigation goal")) {
        return EXIT_FAILURE;
    }

    auto dynamic_source = std::make_shared<SyntheticLocalMapObservationSource>();
    NavigationManager replan_nav(LocalMapConfig{}, LocalPlannerConfig{});
    replan_nav.addObservationSource(dynamic_source);
    replan_nav.startNavigateToPose(walk, NavPose2d{0.5, 0.0, 0.0});
    sim = NavPose2d{};
    for (uint64_t i = 1; i < 5000; ++i) {
        const TimePointUs now{2'000'000 + static_cast<uint64_t>(i * 20'000)};
        fillState(est, sim, now, i);
        if (i == 40) {
            dynamic_source->setStaticSamples({
                LocalMapObservationSample{0.22, 0.00, LocalMapCellState::Occupied},
                LocalMapObservationSample{0.27, 0.00, LocalMapCellState::Occupied},
            });
        }
        last = replan_nav.mergeIntent(stand, est, now);
        if (!replan_nav.active()) {
            break;
        }
        stepPose(sim, last, dt_s);
    }
    if (!expect(replan_nav.monitor().replan_count >= 2,
                "dynamic obstacle insertion should trigger replanning")) {
        return EXIT_FAILURE;
    }
    if (!expect(replan_nav.monitor().lifecycle != NavigationLifecycleState::MapUnavailable,
                "replanned navigation should keep a usable map-aware lifecycle")) {
        return EXIT_FAILURE;
    }

    auto blocked_source = std::make_shared<SyntheticLocalMapObservationSource>();
    blocked_source->setStaticSamples({
        LocalMapObservationSample{0.12, -0.20, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, -0.15, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, -0.10, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, -0.05, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, 0.00, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, 0.05, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, 0.10, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, 0.15, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.12, 0.20, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, -0.20, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, -0.15, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, -0.10, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, -0.05, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, 0.00, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, 0.05, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, 0.10, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, 0.15, LocalMapCellState::Occupied},
        LocalMapObservationSample{0.18, 0.20, LocalMapCellState::Occupied},
    });
    LocalPlannerConfig blocked_cfg{};
    blocked_cfg.blocked_timeout_s = 0.15;
    NavigationManager blocked_nav(LocalMapConfig{}, blocked_cfg);
    blocked_nav.addObservationSource(blocked_source);
    blocked_nav.startNavigateToPose(walk, NavPose2d{0.5, 0.0, 0.0});
    sim = NavPose2d{};
    bool saw_stop = false;
    for (uint64_t i = 1; i < 120; ++i) {
        const TimePointUs now{3'000'000 + static_cast<uint64_t>(i * 20'000)};
        fillState(est, sim, now, i);
        last = blocked_nav.mergeIntent(stand, est, now);
        const PlanarMotionCommand cmd = planarMotionCommand(last);
        if (std::hypot(cmd.vx_mps, cmd.vy_mps) < 1e-6) {
            saw_stop = true;
        }
        if (!blocked_nav.active()) {
            break;
        }
    }
    if (!expect(saw_stop, "blocked navigation should command a safe stop")) {
        return EXIT_FAILURE;
    }
    if (!expect(blocked_nav.monitor().lifecycle == NavigationLifecycleState::Failed ||
                    blocked_nav.monitor().lifecycle == NavigationLifecycleState::Blocked,
                "blocked navigation should surface blocked or failed lifecycle")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
