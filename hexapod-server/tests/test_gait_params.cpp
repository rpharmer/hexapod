#include "control_config.hpp"
#include "gait_scheduler.hpp"
#include "gait_params.hpp"
#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEq(double a, double b, double eps = 1e-6) {
    return std::abs(a - b) <= eps;
}

bool testWalkEntryBeginsInStance() {
    control_config::GaitConfig cfg{};
    GaitScheduler scheduler(cfg);
    RobotState estimated{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    BodyTwist stopped{};

    MotionIntent stand{};
    stand.requested_mode = RobotMode::STAND;
    stand.timestamp_us = TimePointUs{1'000'000};
    (void)scheduler.update(estimated, stand, safety, stopped);

    MotionIntent walk = stand;
    walk.requested_mode = RobotMode::WALK;
    walk.timestamp_us = TimePointUs{1'004'000};
    BodyTwist forward{};
    forward.linear_mps.x = 0.04;
    const GaitState entry = scheduler.update(estimated, walk, safety, forward);

    if (!(entry.duty_factor > 0.9) || !(entry.step_length_m < 1e-9)) {
        std::cerr << "FAIL: walk entry should begin from an all-stance, zero-stride gait\n";
        return false;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!entry.in_stance[static_cast<std::size_t>(leg)]) {
            std::cerr << "FAIL: walk entry should not put a tripod directly into swing\n";
            return false;
        }
    }

    GaitState settled{};
    for (int i = 1; i <= 50; ++i) {
        walk.timestamp_us = TimePointUs{static_cast<uint64_t>(1'004'000 + i * 4'000)};
        settled = scheduler.update(estimated, walk, safety, forward);
    }
    const bool has_swing_leg = std::any_of(
        settled.in_stance.begin(), settled.in_stance.end(), [](const bool stance) { return !stance; });
    if (!has_swing_leg || !(settled.step_length_m > 0.01) || !(settled.duty_factor < 0.55)) {
        std::cerr << "FAIL: walk-entry blend should reach the commanded walking gait by 0.20 s\n";
        return false;
    }
    return true;
}

bool testWalkEntryCoversBothTripodGroups() {
    control_config::GaitConfig cfg{};
    GaitScheduler scheduler(cfg);
    RobotState estimated{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    BodyTwist stopped{};

    MotionIntent stand{};
    stand.requested_mode = RobotMode::STAND;
    stand.timestamp_us = TimePointUs{1'000'000};
    (void)scheduler.update(estimated, stand, safety, stopped);

    MotionIntent walk = stand;
    walk.requested_mode = RobotMode::WALK;
    BodyTwist forward{};
    forward.linear_mps.x = 0.12;

    bool saw_group_a_swing = false;
    bool saw_group_b_swing = false;
    GaitState gait{};
    for (int frame = 0; frame < 72; ++frame) {
        walk.timestamp_us = TimePointUs{static_cast<uint64_t>(1'005'000 + frame * 5'000)};
        gait = scheduler.update(estimated, walk, safety, forward);
        if (frame == 0) {
            for (int leg = 0; leg < kNumLegs; ++leg) {
                if (!gait.in_stance[static_cast<std::size_t>(leg)]) {
                    std::cerr << "FAIL: first-stride coverage must keep the first walking frame all-stance\n";
                    return false;
                }
            }
        }
        constexpr std::array<int, 3> kGroupA{0, 3, 4};
        constexpr std::array<int, 3> kGroupB{1, 2, 5};
        for (const int leg : kGroupA) {
            if (!gait.in_stance[static_cast<std::size_t>(leg)]) {
                saw_group_a_swing = true;
            }
        }
        for (const int leg : kGroupB) {
            if (!gait.in_stance[static_cast<std::size_t>(leg)]) {
                saw_group_b_swing = true;
            }
        }
    }
    if (!saw_group_a_swing || !saw_group_b_swing) {
        std::cerr << "FAIL: a 0.36 s STAND→WALK burst should swing both tripod groups\n";
        return false;
    }
    if (gait.stride_phase_rate_hz.value > 1.6) {
        std::cerr << "FAIL: first-stride coverage should keep adaptive cadence, not a 2 Hz floor\n";
        return false;
    }
    return true;
}

bool testYawWalkEntryKeepsZeroPhaseSeed() {
    control_config::GaitConfig cfg{};
    GaitScheduler scheduler(cfg);
    RobotState estimated{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    BodyTwist stopped{};

    MotionIntent stand{};
    stand.requested_mode = RobotMode::STAND;
    stand.timestamp_us = TimePointUs{1'000'000};
    (void)scheduler.update(estimated, stand, safety, stopped);

    MotionIntent walk = stand;
    walk.requested_mode = RobotMode::WALK;
    walk.timestamp_us = TimePointUs{1'005'000};
    walk.cmd_yaw_radps = AngularRateRadPerSec{0.45};
    BodyTwist turn{};
    turn.angular_radps.z = 0.45;
    const GaitState entry = scheduler.update(estimated, walk, safety, turn);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if (!entry.in_stance[static_cast<std::size_t>(leg)]) {
            std::cerr << "FAIL: yaw walk-entry should begin all-stance\n";
            return false;
        }
        if (entry.phase[static_cast<std::size_t>(leg)] > 0.12) {
            std::cerr << "FAIL: yaw-dominant walk-entry should not seed stride phase\n";
            return false;
        }
    }
    return true;
}

bool testTripodPhaseGroupsSpanBothSides() {
    const GaitPresetTemplate tripod = gaitPresetTemplate(GaitType::TRIPOD);
    constexpr std::array<double, kNumLegs> kExpectedOffsets = {0.0, 0.5, 0.5, 0.0, 0.0, 0.5};
    if (tripod.phase_offset != kExpectedOffsets) {
        std::cerr << "FAIL: tripod phase groups must be {rear-left, middle-right, front-left} "
                     "and {rear-right, middle-left, front-right}\n";
        return false;
    }
    return true;
}

bool testGovernedCadenceUpdatesDurations() {
    control_config::GaitConfig cfg{};
    cfg.transition_blend_s = 0.01;
    GaitScheduler scheduler(cfg);
    RobotState estimated{};
    SafetyState safety{};
    safety.inhibit_motion = false;

    MotionIntent walk{};
    walk.requested_mode = RobotMode::WALK;
    walk.gait = GaitType::TRIPOD;
    walk.timestamp_us = TimePointUs{1'000'000};
    BodyTwist forward{};
    forward.linear_mps.x = 0.06;
    CommandGovernorState governor{};
    governor.cadence_scale = 0.6;

    (void)scheduler.update(estimated, walk, safety, forward, governor);
    walk.timestamp_us = TimePointUs{1'020'000};
    const GaitState gait = scheduler.update(estimated, walk, safety, forward, governor);
    if (!nearlyEq(gait.stance_duration_s,
                  gait.duty_factor / gait.stride_phase_rate_hz.value) ||
        !nearlyEq(gait.swing_duration_s,
                  (1.0 - gait.duty_factor) / gait.stride_phase_rate_hz.value)) {
        std::cerr << "FAIL: governor-scaled cadence should update exported stance/swing durations\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!testWalkEntryBeginsInStance() ||
        !testWalkEntryCoversBothTripodGroups() ||
        !testYawWalkEntryKeepsZeroPhaseSeed() ||
        !testTripodPhaseGroupsSpanBothSides() ||
        !testGovernedCadenceUpdatesDurations()) {
        return EXIT_FAILURE;
    }

    control_config::GaitConfig gait{};
    constexpr double kMinSwingHeightM = 0.014;
    constexpr double kTripodSwingFloorM = 0.021;
    constexpr double kRippleSwingFloorM = 0.019;
    constexpr double kWaveSwingFloorM = 0.018;

    const UnifiedGaitDescription crawl_ref =
        buildTargetUnifiedGait(GaitType::CRAWL, 0.0, 0.0, 0.0, gait, 0.0, 0.0);
    const UnifiedGaitDescription wave_ref =
        buildTargetUnifiedGait(GaitType::WAVE, 0.6, 0.0, 0.0, gait, 0.0, 0.0);
    const UnifiedGaitDescription tripod_low =
        buildAdaptiveTripodCrawlGait(0.06, 0.0, 0.0, 0.0, 0.0, gait);

    const UnifiedGaitDescription adaptive_low =
        buildAdaptiveWaveCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription adaptive_high =
        buildAdaptiveWaveCrawlGait(0.6, 0.0, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription turn_like =
        buildAdaptiveTripodCrawlGait(0.0, 0.0, 0.45, 0.0, 0.0, gait);

    if (!nearlyEq(adaptive_low.duty_factor, wave_ref.duty_factor)) {
        std::cerr << "FAIL: low-speed adaptive WAVE should match WAVE duty\n";
        return EXIT_FAILURE;
    }
    if (!nearlyEq(adaptive_high.duty_factor, wave_ref.duty_factor)) {
        std::cerr << "FAIL: high-speed adaptive WAVE should match WAVE-only duty\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_high.step_length_m > adaptive_low.step_length_m)) {
        std::cerr << "FAIL: higher-speed adaptive WAVE should increase step length\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_high.swing_height_m >= adaptive_low.swing_height_m)) {
        std::cerr << "FAIL: higher-speed adaptive WAVE should not reduce swing clearance\n";
        return EXIT_FAILURE;
    }
    if (!(adaptive_low.swing_height_m >= kMinSwingHeightM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive WAVE should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(tripod_low.swing_height_m >= kTripodSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive TRIPOD should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(crawl_ref.swing_height_m >= 0.015 - 1e-9)) {
        std::cerr << "FAIL: crawl nominal swing height should keep a modest clearance floor\n";
        return EXIT_FAILURE;
    }
    if (!(turn_like.swing_height_m >= 0.018 - 1e-9)) {
        std::cerr << "FAIL: yaw-heavy low-speed gait should lift higher to avoid turn drag\n";
        return EXIT_FAILURE;
    }
    if (!(turn_like.swing_height_m > adaptive_low.swing_height_m)) {
        std::cerr << "FAIL: yaw-heavy low-speed gait should gain more lift than still low-speed motion\n";
        return EXIT_FAILURE;
    }

    // Lateral (crabbing) walk should get more swing clearance than straight forward walking at
    // the same total planar speed. vy=0.06 m/s → lat_frac=1.0 → full oblique boost applies;
    // vx=0.06 m/s → lat_frac=0 → only the base half-boost applies.
    const UnifiedGaitDescription lateral_like =
        buildAdaptiveTripodCrawlGait(0.0, 0.06, 0.0, 0.0, 0.0, gait);
    const UnifiedGaitDescription fwd_same_speed =
        buildAdaptiveTripodCrawlGait(0.06, 0.0, 0.0, 0.0, 0.0, gait);
    if (!(lateral_like.swing_height_m > fwd_same_speed.swing_height_m)) {
        std::cerr << "FAIL: lateral low-speed gait should have higher swing floor than straight forward at same speed\n";
        return EXIT_FAILURE;
    }
    if (!(wave_ref.swing_height_m >= kWaveSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: fast WAVE should retain its swing floor\n";
        return EXIT_FAILURE;
    }
    if (!(buildAdaptiveRippleCrawlGait(0.0, 0.0, 0.0, 0.0, 0.0, gait).swing_height_m >=
          kRippleSwingFloorM - 1e-9)) {
        std::cerr << "FAIL: low-speed adaptive RIPPLE should preserve swing clearance floor\n";
        return EXIT_FAILURE;
    }

    // Scenario 05 slow-phase swing heights: assert every phase's floor sits above the physics
    // foot sphere radius (0.018 m) so the sphere bottom always clears the ground at peak swing.
    // Computed against nominal_planar_speed_mps = 0.32 m/s (default GaitConfig).
    constexpr double kPhysicsFootRadiusM = 0.018;  // kHexapodFootRadiusM in physics_sim_protocol.hpp

    struct Scenario05Case {
        GaitType gait;
        double vx_mps;
        double yaw_rate_radps;
        const char* label;
    };
    const Scenario05Case scenario05_phases[] = {
        {GaitType::TRIPOD, 0.06, 0.0, "scenario05_tripod_0.06mps"},
        {GaitType::RIPPLE, 0.07, 0.0, "scenario05_ripple_0.07mps"},
        {GaitType::WAVE,   0.05, 0.0, "scenario05_wave_0.05mps"},
        {GaitType::TRIPOD, 0.05, 0.0, "scenario05_tripod_0.05mps"},
        {GaitType::RIPPLE, 0.04, 0.0, "scenario05_ripple_0.04mps"},
    };
    for (const auto& c : scenario05_phases) {
        const UnifiedGaitDescription desc =
            buildTargetUnifiedGait(c.gait, c.vx_mps, 0.0, c.yaw_rate_radps, gait, 0.0, 0.0);
        std::cout << c.label << " swing_height_m=" << desc.swing_height_m << '\n';
        if (!(desc.swing_height_m >= kPhysicsFootRadiusM - 1e-9)) {
            std::cerr << "FAIL: " << c.label << " swing_height_m=" << desc.swing_height_m
                      << " must be >= physics foot radius " << kPhysicsFootRadiusM << '\n';
            return EXIT_FAILURE;
        }
    }

    struct Scenario05HeadingCase {
        GaitType gait;
        double speed_mps;
        double heading_rad;
        bool expect_more_than_forward;
        const char* label;
    };
    const Scenario05HeadingCase scenario05_heading_cases[] = {
        {GaitType::RIPPLE, 0.07, 1.57, true, "scenario05_ripple_lateral_heading"},
        {GaitType::WAVE, 0.05, 3.14, false, "scenario05_wave_backward_heading"},
        {GaitType::TRIPOD, 0.05, -1.57, true, "scenario05_tripod_lateral_heading"},
    };
    for (const auto& c : scenario05_heading_cases) {
        const double vx = c.speed_mps * std::cos(c.heading_rad);
        const double vy = c.speed_mps * std::sin(c.heading_rad);
        const UnifiedGaitDescription desc =
            buildTargetUnifiedGait(c.gait, vx, vy, 0.0, gait, 0.0, 0.0);
        const UnifiedGaitDescription forward_ref =
            buildTargetUnifiedGait(c.gait, c.speed_mps, 0.0, 0.0, gait, 0.0, 0.0);
        std::cout << c.label << " vx_mps=" << vx
                  << " vy_mps=" << vy
                  << " swing_height_m=" << desc.swing_height_m
                  << " forward_ref_m=" << forward_ref.swing_height_m << '\n';
        if (!(desc.swing_height_m >= kPhysicsFootRadiusM - 1e-9)) {
            std::cerr << "FAIL: " << c.label << " swing_height_m=" << desc.swing_height_m
                      << " must remain above the physics foot radius " << kPhysicsFootRadiusM << '\n';
            return EXIT_FAILURE;
        }
        if (c.expect_more_than_forward) {
            if (!(desc.swing_height_m > forward_ref.swing_height_m)) {
                std::cerr << "FAIL: " << c.label
                          << " should gain extra oblique/lateral clearance over forward motion at the same speed\n";
                return EXIT_FAILURE;
            }
        } else if (!(desc.swing_height_m + 1e-6 >= forward_ref.swing_height_m)) {
            std::cerr << "FAIL: " << c.label
                      << " should not lose swing clearance relative to same-speed forward motion\n";
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
