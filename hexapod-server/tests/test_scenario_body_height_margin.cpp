#include "control_config.hpp"
#include "physics_sim_protocol.hpp"
#include "scenario_driver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

Vec3 normalizeVec3(const Vec3& v) {
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n <= 1e-12) {
        return Vec3{};
    }
    return Vec3{v.x / n, v.y / n, v.z / n};
}

Vec3 legAxisFromMountAngleDegrees(const double angle_degrees) {
    const double radians = angle_degrees * 3.14159265358979323846 / 180.0;
    return normalizeVec3(Vec3{std::sin(radians), 0.0, std::cos(radians)});
}

Vec3 legPitchDirection(const Vec3& leg_axis, const double angle_radians) {
    return normalizeVec3(leg_axis * std::cos(angle_radians) + Vec3{0.0, 1.0, 0.0} * std::sin(angle_radians));
}

double computeStandingBodyHeightM() {
    constexpr Vec3 kFrontMountOffsetBody{0.063, -0.007, -0.0835};
    constexpr double kCoxaLengthM = 0.043;
    constexpr double kFemurLengthM = 0.060;
    const Vec3 leg_axis = legAxisFromMountAngleDegrees(143.0);
    const Vec3 femur_direction = legPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad);
    const Vec3 tibia_direction =
        legPitchDirection(leg_axis, physics_sim::kAssemblyFemurPitchRad + physics_sim::kAssemblyTibiaPitchRad);
    const Vec3 foot_center_relative =
        kFrontMountOffsetBody + leg_axis * kCoxaLengthM + femur_direction * kFemurLengthM +
        tibia_direction * physics_sim::kHexapodTibiaLinkLengthM;
    return std::max(0.04, static_cast<double>(physics_sim::kHexapodFootRadiusM) -
                                static_cast<double>(foot_center_relative.y) + 0.001) +
           0.002;
}

struct ScenarioStats {
    double min_motion_body_height_m{std::numeric_limits<double>::infinity()};
    double min_navigation_body_height_m{std::numeric_limits<double>::infinity()};
    std::size_t motion_count{0};
    std::size_t navigation_count{0};
};

ScenarioStats summarizeScenario(const ScenarioDefinition& scenario) {
    ScenarioStats stats{};
    for (const ScenarioEvent& event : scenario.events) {
        if (event.motion.enabled) {
            stats.min_motion_body_height_m =
                std::min(stats.min_motion_body_height_m, event.motion.body_height_m);
            ++stats.motion_count;
        }
        if (event.has_navigation_command && event.navigation.enabled) {
            stats.min_navigation_body_height_m =
                std::min(stats.min_navigation_body_height_m, event.navigation.body_height_m);
            ++stats.navigation_count;
        }
    }
    return stats;
}

} // namespace

int main() {
    const double standing_height_m = computeStandingBodyHeightM();
    const double min_safe_body_height_m =
        standing_height_m + control_config::kDefaultGovernorBodyHeightSquatMaxM - 0.001;

    const std::array<const char*, 2> scenario_paths{
        "../scenarios/01_nominal_stand_walk.toml",
        "../scenarios/05_long_walk_observability.toml",
    };

    bool ok = true;
    for (const char* path : scenario_paths) {
        ScenarioDefinition scenario{};
        std::string error;
        if (!expect(ScenarioDriver::loadFromToml(path, scenario, error),
                    std::string("scenario should parse: ") + path + (error.empty() ? "" : (" (" + error + ")")))) {
            return 1;
        }

        const ScenarioStats stats = summarizeScenario(scenario);
        if (!expect(stats.motion_count > 0, std::string("scenario should contain motion events: ") + path)) {
            ok = false;
        }

        if (stats.motion_count > 0 && !expect(stats.min_motion_body_height_m >= min_safe_body_height_m,
                                              std::string("motion body height should leave clearance: ") + path)) {
            std::cerr << "scenario=" << path
                      << " standing_height_m=" << standing_height_m
                      << " min_safe_body_height_m=" << min_safe_body_height_m
                      << " min_motion_body_height_m=" << stats.min_motion_body_height_m << '\n';
            ok = false;
        }

        if (stats.navigation_count > 0 && !expect(stats.min_navigation_body_height_m >= min_safe_body_height_m,
                                                  std::string("navigation body height should leave clearance: ") +
                                                      path)) {
            std::cerr << "scenario=" << path
                      << " standing_height_m=" << standing_height_m
                      << " min_safe_body_height_m=" << min_safe_body_height_m
                      << " min_navigation_body_height_m=" << stats.min_navigation_body_height_m << '\n';
            ok = false;
        }

        std::cout << path << " standing_height_m=" << standing_height_m
                  << " min_safe_body_height_m=" << min_safe_body_height_m
                  << " min_motion_body_height_m="
                  << (stats.motion_count > 0 ? stats.min_motion_body_height_m : -1.0)
                  << " min_navigation_body_height_m="
                  << (stats.navigation_count > 0 ? stats.min_navigation_body_height_m : -1.0)
                  << '\n';
    }

    return ok ? 0 : 1;
}
