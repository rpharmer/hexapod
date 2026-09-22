#pragma once

#include "geometry_config.hpp"
#include "leg_link_angular_velocity.hpp"
#include "physics_sim_bridge.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>

// Test-wrapper observation only. Never changes validity, bus status or targets.
// Reconstruct at the received pose, not the simulator's pre-integration pose.
inline void auditPublishedPhysicsSimLinkSpeed(
    const RobotState& state, const std::optional<PhysicsSimSolverTelemetry>& telemetry) {
    const char* enabled = std::getenv("HEXAPOD_PHYSICS_TRACE_PUBLISHED_LINK_SPEED");
    if (!enabled || std::string{enabled} != "1" || !telemetry || !state.has_body_twist_state) return;
    const Vec3 body = state.body_twist_state.twist_vel_radps.raw();
    if (!std::isfinite(body.x) || !std::isfinite(body.y) || !std::isfinite(body.z)) return;
    double peak = 0.0;
    int peak_leg = -1, peak_joint = -1;
    const auto& geometry = geometry_config::activeHexapodGeometry();
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const auto& quality = state.joint_state_quality[leg];
        if (!quality.position_valid || !quality.velocity_valid || quality.source != JointStateSource::Simulated) return;
        std::array<double, kJointsPerLeg> rates{};
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            rates[joint] = state.leg_states[leg].joint_state[joint].vel_radps.value;
            if (!std::isfinite(rates[joint]) || !std::isfinite(state.leg_states[leg].joint_state[joint].pos_rad.value)) return;
        }
        const auto relative = legRelativeLinkAngularVelocities(geometry.legGeometry[leg], state.leg_states[leg], rates);
        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const auto& w = relative[joint];
            const double speed = std::sqrt((w.x + body.x) * (w.x + body.x)
                + (w.y + body.y) * (w.y + body.y) + (w.z + body.z) * (w.z + body.z));
            if (speed > peak) { peak = speed; peak_leg = leg; peak_joint = joint; }
        }
    }
    // Float wire rounding allowance for this diagnostic, not a new safety cap.
    if (peak <= 10.0 + 1e-5) return;
    const char* names[] = {"coxa", "femur", "tibia"};
    std::fprintf(stderr, "[published-link-speed] {\"sample\":%llu,\"frame\":\"leg_%d_%s_body\","
        "\"received_w\":%.12g,\"pre_guard_max\":%.12g,\"solver_status\":%d,\"body_angular_rate\":[%.12g,%.12g,%.12g],"
        "\"servo_positions\":[", static_cast<unsigned long long>(state.sample_id), peak_leg, names[peak_joint], peak,
        telemetry->preintegration_angular_speed, static_cast<int>(telemetry->status), body.x, body.y, body.z);
    for (int wire = 0; wire < 18; ++wire) std::fprintf(stderr, "%s%.12g", wire ? "," : "",
        state.leg_states[wire / 3].joint_state[wire % 3].pos_rad.value);
    std::fprintf(stderr, "],\"servo_rates\":[");
    for (int wire = 0; wire < 18; ++wire) std::fprintf(stderr, "%s%.12g", wire ? "," : "",
        state.leg_states[wire / 3].joint_state[wire % 3].vel_radps.value);
    std::fprintf(stderr, "]}\n");
}
