#include "minphys3d/demo/pinocchio_hexapod.hpp"
#include "config/geometry_config.hpp"
#include "control/swing_link_rate_governor.hpp"
#include "physics_sim_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <random>
#include <vector>

int main() {
    using namespace minphys3d::demo;
    minphys3d::World world({0.0, 0.0, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    std::array<std::uint32_t, 18> joints{};
    std::array<double, 18> zero{};
    for (int leg = 0; leg < 6; ++leg) {
        joints[3 * leg] = scene.legs[leg].bodyToCoxaJoint;
        joints[3 * leg + 1] = scene.legs[leg].coxaToFemurJoint;
        joints[3 * leg + 2] = scene.legs[leg].femurToTibiaJoint;
    }
    for (int wire = 0; wire < 18; ++wire) zero[wire] = world.GetServoJointAngle(joints[wire]);
    PinocchioHexapodModel model(world, scene, joints);
    const HexapodGeometry geometry = geometry_config::buildDefaultHexapodGeometry();
    std::vector<double> q, v;
    if (!model.readState(world, q, v)) return 1;
    const auto initial_q = q;
    std::mt19937 generator(0x9162026);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    double worst = 0.0;
    for (int sample = 0; sample < 1000; ++sample) {
        double norm = 0.0;
        for (int i = 3; i < 7; ++i) { q[i] = unit(generator); norm += q[i] * q[i]; }
        for (int i = 3; i < 7; ++i) q[i] /= std::sqrt(norm);
        for (int i = 0; i < 6; ++i) v[i] = 4.0 * unit(generator);
        for (int i = 0; i < 18; ++i) { q[7 + i] = 1.5 * unit(generator); v[6 + i] = 8.0 * unit(generator); }
        // Include exact femur/tibia cancellation, not merely random near-cancellation.
        if (sample % 3 == 0) for (int leg = 0; leg < 6; ++leg) v[6 + 3 * leg + 2] = -v[6 + 3 * leg + 1];
        if (!model.writeState(world, q, v)) return 1;
        const auto& chassis = world.GetBody(scene.body);
        const auto body_to_server = [&chassis](const minphys3d::Vec3& angular_world) {
            const auto local = minphys3d::Rotate(minphys3d::Conjugate(chassis.orientation), angular_world);
            return ::Vec3{-local.z, local.x, local.y};
        };
        const ::Vec3 base = body_to_server(chassis.angularVelocity);
        auto projected_v = v;
        for (int leg = 0; leg < 6; ++leg) {
            const auto& calibration = geometry.legGeometry[leg].servo;
            LegState mechanical{};
            mechanical.joint_state[COXA].pos_rad = AngleRad{physics_sim::kWireZeroCoxaMechanicalRad - zero[3 * leg] - q[7 + 3 * leg]};
            mechanical.joint_state[FEMUR].pos_rad = AngleRad{physics_sim::kWireZeroFemurMechanicalRad + zero[3 * leg + 1] + q[7 + 3 * leg + 1]};
            mechanical.joint_state[TIBIA].pos_rad = AngleRad{physics_sim::kWireZeroTibiaMechanicalRad + zero[3 * leg + 2] + q[7 + 3 * leg + 2]};
            const LegState servo = calibration.toServoAngles(mechanical);
            const std::array<double, 3> rates{{-calibration.coxaSign * v[6 + 3 * leg], calibration.femurSign * v[6 + 3 * leg + 1], calibration.tibiaSign * v[6 + 3 * leg + 2]}};
            const auto relative = legRelativeLinkAngularVelocities(geometry.legGeometry[leg], servo, rates);
            const std::array<std::uint32_t, 3> links{{scene.legs[leg].coxa, scene.legs[leg].femur, scene.legs[leg].tibia}};
            for (int link = 0; link < 3; ++link) {
                const ::Vec3 expected = base + relative[link];
                const ::Vec3 actual = body_to_server(world.GetBody(links[link]).angularVelocity);
                const double error = vecNorm(expected - actual);
                worst = std::max(worst, error);
                if (error > 1e-9 * std::max(1.0, vecNorm(actual))) {
                    std::cerr << "mapping failure sample=" << sample << " leg=" << leg << " link=" << link
                              << " error=" << error << " expected=" << expected.x << ',' << expected.y << ',' << expected.z
                              << " actual=" << actual.x << ',' << actual.y << ',' << actual.z << '\n';
                    return 1;
                }
            }
            const auto projection = projectSwingLinkRates(geometry.legGeometry[leg], servo, rates, base, 10.0);
            if (projection.status != SwingLinkRateStatus::Limited && projection.status != SwingLinkRateStatus::Unchanged) {
                std::cerr << "valid oracle state rejected by projection\n";
                return 1;
            }
            for (int joint = 0; joint < 3; ++joint) projected_v[6 + 3 * leg + joint] *= projection.scale;
        }
        if (!model.writeState(world, q, projected_v)) return 1;
        for (const auto& leg : scene.legs) {
            for (const auto id : {leg.coxa, leg.femur, leg.tibia}) {
                if (minphys3d::Length(world.GetBody(id).angularVelocity) > 10.0 + 1e-9) {
                    std::cerr << "projected rate fails actual Pinocchio link envelope\n";
                    return 1;
                }
            }
        }
    }
    std::cout << "1000 poses, 18000 link/wire comparisons; worst angular error=" << worst << '\n';

    // Diagnosis-only: a bound at q_old does not bound transported velocity at
    // q_new. Use the real rig, legal individual rates, and the angular part of
    // free-flyer/revolute integration; translation does not affect angular J.
    q = initial_q;
    std::fill(v.begin(), v.end(), 0.0);
    const double yaw = legFrameYawRad(geometry.legGeometry[0]);
    const ::Vec3 base{-std::cos(yaw), -std::sin(yaw), 0.0};
    v[3] = base.y; v[4] = base.z; v[5] = -base.x; // C^-1 bridge body vector
    v[6] = -5.0;
    v[7] = v[8] = 0.5 * std::sqrt((10.0 - 1e-6) * (10.0 - 1e-6) - 26.0);
    if (!model.writeState(world, q, v)) return 1;
    const double speed_old = minphys3d::Length(world.GetBody(scene.legs[0].tibia).angularVelocity);
    minphys3d::demo::ProximalSolverSettings guard_settings{};
    minphys3d::demo::ProximalStepDiagnostics guard_diagnostics{};
    if (!model.validateDynamicState(q, v, guard_settings, guard_diagnostics)) {
        std::cerr << "old pose must satisfy full dynamic validation\n";
        return 1;
    }
    const auto old_q = q, old_v = v;
    for (const auto& leg : scene.legs) {
        for (const auto id : {leg.coxa, leg.femur, leg.tibia}) {
            if (minphys3d::Length(world.GetBody(id).angularVelocity) >= 10.0) {
                std::cerr << "guard-gap example must start below every link angular bound\n";
                return 1;
            }
        }
    }
    constexpr double dt = 1.0 / 600.0;
    const double angular_norm = std::sqrt(v[3]*v[3] + v[4]*v[4] + v[5]*v[5]);
    const double half_angle = 0.5 * angular_norm * dt;
    const minphys3d::Quat rotation_delta{std::cos(half_angle), v[3] * std::sin(half_angle) / angular_norm,
        v[4] * std::sin(half_angle) / angular_norm, v[5] * std::sin(half_angle) / angular_norm};
    const minphys3d::Quat old_rotation{q[6], q[3], q[4], q[5]};
    const auto new_rotation = old_rotation * rotation_delta;
    q[3] = new_rotation.x; q[4] = new_rotation.y; q[5] = new_rotation.z; q[6] = new_rotation.w;
    for (int wire = 0; wire < 18; ++wire) q[7 + wire] += dt * v[6 + wire];
    guard_diagnostics = {};
    if (model.writeValidatedState(world, q, v, guard_settings, guard_diagnostics)
        || guard_diagnostics.failureReason != minphys3d::demo::ProximalFailureReason::SpeedLimit
        || guard_diagnostics.speedLimitFrame != minphys3d::demo::ProximalSpeedLimitFrame::Tibia) {
        std::cerr << "integrated guard-gap candidate was not rejected\n";
        return 1;
    }
    std::vector<double> untouched_q, untouched_v;
    if (!model.readState(world, untouched_q, untouched_v)) return 1;
    for (std::size_t i = 0; i < old_q.size(); ++i) {
        if (std::abs(untouched_q[i] - old_q[i]) > 1e-9) return 1;
    }
    for (std::size_t i = 0; i < old_v.size(); ++i) {
        if (std::abs(untouched_v[i] - old_v[i]) > 1e-9) return 1;
    }
    // Raw model mapping deliberately bypasses the guard only for this oracle.
    if (!model.writeState(world, q, v)) return 1;
    const double speed_new = minphys3d::Length(world.GetBody(scene.legs[0].tibia).angularVelocity);
    if (!(speed_old < 10.0 && speed_new > 10.0)) {
        std::cerr << "failed to reproduce configuration-transport guard gap old=" << speed_old << " new=" << speed_new << '\n';
        return 1;
    }
    std::cout << std::setprecision(12) << "configuration transport, unchanged tangent velocity: old=" << speed_old << " new=" << speed_new
              << " (old-pose angular guard alone is insufficient)\n";
}
