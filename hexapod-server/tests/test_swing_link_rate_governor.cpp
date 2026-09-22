#include "config/geometry_config.hpp"
#include "control/servo_dynamics_clamp.hpp"
#include "control/swing_link_rate_governor.hpp"
#include "hardware/physics_sim_bridge.hpp"
#include "hardware/sim_hardware_bridge.hpp"
#include "stored_motion_experiment.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <random>

namespace {
bool expect(bool value, const char* text) {
    if (!value) std::cerr << "FAIL: " << text << '\n';
    return value;
}
bool identical(const JointTargets& a, const JointTargets& b) {
    for (int leg = 0; leg < kNumLegs; ++leg) for (int joint = 0; joint < 3; ++joint) {
        const auto& x = a.leg_states[leg].joint_state[joint];
        const auto& y = b.leg_states[leg].joint_state[joint];
        if (x.pos_rad.value != y.pos_rad.value || x.vel_radps.value != y.vel_radps.value) return false;
    }
    return true;
}
}

int main() {
    auto geometry = geometry_config::buildDefaultHexapodGeometry();
    bool ok = true;
    {
        physics_sim_test_utils::StoredMotionExperiment limiter(.005);
        RobotState live{};
        live.bus_ok = true;
        live.has_body_twist_state = true;
        JointTargets command{};
        for (int l=0; l<kNumLegs; ++l) {
            live.joint_state_quality[l].position_valid = true;
            live.joint_state_quality[l].source = JointStateSource::Simulated;
            command.leg_states[l].joint_state[FEMUR].pos_rad.value =
                std::copysign(.6, geometry.legGeometry[l].servo.femurSign);
            command.leg_states[l].joint_state[TIBIA].pos_rad.value =
                std::copysign(.6, geometry.legGeometry[l].servo.tibiaSign);
        }
        ok &= expect(identical(limiter.apply(command, &live, geometry, false), command),
                     "disabled stored-motion experiment is exactly unchanged");
        const auto bounded = limiter.apply(command, &live, geometry, true);
        for (int l=0; l<kNumLegs; ++l) {
            std::array<double, 3> rates{};
            for (int j=0; j<3; ++j) rates[j] = bounded.leg_states[l].joint_state[j].pos_rad.value / .08;
            ok &= expect(swingLinkPeakRadps(geometry.legGeometry[l], live.leg_states[l], rates, {}) <= 10.0+1e-12,
                         "stored error projects composed request for every mirrored leg");
            ok &= expect(std::abs(std::abs(bounded.leg_states[l].joint_state[FEMUR].pos_rad.value)-.4) < 1e-12,
                         "stored-error projection preserves direction rather than zeroing torque");
        }
        const auto held = limiter.apply(command, &live, geometry, true);
        ok &= expect(held.leg_states[0].joint_state[FEMUR].vel_radps.value == 0,
                     "unchanged projected command reports zero metadata velocity");
        live.bus_ok = false;
        ok &= expect(identical(limiter.apply(command, &live, geometry, true), command),
                     "invalid feedback does not shape stored error");
    }
    SimHardwareBridge simple_sim{};
    PhysicsSimBridge physics_sim{"127.0.0.1", 1, 5000, PhysicsSimSolverSettings{}};
    ok &= expect(!simple_sim.usesPhysicsSimBodyAngularConvention() && physics_sim.usesPhysicsSimBodyAngularConvention(),
                 "explicit convention capability excludes the simple simulator");
    LegState measured{};
    const auto& leg = geometry.legGeometry[0];
    // Femur and tibia cancel at tibia; femur itself is still checked.
    auto result = projectSwingLinkRates(leg, measured, {{0.0, 7.48, -7.48}}, {}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Unchanged && vecNorm(result.predicted_after[TIBIA]) == 0.0,
                 "opposing pitch rates cancel without unnecessary reduction");
    result = projectSwingLinkRates(leg, measured, {{0.0, 7.48, 7.48}}, {}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Limited && result.limiting_link == TIBIA
                 && std::abs(result.scale - 10.0 / 14.96) < 1e-12, "reinforcing rates get one common exact scale");
    result = projectSwingLinkRates(leg, measured, {{0.0, 0.0, 0.0}}, {0.0, 0.0, 10.1}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Infeasible && result.scale == 1.0, "over-budget base retains baseline");
    result = projectSwingLinkRates(leg, measured, {{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}}, {}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Unavailable, "non-finite rate rejected");
    result = projectSwingLinkRates(leg, measured, {{1.0, 0.0, 0.0}}, {0.0, 0.0, -10.0}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Limited && result.scale == 0.0, "boundary outward rate has zero feasible scale");
    result = projectSwingLinkRates(leg, measured, {{1.0, 0.0, 0.0}}, {0.0, 0.0, 10.0}, 10.0);
    ok &= expect(result.status == SwingLinkRateStatus::Unchanged, "boundary inward rate remains available");
    std::mt19937 random(20260916);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    for (int sample = 0; sample < 1000; ++sample) {
        const auto& g = geometry.legGeometry[sample % 6];
        for (auto& joint : measured.joint_state) joint.pos_rad = AngleRad{unit(random)};
        const Vec3 base{4 * unit(random), 4 * unit(random), 4 * unit(random)};
        const std::array<double, 3> rates{{7.48 * unit(random), 7.48 * unit(random), 7.48 * unit(random)}};
        result = projectSwingLinkRates(g, measured, rates, base, 10.0);
        for (const auto& velocity : result.predicted_after) ok &= expect(vecNorm(velocity) <= 10.0 + 1e-12, "random projection stays feasible");
        if (result.status == SwingLinkRateStatus::Limited) {
            const auto relative = legRelativeLinkAngularVelocities(g, measured, rates);
            ok &= expect(vecNorm(base + relative[result.limiting_link] * (result.scale + 1e-7)) > 10.0,
                         "chosen scale is maximal");
        }
    }
    RobotState raw{};
    raw.has_body_twist_state = true;
    for (auto& quality : raw.joint_state_quality) { quality.position_valid = true; quality.source = JointStateSource::Simulated; }
    std::array<bool, 6> stance{};
    stance[1] = true;
    JointTargets previous{}, requested{};
    for (auto& l : previous.leg_states) {
        l.joint_state[FEMUR].pos_rad = AngleRad{1.0};
        l.joint_state[TIBIA].pos_rad = AngleRad{1.0};
    }
    for (auto& l : requested.leg_states) {
        l.joint_state[FEMUR].pos_rad = AngleRad{1.5};
        l.joint_state[TIBIA].pos_rad = AngleRad{1.5};
    }
    for (double dt : {1.0/120.0, 1.0/240.0, 1.0/480.0}) {
        const auto slew = clampJointTargetsToServoDynamics(previous, requested, geometry, dt);
        auto governed = governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, dt, true);
        ok &= expect(governed.legs[0].status == SwingLinkRateStatus::Limited && governed.legs[1].status == SwingLinkRateStatus::Inactive,
                     "only swing is governed");
        ok &= expect(governed.targets.leg_states[1].joint_state[FEMUR].pos_rad.value == slew.targets.leg_states[1].joint_state[FEMUR].pos_rad.value,
                     "stance is unchanged");
        ok &= expect(std::abs(governed.targets.leg_states[0].joint_state[FEMUR].vel_radps.value - 5.0) < 1e-12,
                     "cadences yield equivalent coordinated rates");
        ok &= expect(std::abs(governed.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - (1.0 + 5.0 * dt)) < 1e-12,
                     "target increments stay anchored to previous command, not measured position");
        ok &= expect(identical(governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, dt, false).targets, slew.targets),
                     "disabled path is exactly unchanged");
        ok &= expect(identical(governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::STAND, stance, dt, true).targets, slew.targets),
                     "STAND and recovery are unchanged");
        raw.has_body_twist_state = false;
        governed = governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, dt, true);
        ok &= expect(identical(governed.targets, slew.targets) && governed.legs[0].status == SwingLinkRateStatus::Unavailable, "missing state keeps baseline");
        raw.has_body_twist_state = true;
        raw.joint_state_quality[0].source = JointStateSource::Measured;
        governed = governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, dt, true);
        ok &= expect(governed.legs[0].status == SwingLinkRateStatus::Unavailable
                     && governed.targets.leg_states[0].joint_state[FEMUR].pos_rad.value == slew.targets.leg_states[0].joint_state[FEMUR].pos_rad.value,
                     "hardware provenance is never shaped");
        raw.joint_state_quality[0].source = JointStateSource::Simulated;
        governed = governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, 0.0, true);
        ok &= expect(identical(governed.targets, slew.targets), "zero timestep retains baseline");
        raw.bus_ok = false;
        governed = governSwingLinkRates(previous, slew.targets, geometry, raw, RobotMode::WALK, stance, dt, true);
        ok &= expect(identical(governed.targets, slew.targets), "failed bus retains baseline");
        raw.bus_ok = true;
    }

    raw.has_body_twist_state = true;
    raw.bus_ok = true;
    raw.joint_state_quality[0].position_valid = true;
    raw.joint_state_quality[0].velocity_valid = true;
    raw.joint_state_quality[0].source = JointStateSource::Simulated;
    JointTargets far{};
    far.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{1.5};
    raw.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.4};
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    auto near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Limited
                 && std::abs(near.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.4) < 1e-12,
                 "near-cap measured femur rate snaps the swing target to live");
    ok &= expect(near.targets.leg_states[1].joint_state[FEMUR].pos_rad.value == far.leg_states[1].joint_state[FEMUR].pos_rad.value,
                 "near-cap snap does not move stance");
    // The last angle intervention must also update the emitted rate. Otherwise
    // an observer/feedforward consumer sees the pre-snap motion request.
    for (double dt : {1.0/120.0, 0.005, 1.0/240.0, 1.0/480.0}) {
        auto emitted = near.targets;
        ok &= expect(refreshJointTargetVelocities(previous, emitted, dt), "valid final targets have rates");
        ok &= expect(std::abs(emitted.leg_states[0].joint_state[FEMUR].vel_radps.value
                     - (0.4 - 1.0) / dt) < 1e-10,
                     "post-snap velocity describes final angle difference, not stale slew");
        for (int l = 0; l < kNumLegs; ++l) for (int j = 0; j < kJointsPerLeg; ++j)
            ok &= expect(emitted.leg_states[l].joint_state[j].pos_rad.value
                         == near.targets.leg_states[l].joint_state[j].pos_rad.value,
                         "metadata repair does not modify any angle");
        const auto same = emitted;
        ok &= expect(refreshJointTargetVelocities(same, emitted, dt)
                     && emitted.leg_states[0].joint_state[FEMUR].vel_radps.value == 0.0,
                     "holding a final target reports zero rate");
        ok &= expect(refreshJointTargetVelocities(near.targets, emitted = previous, dt)
                     && std::abs(emitted.leg_states[0].joint_state[FEMUR].vel_radps.value - 0.6/dt) < 1e-10,
                     "target reversal reports opposite signed rate");
    }
    auto invalid_rates = near.targets;
    for (double dt : {0.0, -0.005, std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()})
        ok &= expect(!refreshJointTargetVelocities(previous, invalid_rates, dt)
                     && identical(invalid_rates, near.targets), "invalid cadence is atomic no-op");
    auto invalid_previous = previous;
    invalid_previous.leg_states[5].joint_state[TIBIA].pos_rad.value = std::numeric_limits<double>::quiet_NaN();
    ok &= expect(!refreshJointTargetVelocities(invalid_previous, invalid_rates, .005)
                 && identical(invalid_rates, near.targets), "invalid late joint cannot leave partial metadata");
    auto wrap_before = previous, wrap_after = previous;
    wrap_before.leg_states[0].joint_state[COXA].pos_rad.value = 3.1;
    wrap_after.leg_states[0].joint_state[COXA].pos_rad.value = -3.1;
    ok &= expect(refreshJointTargetVelocities(wrap_before, wrap_after, .005)
                 && std::abs(wrap_after.leg_states[0].joint_state[COXA].vel_radps.value + 1240.0) < 1e-10,
                 "metadata does not hide a discontinuous unwrapped command");
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{7.0};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Unchanged
                 && std::abs(near.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 1.5) < 1e-12,
                 "healthy 7 rad/s swing must not snap (that is the rejected always-on governor)");
    raw.joint_state_quality[0].source = JointStateSource::Measured;
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Unavailable
                 && std::abs(near.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 1.5) < 1e-12,
                 "hardware provenance is never snapped");
    raw.joint_state_quality[0].source = JointStateSource::Simulated;
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::STAND, stance, true);
    ok &= expect(identical(near.targets, far), "STAND is unchanged");
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, false);
    ok &= expect(identical(near.targets, far), "disabled near-cap path is exactly unchanged");

    raw.joint_state_quality[0].source = JointStateSource::Simulated;
    raw.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.4};
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    JointTargets first_snap{};
    for (double dt : {1.0 / 120.0, 1.0 / 240.0, 1.0 / 480.0}) {
        (void)dt;
        auto snapped = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
        ok &= expect(snapped.legs[0].status == SwingLinkRateStatus::Limited
                     && snapped.legs[1].status == SwingLinkRateStatus::Inactive,
                     "near-cap fire is independent of control dt (snap is not ω·dt)");
        ok &= expect(std::abs(snapped.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.4) < 1e-12,
                     "near-cap snap is to the live angle, not a rate increment");
        ok &= expect(snapped.targets.leg_states[1].joint_state[FEMUR].pos_rad.value
                         == far.leg_states[1].joint_state[FEMUR].pos_rad.value,
                     "stance legs stay at the requested command under snap characterization");
        const double jump = snapped.targets.leg_states[0].joint_state[FEMUR].pos_rad.value
            - far.leg_states[0].joint_state[FEMUR].pos_rad.value;
        ok &= expect(std::abs(jump - (0.4 - 1.5)) < 1e-12,
                     "snapped femur discontinuity equals live minus requested, independent of dt");
        if (first_snap.leg_states[0].joint_state[FEMUR].pos_rad.value == 0.0
            && first_snap.leg_states[0].joint_state[TIBIA].pos_rad.value == 0.0) {
            first_snap = snapped.targets;
        } else {
            ok &= expect(identical(snapped.targets, first_snap),
                         "dt 1/120, 1/240, 1/480 produce identical snap targets");
        }
    }
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.785};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Unchanged
                 && std::abs(near.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 1.5) < 1e-12,
                 "peak 9.785 is under kSwingLinkNearCapRadps and must not snap");
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.787};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Limited
                 && std::abs(near.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 0.4) < 1e-12,
                 "peak 9.787 is over kSwingLinkNearCapRadps and must snap");
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.785};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, stance, true);
    ok &= expect(near.legs[0].status == SwingLinkRateStatus::Unchanged,
                 "existing near-cap gate does not need extra hysteresis: 9.787 then 9.785 is fire then no-fire");
    std::array<bool, 6> loaded_stance = stance;
    loaded_stance[0] = true;
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    near = snapSwingTargetsNearMeasuredLinkCap(far, geometry, raw, RobotMode::WALK, loaded_stance, true);
    ok &= expect(identical(near.targets, far) && near.legs[0].status == SwingLinkRateStatus::Inactive,
                 "planned_stance skip is unchanged: gait stance is not snapped");

    raw.joint_state_quality[0].source = JointStateSource::Simulated;
    raw.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.1};
    raw.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{0.4};
    raw.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{0.4};
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    raw.leg_states[0].joint_state[TIBIA].vel_radps = AngularRateRadPerSec{0.0};
    JointTargets dump_analog = far;
    dump_analog.leg_states[0].joint_state[COXA].pos_rad = AngleRad{0.8};
    dump_analog.leg_states[0].joint_state[FEMUR].pos_rad = AngleRad{1.5};
    dump_analog.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{1.03};
    auto tibia = clampNearCapTibiaTowardMeasured(
        dump_analog, geometry, raw, RobotMode::WALK, true);
    ok &= expect(tibia.legs[0].status == SwingLinkRateStatus::Limited
                 && tibia.legs[0].limiting_link == TIBIA
                 && std::abs(tibia.targets.leg_states[0].joint_state[TIBIA].pos_rad.value - 0.65) < 1e-12,
                 "near-cap tibia remainder binds dump-analog 0.63 error to 0.25");
    ok &= expect(std::abs(tibia.targets.leg_states[0].joint_state[COXA].pos_rad.value - 0.8) < 1e-12
                 && std::abs(tibia.targets.leg_states[0].joint_state[FEMUR].pos_rad.value - 1.5) < 1e-12,
                 "near-cap tibia remainder does not move coxa or femur");
    ok &= expect(tibia.targets.leg_states[1].joint_state[TIBIA].pos_rad.value
                     == dump_analog.leg_states[1].joint_state[TIBIA].pos_rad.value,
                 "near-cap tibia remainder is per-leg");
    dump_analog.leg_states[0].joint_state[COXA].vel_radps = AngularRateRadPerSec{11.0};
    dump_analog.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{11.0};
    dump_analog.leg_states[0].joint_state[TIBIA].vel_radps = AngularRateRadPerSec{11.0};
    tibia = clampNearCapTibiaTowardMeasured(dump_analog, geometry, raw, RobotMode::WALK, true);
    ok &= expect(tibia.legs[0].status == SwingLinkRateStatus::Unchanged
                 && std::abs(tibia.targets.leg_states[0].joint_state[TIBIA].pos_rad.value - 1.03) < 1e-12,
                 "command-at-q over the near-cap does not bind tibia remainder");
    dump_analog.leg_states[0].joint_state[COXA].vel_radps = AngularRateRadPerSec{0.0};
    dump_analog.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{0.0};
    dump_analog.leg_states[0].joint_state[TIBIA].vel_radps = AngularRateRadPerSec{0.0};
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{7.0};
    tibia = clampNearCapTibiaTowardMeasured(dump_analog, geometry, raw, RobotMode::WALK, true);
    ok &= expect(tibia.legs[0].status == SwingLinkRateStatus::Unchanged
                 && std::abs(tibia.targets.leg_states[0].joint_state[TIBIA].pos_rad.value - 1.03) < 1e-12,
                 "healthy 7 rad/s measured peak must not bind tibia remainder");
    raw.leg_states[0].joint_state[FEMUR].vel_radps = AngularRateRadPerSec{9.9};
    raw.joint_state_quality[0].source = JointStateSource::Measured;
    tibia = clampNearCapTibiaTowardMeasured(dump_analog, geometry, raw, RobotMode::WALK, true);
    ok &= expect(tibia.legs[0].status == SwingLinkRateStatus::Unavailable
                 && std::abs(tibia.targets.leg_states[0].joint_state[TIBIA].pos_rad.value - 1.03) < 1e-12,
                 "hardware provenance is never tibia-remainder clipped");
    raw.joint_state_quality[0].source = JointStateSource::Simulated;
    tibia = clampNearCapTibiaTowardMeasured(dump_analog, geometry, raw, RobotMode::STAND, true);
    ok &= expect(identical(tibia.targets, dump_analog), "STAND is unchanged by tibia remainder");
    tibia = clampNearCapTibiaTowardMeasured(dump_analog, geometry, raw, RobotMode::WALK, false);
    ok &= expect(identical(tibia.targets, dump_analog), "disabled tibia remainder is exactly unchanged");

    // Contact-aware liftoff. Bound is derived, so assert the derivation rather
    // than a literal: a single joint may store guard/(0.3679 ωn) = 1.09 rad, and
    // three aligned joints of one leg divide that.
    ok &= expect(std::abs(kLoadedSwingTrackingErrorRad
                          - kSpeedGuardLinkAngularRadps
                              / (3.0 * kServoCriticalPeakVelocityGain
                                 * hexapod_dynamics::kServoOmegaN))
                     < 1e-15,
                 "loaded-swing bound is the servo-model value, not a tuned constant");
    ok &= expect(std::abs(kSpeedGuardLinkAngularRadps
                          / (kServoCriticalPeakVelocityGain * hexapod_dynamics::kServoOmegaN)
                          - 1.0874)
                     < 1e-3,
                 "single-joint form reproduces the 1.08 rad observed at the default-straight trip");

    JointTargets loaded{};
    RobotState est{};
    std::array<bool, kNumLegs> swing_all{};
    std::array<bool, kNumLegs> contact_all{};
    for (int i = 0; i < kNumLegs; ++i) {
        est.joint_state_quality[i].position_valid = true;
        contact_all[i] = true;
    }
    // Winner analogue from support-divergence-default-straight-v1: planned swing,
    // still in raw contact, 1.03 rad of tibia error waiting to discharge.
    est.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{0.2};
    loaded.leg_states[0].joint_state[TIBIA].pos_rad = AngleRad{1.23};
    est.leg_states[1].joint_state[TIBIA].pos_rad = AngleRad{0.2};
    loaded.leg_states[1].joint_state[TIBIA].pos_rad = AngleRad{1.23};
    auto hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, contact_all, true);
    ok &= expect(hold.legs[0].status == SwingLinkRateStatus::Limited
                 && hold.legs[0].limiting_link == TIBIA
                 && std::abs(hold.targets.leg_states[0].joint_state[TIBIA].pos_rad.value
                             - (0.2 + kLoadedSwingTrackingErrorRad))
                     < 1e-12,
                 "loaded planned-swing leg keeps only the derived error bound");
    ok &= expect(hold.targets.leg_states[0].joint_state[TIBIA].pos_rad.value
                     > est.leg_states[0].joint_state[TIBIA].pos_rad.value,
                 "bound keeps lifting torque in the swing direction, it is not a freeze to live");

    std::array<bool, kNumLegs> no_contact{};
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, no_contact, true);
    ok &= expect(identical(hold.targets, loaded)
                 && hold.legs[0].status == SwingLinkRateStatus::Inactive,
                 "an airborne swing leg is untouched: the discharge only matters against the floor");

    std::array<bool, kNumLegs> stance_all{};
    for (int i = 0; i < kNumLegs; ++i) stance_all[i] = true;
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, stance_all, contact_all, true);
    ok &= expect(identical(hold.targets, loaded)
                 && hold.legs[0].status == SwingLinkRateStatus::Inactive,
                 "loaded stance is never bound: that was the reverted measured-unload snap");

    loaded.leg_states[2].joint_state[TIBIA].pos_rad = AngleRad{0.2};
    est.leg_states[2].joint_state[TIBIA].pos_rad = AngleRad{0.2};
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, contact_all, true);
    ok &= expect(hold.legs[2].status == SwingLinkRateStatus::Unchanged
                 && hold.targets.leg_states[2].joint_state[TIBIA].pos_rad.value == 0.2,
                 "a tracking leg inside the bound is reported unchanged and not moved");

    est.joint_state_quality[3].position_valid = false;
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, contact_all, true);
    ok &= expect(hold.legs[3].status == SwingLinkRateStatus::Unavailable,
                 "invalid measured position cannot be used as a live reference");
    est.joint_state_quality[3].position_valid = true;

    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::STAND, swing_all, contact_all, true);
    ok &= expect(identical(hold.targets, loaded), "STAND is unchanged by the loaded-swing bound");
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, contact_all, false);
    ok &= expect(identical(hold.targets, loaded), "disabled loaded-swing bound is exactly unchanged");
    hold = clampLoadedSwingTargetsTowardMeasured(
        loaded, est, RobotMode::WALK, swing_all, contact_all, true, 0.0);
    ok &= expect(identical(hold.targets, loaded), "non-positive bound is a no-op, not a freeze");
    return ok ? 0 : 1;
}
