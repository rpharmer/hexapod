#include "hexapod_dynamics_constants.hpp"
#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"
#include "locomotion_command.hpp"
#include "motion_intent_utils.hpp"
#include "physics_sim_bridge.hpp"
#include "physics_sim_test_argv.hpp"
#include "physics_sim_protocol.hpp"
#include "physics_sim_test_utils.hpp"
#include "stance_progress_metrics.hpp"
#include "twist_field.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#if defined(__linux__)
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

int positiveEnvOrDefault(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const int parsed = std::atoi(value);
    return parsed > 0 ? parsed : fallback;
}

double positiveDoubleEnvOrDefault(const char* name, const double fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    const double parsed = std::atof(value);
    return parsed > 0.0 ? parsed : fallback;
}

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

// Internal order is R3, L3, R2, L2, R1, L1. Raise one alternating tripod.
constexpr std::array<int, 3> kRaisedLegs{{0, 3, 4}};
constexpr std::array<bool, kNumLegs> kSupportLegs{{false, true, true, false, false, true}};

BodyPose makeBodyPose(const RobotState& state) {
    BodyPose pose{};
    pose.position = state.body_twist_state.body_trans_m;
    pose.roll = AngleRad{state.body_twist_state.twist_pos_rad.x};
    pose.pitch = AngleRad{state.body_twist_state.twist_pos_rad.y};
    pose.yaw = AngleRad{state.body_twist_state.twist_pos_rad.z};
    return pose;
}

LegTargets standFootTargets() {
    BodyController body{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);
    MotionIntent stand = makeMotionIntent(RobotMode::STAND, GaitType::TRIPOD, 0.14);
    GaitState gait{};
    const BodyTwist cmd_twist = rawLocomotionTwistFromIntent(stand, planarMotionCommand(stand));
    return body.update(est, stand, gait, safety, cmd_twist, nullptr);
}

// Stand Cartesian is body-frame and assumes ~0.14 m height. After sag, a body-frame
// +Z raise still plants on the 18 mm sphere. Lift in world Z from measured XY.
constexpr double kFootRadiusM = static_cast<double>(physics_sim::kHexapodFootRadiusM);
constexpr double kHexapodMassKg =
    hexapod_dynamics::kBodyMassKg
    + 6.0 * (hexapod_dynamics::kCoxaMassKg + hexapod_dynamics::kFemurMassKg
             + hexapod_dynamics::kTibiaMassKg + hexapod_dynamics::kFootMassKg);
constexpr double kRaisedClearanceM = 0.040;

Vec3 worldToBody(const BodyPose& pose, const Vec3& world) {
    return pose.rotationBodyToWorld().transpose() * (world - pose.position.raw());
}

Mat3 simToServerC() {
    Mat3 c{};
    c.m[0][2] = -1.0;
    c.m[1][0] = 1.0;
    c.m[2][1] = 1.0;
    return c;
}

Vec3 serverWorldToSim(const Vec3& srv) {
    return Vec3{srv.y, srv.z, -srv.x};
}

// Chassis-relative sphere in sim body axes. Sphere sim-Y is not on the wire;
// using the chassis sim-Y makes relative Y 0, which leaks <1 mm into XY at
// the observed stand pitch.
Vec3 chassisRelativeSphereSim(const BodyPose& pose, const double sim_foot_x, const double sim_foot_z) {
    const Vec3 p_sim = serverWorldToSim(pose.position.raw());
    const Vec3 sphere_sim{sim_foot_x, p_sim.y, sim_foot_z};
    const Mat3 C = simToServerC();
    const Mat3 R_sim = C.transpose() * pose.rotationBodyToWorld() * C;
    return R_sim.transpose() * (sphere_sim - p_sim);
}

void assignFeetWorldXY(LegTargets* feet,
                       const RobotState& est,
                       const std::array<Vec3, kNumLegs>& world_xy,
                       const std::array<bool, kNumLegs>& mask,
                       const double target_world_z) {
    const BodyPose pose = makeBodyPose(est);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        if (!mask[idx]) {
            continue;
        }
        const Vec3 desired_world{world_xy[idx].x, world_xy[idx].y, target_world_z};
        feet->feet[leg].pos_body_m = worldToBody(pose, desired_world);
        feet->feet[leg].vel_body_mps = Vec3{};
    }
}

void assignRaisedFeetWorldZ(LegTargets* feet,
                            const RobotState& est,
                            const std::array<Vec3, kNumLegs>& raised_world_xy,
                            const double target_world_z) {
    std::array<bool, kNumLegs> raised_mask{};
    for (const int leg : kRaisedLegs) {
        raised_mask[static_cast<std::size_t>(leg)] = true;
    }
    assignFeetWorldXY(feet, est, raised_world_xy, raised_mask, target_world_z);
}

bool jointsFinite(const JointTargets& joints) {
    for (const auto& leg_state : joints.leg_states) {
        for (const auto& joint : leg_state.joint_state) {
            if (!std::isfinite(joint.pos_rad.value) || !std::isfinite(joint.vel_radps.value)) {
                return false;
            }
        }
    }
    return true;
}

std::array<Vec3, kNumLegs> captureFootWorld(LegFK& fk,
                                            const RobotState& est,
                                            const HexapodGeometry& geometry) {
    std::array<Vec3, kNumLegs> out{};
    const BodyPose pose = makeBodyPose(est);
    for (int leg = 0; leg < kNumLegs; ++leg) {
        out[static_cast<std::size_t>(leg)] = fk.footInWorldFrame(
            est.leg_states[static_cast<std::size_t>(leg)],
            pose,
            geometry.legGeometry[static_cast<std::size_t>(leg)]).pos_body_m.raw();
    }
    return out;
}

JointTargets solveTargets(LegIK& ik, const LegTargets& feet, const RobotState& est) {
    SafetyState safety{};
    safety.inhibit_motion = false;
    safety.torque_cut = false;
    safety.leg_enabled.fill(true);
    return ik.solve(est, feet, safety);
}

bool holdTargets(PhysicsSimBridge& bridge,
                 const JointTargets& targets,
                 const int steps,
                 RobotState* last_state) {
    RobotState state{};
    for (int i = 0; i < steps; ++i) {
        if (!bridge.write(targets)) {
            return false;
        }
        if (!bridge.read(state)) {
            return false;
        }
    }
    if (last_state != nullptr) {
        *last_state = state;
    }
    return true;
}

struct GroupClearance {
    double commanded_cart_world_z_sum{0.0};
    double post_ik_fk_world_z_sum{0.0};
    double measured_fk_world_z_sum{0.0};
    double measured_clearance_sum{0.0};
    double contact_sum{0.0};
    double reach_hit_sum{0.0};
    std::uint64_t samples{0};
};

void accumulateGroup(GroupClearance* group,
                     const bool in_group,
                     const double commanded_cart_world_z,
                     const double post_ik_fk_world_z,
                     const double measured_fk_world_z,
                     const bool contact,
                     const bool reach_hit,
                     const double foot_radius_m) {
    if (!in_group) {
        return;
    }
    group->commanded_cart_world_z_sum += commanded_cart_world_z;
    group->post_ik_fk_world_z_sum += post_ik_fk_world_z;
    group->measured_fk_world_z_sum += measured_fk_world_z;
    group->measured_clearance_sum += measured_fk_world_z - foot_radius_m;
    group->contact_sum += contact ? 1.0 : 0.0;
    group->reach_hit_sum += reach_hit ? 1.0 : 0.0;
    ++group->samples;
}

} // namespace

int main(int argc, char** argv) {
#if !defined(__linux__)
    std::cout << "skip test_physics_sim_tripod_stroke_probe (Linux-only)\n";
    return 0;
#else
    bool emit_metrics_json = false;
    const char* sim_exe = nullptr;
    physics_sim_test_argv::parse(argc, argv, emit_metrics_json, sim_exe);
    if (sim_exe == nullptr || sim_exe[0] == '\0') {
        std::cout << "skip test_physics_sim_tripod_stroke_probe "
                     "(pass sim path or HEXAPOD_PHYSICS_SIM_EXE)\n";
        return 0;
    }

    const auto harness = physics_sim_test_utils::loadHarnessSettings();
    const int replay_period_us = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_PERIOD_US", 5000);
    const double dt_s = static_cast<double>(replay_period_us) * 1.0e-6;
    const double vx_mps = positiveDoubleEnvOrDefault("HEXAPOD_TRIPOD_STROKE_VX_MPS", 0.12);
    const double torque_scale = positiveDoubleEnvOrDefault("HEXAPOD_TRIPOD_STROKE_TORQUE_SCALE", 1.0);
    const std::string torque_scale_text = std::to_string(torque_scale);
    if (std::abs(torque_scale - 1.0) > 1.0e-12) {
        ::setenv("HEXAPOD_SERVO_TORQUE_SCALE", torque_scale_text.c_str(), 1);
    }
    const int warmup_stand = positiveEnvOrDefault("HEXAPOD_TRIPOD_STROKE_STAND_FRAMES", 200);
    const int warmup_tripod = positiveEnvOrDefault("HEXAPOD_TRIPOD_STROKE_RAISE_FRAMES", 300);
    const int stroke_frames = positiveEnvOrDefault("HEXAPOD_TRIPOD_STROKE_FRAMES", 160);
    PhysicsSimSolverSettings solver{};
    solver.mode = envEnabled("HEXAPOD_EXACT_REPLAY_LEGACY")
        ? physics_sim::PhysicsSolverMode::LegacyPgs
        : physics_sim::PhysicsSolverMode::PinocchioProximal;
    solver.iterations = positiveEnvOrDefault("HEXAPOD_EXACT_REPLAY_SOLVER_ITERATIONS", 500);
    solver.proximal_mu = static_cast<float>(
        positiveDoubleEnvOrDefault("HEXAPOD_EXACT_REPLAY_PROXIMAL_MU", 1.0e-6));
    solver.contact_regularization = static_cast<float>(
        positiveDoubleEnvOrDefault("HEXAPOD_EXACT_REPLAY_CONTACT_REGULARIZATION", 1.0e-10));
    solver.absolute_tolerance = static_cast<float>(
        positiveDoubleEnvOrDefault("HEXAPOD_EXACT_REPLAY_ABSOLUTE_TOLERANCE", 1.0e-8));
    solver.relative_tolerance = static_cast<float>(
        positiveDoubleEnvOrDefault("HEXAPOD_EXACT_REPLAY_RELATIVE_TOLERANCE", 1.0e-6));

    const int port = 24500 + (static_cast<int>(::getpid()) % 4000);
    const pid_t child = ::fork();
    if (child < 0) {
        std::perror("fork");
        return 2;
    }
    if (child == 0) {
        physics_sim_test_utils::quietChildProcessStdIo();
        const std::string port_text = std::to_string(port);
        ::execl(sim_exe, sim_exe, "--serve", "--serve-port", port_text.c_str(), nullptr);
        std::perror("execl");
        _exit(127);
    }
    const auto stop_child = [&]() {
        ::kill(child, SIGTERM);
        ::waitpid(child, nullptr, 0);
    };
    std::this_thread::sleep_for(std::chrono::milliseconds{250});

    PhysicsSimBridge bridge("127.0.0.1", port, replay_period_us, solver, nullptr);
    if (!bridge.init()) {
        std::cerr << "FAIL: tripod stroke probe bridge did not initialize\n";
        stop_child();
        return 1;
    }

    LegIK ik(defaultHexapodGeometry());
    const HexapodGeometry geometry = defaultHexapodGeometry();
    LegFK fk{};
    const double raised_world_z = kFootRadiusM + kRaisedClearanceM;
    LegTargets feet = standFootTargets();
    RobotState est{};
    const JointTargets stand_joints = solveTargets(ik, feet, est);
    if (!holdTargets(bridge, stand_joints, warmup_stand, &est)) {
        std::cerr << "FAIL: stand warmup write/read failed\n";
        stop_child();
        return 1;
    }

    const std::array<Vec3, kNumLegs> captured_world = captureFootWorld(fk, est, geometry);
    double raised_start_z = 0.0;
    for (const int leg : kRaisedLegs) {
        raised_start_z += captured_world[static_cast<std::size_t>(leg)].z;
    }
    raised_start_z /= static_cast<double>(kRaisedLegs.size());
    JointTargets raised_joints{};
    std::array<bool, kNumLegs> raise_reach{};
    for (int i = 0; i < warmup_tripod; ++i) {
        const double alpha = static_cast<double>(i + 1) / static_cast<double>(warmup_tripod);
        const double raised_z = raised_start_z + alpha * (raised_world_z - raised_start_z);
        assignRaisedFeetWorldZ(&feet, est, captured_world, raised_z);
        assignFeetWorldXY(&feet, est, captured_world, kSupportLegs, kFootRadiusM);
        raised_joints = solveTargets(ik, feet, est);
        raise_reach = ik.lastReachClampHit();
        if (!jointsFinite(raised_joints)) {
            std::cerr << "FAIL: non-finite raise IK at frame " << i << '\n';
            stop_child();
            return 1;
        }
        if (!bridge.write(raised_joints) || !bridge.read(est)) {
            std::cerr << "FAIL: tripod raise warmup write/read failed at frame " << i << '\n';
            stop_child();
            return 1;
        }
    }
    const std::array<Vec3, kNumLegs> raised_world = captured_world;
    const BodyPose raise_pose = makeBodyPose(est);
    const Mat3 raise_R = raise_pose.rotationBodyToWorld();
    GroupClearance raise_warmup_raised{};
    GroupClearance raise_warmup_support{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        const Vec3 cart_body = feet.feet[idx].pos_body_m.raw();
        const Vec3 cart_world = raise_pose.position.raw() + (raise_R * cart_body);
        const Vec3 post_ik_world = fk.footInWorldFrame(
            raised_joints.leg_states[idx], raise_pose, geometry.legGeometry[idx])
            .pos_body_m.raw();
        const Vec3 measured_world = fk.footInWorldFrame(
            est.leg_states[idx], raise_pose, geometry.legGeometry[idx]).pos_body_m.raw();
        accumulateGroup(
            kSupportLegs[idx] ? &raise_warmup_support : &raise_warmup_raised,
            true,
            cart_world.z,
            post_ik_world.z,
            measured_world.z,
            est.foot_contacts[idx],
            raise_reach[idx],
            kFootRadiusM);
    }

    BodyVelocityCommand stroke_cmd{};
    // A canonical +X body command makes a planted foot stroke in canonical
    // -X / sim +Z while the chassis advances along sim -Z.
    stroke_cmd.linear_mps.x = vx_mps;
    std::optional<Vec3> start_position{};
    std::optional<Vec3> last_position{};
    std::optional<Vec3> previous_body_position{};
    std::array<Vec3, kNumLegs> previous_foot_world{};
    std::array<Vec3, kNumLegs> previous_target_body{};
    std::array<bool, kNumLegs> have_previous_contact{};
    double n_raw_contact_sum = 0.0;
    double n_support_contact_sum = 0.0;
    std::uint64_t census_frames = 0;
    std::uint64_t clean_tripod_frames = 0;
    double commanded_world_sum = 0.0;
    double uncommanded_slip_sum = 0.0;
    double contact_world_sum = 0.0;
    std::uint64_t slip_samples = 0;
    double abs_pitch_sum = 0.0;
    double cartesian_opposition_sum = 0.0;
    std::uint64_t cartesian_samples = 0;
    GroupClearance stroke_raised{};
    GroupClearance stroke_support{};
    double peak_normal_impulse_sum = 0.0;
    double peak_friction_impulse_sum = 0.0;
    double sum_friction_impulse_world_x_sum = 0.0;
    double sum_friction_impulse_world_z_sum = 0.0;
    double sum_abs_friction_impulse_world_x_sum = 0.0;
    double sum_abs_friction_impulse_world_z_sum = 0.0;
    double sum_friction_impulse_world_y_sum = 0.0;
    double contact_delta_vx_sum = 0.0;
    double contact_delta_vz_sum = 0.0;
    std::array<double, kNumLegs> leg_friction_world_x_sum{};
    std::array<double, kNumLegs> leg_friction_world_z_sum{};
    std::array<double, kNumLegs> leg_pinocchio_drift_tx_sum{};
    std::array<double, kNumLegs> leg_world_slip_tx_sum{};
    std::array<double, kNumLegs> leg_world_slip_ty_sum{};
    std::array<double, kNumLegs> leg_contact_count_sum{};
    std::array<double, kNumLegs> leg_tibia_vx_sum{};
    std::array<double, kNumLegs> leg_spin_vx_sum{};
    std::array<double, kNumLegs> leg_t0_x_sum{};
    std::array<double, kNumLegs> leg_foot_vx_sum{};
    std::array<double, kNumLegs> leg_foot_x_sum{};
    std::array<double, kNumLegs> leg_foot_pos_vx_sum{};
    std::array<double, kNumLegs> leg_foot_vz_sum{};
    std::array<double, kNumLegs> leg_foot_z_sum{};
    std::array<double, kNumLegs> leg_foot_pos_vz_sum{};
    std::array<double, kNumLegs> leg_cmd_foot_vx_sum{};
    std::array<double, kNumLegs> leg_fk_foot_vx_sum{};
    std::array<std::uint64_t, kNumLegs> leg_cmd_fk_samples{};
    std::array<double, kNumLegs> leg_fk_minus_mapped_x_sum{};
    std::array<double, kNumLegs> leg_fk_minus_mapped_y_sum{};
    std::array<std::uint64_t, kNumLegs> leg_fk_sphere_samples{};
    std::array<double, kNumLegs> stroke_start_fk_x{};
    std::array<double, kNumLegs> stroke_start_mapped_x{};
    std::array<double, kNumLegs> stroke_start_align_x{};
    std::array<double, kNumLegs> stroke_end_fk_x{};
    std::array<double, kNumLegs> stroke_end_mapped_x{};
    std::array<double, kNumLegs> stroke_end_align_x{};
    std::array<double, kNumLegs> rest_fk_minus_bridge_x{};
    std::array<double, kNumLegs> rest_fk_minus_bridge_y{};
    std::array<double, kNumLegs> rest_fk_minus_align_x{};
    std::array<double, kNumLegs> rest_fk_minus_align_y{};
    std::array<double, kNumLegs> rest_body_minus_bridge_x{};
    std::array<double, kNumLegs> rest_body_minus_bridge_y{};
    std::array<double, kNumLegs> rest_body_minus_align_x{};
    std::array<double, kNumLegs> rest_body_minus_align_y{};
    std::array<double, kNumLegs> rest_body_x{};
    std::array<double, kNumLegs> rest_body_y{};
    std::array<double, kNumLegs> rest_sphere_align_x{};
    std::array<double, kNumLegs> rest_sphere_align_y{};
    std::array<double, kNumLegs> rest_sphere_bridge_x{};
    std::array<double, kNumLegs> rest_sphere_bridge_y{};
    std::array<double, kNumLegs> plant_minus_align_x{};
    std::array<double, kNumLegs> plant_minus_align_y{};
    std::array<double, kNumLegs> plant_minus_bridge_x{};
    std::array<double, kNumLegs> plant_minus_bridge_y{};
    std::array<bool, kNumLegs> have_stroke_start_xyz{};
    std::array<double, 3> max_joint_tracking_error_rad{};
    double friction_to_normal_ratio_sum = 0.0;
    double servo_torque_utilization_sum = 0.0;
    double cone_residual_sum = 0.0;
    double max_contact_penetration_sum = 0.0;
    double max_servo_tracking_error_rad = 0.0;
    std::uint64_t telemetry_frames = 0;
    std::uint64_t friction_ratio_samples = 0;
    double expected_com_delta_v_sum = 0.0;
    double body_vx_sum = 0.0;
    double support_foot_world_x_sum = 0.0;
    std::uint64_t support_foot_x_samples = 0;
    double last_body_vx = 0.0;

    for (int i = 0; i < stroke_frames; ++i) {
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!kSupportLegs[static_cast<std::size_t>(leg)]) {
                continue;
            }
            const Vec3 p = feet.feet[leg].pos_body_m.raw();
            const Vec3 v = TwistField::stanceFootVelocity(stroke_cmd, p);
            feet.feet[leg].pos_body_m = p + (v * dt_s);
            feet.feet[leg].vel_body_mps = v;
        }
        assignRaisedFeetWorldZ(&feet, est, raised_world, raised_world_z);
        const JointTargets joints = solveTargets(ik, feet, est);
        const auto reach_hit = ik.lastReachClampHit();
        for (const auto& leg_state : joints.leg_states) {
            for (const auto& joint : leg_state.joint_state) {
                if (!std::isfinite(joint.pos_rad.value)) {
                    std::cerr << "FAIL: non-finite stroke IK at frame " << i << '\n';
                    stop_child();
                    return 1;
                }
            }
        }
        if (!bridge.write(joints)) {
            std::cerr << "FAIL: stroke write failed at frame " << i << '\n';
            stop_child();
            return 1;
        }
        if (!bridge.read(est)) {
            std::cerr << "FAIL: stroke read failed at frame " << i << '\n';
            stop_child();
            return 1;
        }
        for (int leg = 0; leg < kNumLegs; ++leg) {
            if (!kSupportLegs[static_cast<std::size_t>(leg)]) {
                continue;
            }
            const std::size_t idx = static_cast<std::size_t>(leg);
            for (int joint = 0; joint < kJointsPerLeg; ++joint) {
                const double error = std::abs(std::remainder(
                    joints.leg_states[idx].joint_state[static_cast<std::size_t>(joint)].pos_rad.value
                        - est.leg_states[idx].joint_state[static_cast<std::size_t>(joint)].pos_rad.value,
                    2.0 * kPi));
                max_servo_tracking_error_rad = std::max(max_servo_tracking_error_rad, error);
                max_joint_tracking_error_rad[static_cast<std::size_t>(joint)] =
                    std::max(max_joint_tracking_error_rad[static_cast<std::size_t>(joint)], error);
            }
        }
        std::optional<double> frame_peak_friction_impulse;
        std::optional<PhysicsSimSolverTelemetry> frame_telemetry;
        if (const auto telemetry = bridge.latestSolverTelemetry(); telemetry.has_value()) {
            frame_telemetry = telemetry;
            ++telemetry_frames;
            peak_normal_impulse_sum += telemetry->peak_normal_impulse;
            peak_friction_impulse_sum += telemetry->peak_friction_impulse;
            sum_friction_impulse_world_x_sum += telemetry->sum_friction_impulse_world_x;
            sum_friction_impulse_world_z_sum += telemetry->sum_friction_impulse_world_z;
            sum_abs_friction_impulse_world_x_sum += telemetry->sum_abs_friction_impulse_world_x;
            sum_abs_friction_impulse_world_z_sum += telemetry->sum_abs_friction_impulse_world_z;
            sum_friction_impulse_world_y_sum += telemetry->sum_friction_impulse_world_y;
            contact_delta_vx_sum += telemetry->contact_delta_vx;
            contact_delta_vz_sum += telemetry->contact_delta_vz;
            for (int leg = 0; leg < kNumLegs; ++leg) {
                const std::size_t idx = static_cast<std::size_t>(leg);
                leg_friction_world_x_sum[idx] += telemetry->leg_friction_impulse_world_x[idx];
                leg_friction_world_z_sum[idx] += telemetry->leg_friction_impulse_world_z[idx];
                leg_pinocchio_drift_tx_sum[idx] += telemetry->leg_pinocchio_drift_tx[idx];
                leg_world_slip_tx_sum[idx] += telemetry->leg_world_slip_tx[idx];
                leg_world_slip_ty_sum[idx] += telemetry->leg_world_slip_ty[idx];
                leg_contact_count_sum[idx] += telemetry->leg_contact_count[idx];
                leg_tibia_vx_sum[idx] += telemetry->leg_tibia_vx[idx];
                leg_spin_vx_sum[idx] += telemetry->leg_spin_vx[idx];
                leg_t0_x_sum[idx] += telemetry->leg_t0_x[idx];
                leg_foot_vx_sum[idx] += telemetry->leg_foot_vx[idx];
                leg_foot_x_sum[idx] += telemetry->leg_foot_x[idx];
                leg_foot_pos_vx_sum[idx] += telemetry->leg_foot_pos_vx[idx];
                leg_foot_vz_sum[idx] += telemetry->leg_foot_vz[idx];
                leg_foot_z_sum[idx] += telemetry->leg_foot_z[idx];
                leg_foot_pos_vz_sum[idx] += telemetry->leg_foot_pos_vz[idx];
            }
            servo_torque_utilization_sum += telemetry->peak_servo_torque_utilization;
            cone_residual_sum += telemetry->cone_residual;
            max_contact_penetration_sum += telemetry->max_contact_penetration;
            frame_peak_friction_impulse = telemetry->peak_friction_impulse;
            if (telemetry->peak_normal_impulse > 1.0e-12f) {
                friction_to_normal_ratio_sum +=
                    static_cast<double>(telemetry->peak_friction_impulse)
                    / static_cast<double>(telemetry->peak_normal_impulse);
                ++friction_ratio_samples;
            }
        }
        const RobotState& state = est;
        const Vec3 position{
            state.body_twist_state.body_trans_m.x,
            state.body_twist_state.body_trans_m.y,
            state.body_twist_state.body_trans_m.z};
        if (!start_position.has_value()) {
            start_position = position;
        }
        last_position = position;
        const BodyPose body_pose = makeBodyPose(state);
        const Mat3 R = body_pose.rotationBodyToWorld();
        const Vec3 body_step = previous_body_position.has_value()
            ? Vec3{position.x - previous_body_position->x,
                   position.y - previous_body_position->y,
                   0.0}
            : Vec3{};
        std::size_t n_raw = 0;
        std::size_t n_support_contact = 0;
        std::array<bool, kNumLegs> planned{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            planned[idx] = kSupportLegs[idx];
            if (state.foot_contacts[idx]) {
                ++n_raw;
                if (kSupportLegs[idx]) {
                    ++n_support_contact;
                }
            }
        }
        ++census_frames;
        n_raw_contact_sum += static_cast<double>(n_raw);
        n_support_contact_sum += static_cast<double>(n_support_contact);
        if (frame_peak_friction_impulse.has_value()) {
            expected_com_delta_v_sum +=
                static_cast<double>(n_support_contact) * *frame_peak_friction_impulse / kHexapodMassKg;
        }
        last_body_vx = state.body_twist_state.body_trans_mps.x;
        body_vx_sum += last_body_vx;
        if (isCleanTripodFrame(planned, state.foot_contacts, 0)) {
            ++clean_tripod_frames;
        }
        abs_pitch_sum += std::abs(state.body_twist_state.twist_pos_rad.y);
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            const Vec3 cart_body = feet.feet[idx].pos_body_m.raw();
            const Vec3 cart_world = body_pose.position.raw() + (R * cart_body);
            const Vec3 target_body = fk.footInBodyFrame(
                joints.leg_states[idx], geometry.legGeometry[idx]).pos_body_m.raw();
            const Vec3 post_ik_world = fk.footInWorldFrame(
                joints.leg_states[idx], body_pose, geometry.legGeometry[idx])
                .pos_body_m.raw();
            const Vec3 measured_world = fk.footInWorldFrame(
                state.leg_states[idx], body_pose, geometry.legGeometry[idx]).pos_body_m.raw();
            const Vec3 measured_body = fk.footInBodyFrame(
                state.leg_states[idx], geometry.legGeometry[idx]).pos_body_m.raw();
            if (kSupportLegs[idx] && frame_telemetry.has_value()) {
                const double sim_x = static_cast<double>(frame_telemetry->leg_foot_x[idx]);
                const double sim_z = static_cast<double>(frame_telemetry->leg_foot_z[idx]);
                const double mapped_x = -sim_z;
                const double mapped_y = sim_x;
                const double align_x = sim_x;
                const double align_y = sim_z;
                const Vec3 rel_sim = chassisRelativeSphereSim(body_pose, sim_x, sim_z);
                const Vec3 sphere_bridge{-rel_sim.z, rel_sim.x, rel_sim.y};
                const Vec3 sphere_align{rel_sim.x, rel_sim.z, rel_sim.y};
                leg_fk_minus_mapped_x_sum[idx] += measured_world.x - mapped_x;
                leg_fk_minus_mapped_y_sum[idx] += measured_world.y - mapped_y;
                ++leg_fk_sphere_samples[idx];
                if (!have_stroke_start_xyz[idx]) {
                    stroke_start_fk_x[idx] = measured_world.x;
                    stroke_start_mapped_x[idx] = mapped_x;
                    stroke_start_align_x[idx] = align_x;
                    rest_fk_minus_bridge_x[idx] = measured_world.x - mapped_x;
                    rest_fk_minus_bridge_y[idx] = measured_world.y - mapped_y;
                    rest_fk_minus_align_x[idx] = measured_world.x - align_x;
                    rest_fk_minus_align_y[idx] = measured_world.y - align_y;
                    rest_body_minus_bridge_x[idx] = measured_body.x - sphere_bridge.x;
                    rest_body_minus_bridge_y[idx] = measured_body.y - sphere_bridge.y;
                    rest_body_minus_align_x[idx] = measured_body.x - sphere_align.x;
                    rest_body_minus_align_y[idx] = measured_body.y - sphere_align.y;
                    rest_body_x[idx] = measured_body.x;
                    rest_body_y[idx] = measured_body.y;
                    rest_sphere_align_x[idx] = sphere_align.x;
                    rest_sphere_align_y[idx] = sphere_align.y;
                    rest_sphere_bridge_x[idx] = sphere_bridge.x;
                    rest_sphere_bridge_y[idx] = sphere_bridge.y;
                    plant_minus_bridge_x[idx] = captured_world[idx].x - mapped_x;
                    plant_minus_bridge_y[idx] = captured_world[idx].y - mapped_y;
                    plant_minus_align_x[idx] = captured_world[idx].x - align_x;
                    plant_minus_align_y[idx] = captured_world[idx].y - align_y;
                    have_stroke_start_xyz[idx] = true;
                }
                stroke_end_fk_x[idx] = measured_world.x;
                stroke_end_mapped_x[idx] = mapped_x;
                stroke_end_align_x[idx] = align_x;
            }
            accumulateGroup(
                kSupportLegs[idx] ? &stroke_support : &stroke_raised,
                true,
                cart_world.z,
                post_ik_world.z,
                measured_world.z,
                state.foot_contacts[idx],
                reach_hit[idx],
                kFootRadiusM);
            if (kSupportLegs[idx] && previous_body_position.has_value()) {
                const Vec3 cartesian_step = target_body - previous_target_body[idx];
                cartesian_opposition_sum -= cartesian_step.x;
                ++cartesian_samples;
            }
            if (state.foot_contacts[idx] && have_previous_contact[idx]
                && previous_body_position.has_value()) {
                const Vec3 measured_delta{
                    measured_world.x - previous_foot_world[idx].x,
                    measured_world.y - previous_foot_world[idx].y,
                    0.0};
                const Vec3 target_step = target_body - previous_target_body[idx];
                const Vec3 commanded_delta = body_step + (R * target_step);
                commanded_world_sum += std::hypot(commanded_delta.x, commanded_delta.y);
                uncommanded_slip_sum += std::hypot(
                    measured_delta.x - commanded_delta.x,
                    measured_delta.y - commanded_delta.y);
                contact_world_sum += std::hypot(measured_delta.x, measured_delta.y);
                ++slip_samples;
                if (kSupportLegs[idx]) {
                    support_foot_world_x_sum += measured_delta.x;
                    ++support_foot_x_samples;
                    leg_cmd_foot_vx_sum[idx] += commanded_delta.x;
                    leg_fk_foot_vx_sum[idx] += measured_delta.x;
                    ++leg_cmd_fk_samples[idx];
                }
            }
            have_previous_contact[idx] = state.foot_contacts[idx];
            previous_foot_world[idx] = measured_world;
            previous_target_body[idx] = target_body;
        }
        previous_body_position = position;
    }

    stop_child();

    const double commanded_translation = vx_mps * static_cast<double>(stroke_frames) * dt_s;
    const double command_progress = (start_position.has_value() && last_position.has_value())
        ? (last_position->x - start_position->x)
        : 0.0;
    const auto meanValue = [](const double sum, const std::uint64_t samples) {
        return samples == 0 ? 0.0 : sum / static_cast<double>(samples);
    };
    const auto meanRate = [dt_s](const double sum, const std::uint64_t samples) {
        return samples == 0 ? 0.0 : sum / static_cast<double>(samples) / dt_s;
    };
    const auto emitGroup = [&](const char* prefix, const GroupClearance& group) {
        std::cout << ",\"" << prefix << "_commanded_cart_world_z_m\":"
                  << meanValue(group.commanded_cart_world_z_sum, group.samples)
                  << ",\"" << prefix << "_post_ik_fk_world_z_m\":"
                  << meanValue(group.post_ik_fk_world_z_sum, group.samples)
                  << ",\"" << prefix << "_measured_fk_world_z_m\":"
                  << meanValue(group.measured_fk_world_z_sum, group.samples)
                  << ",\"" << prefix << "_measured_clearance_m\":"
                  << meanValue(group.measured_clearance_sum, group.samples)
                  << ",\"" << prefix << "_contact_fraction\":"
                  << meanValue(group.contact_sum, group.samples)
                  << ",\"" << prefix << "_reach_hit_fraction\":"
                  << meanValue(group.reach_hit_sum, group.samples);
    };

    const double raised_clearance = meanValue(stroke_raised.measured_clearance_sum, stroke_raised.samples);
    const double raised_contact = meanValue(stroke_raised.contact_sum, stroke_raised.samples);
    const double raised_reach = meanValue(stroke_raised.reach_hit_sum, stroke_raised.samples);
    const double raised_fk_z = meanValue(stroke_raised.measured_fk_world_z_sum, stroke_raised.samples);
    const double support_fk_z = meanValue(stroke_support.measured_fk_world_z_sum, stroke_support.samples);
    const char* named_unload_cause = "true_plant";
    if (raised_reach > 0.5 && raised_clearance < 0.005) {
        named_unload_cause = "reach_ik";
    } else if (raised_clearance > 0.010 && raised_contact > 0.5) {
        named_unload_cause = "shaft_contact_bit";
    } else if (raised_clearance > 0.010 && raised_contact <= 0.5) {
        named_unload_cause = "unloaded";
    } else if (std::abs(raised_fk_z - support_fk_z) < 0.010 && raised_reach <= 0.5) {
        named_unload_cause = "true_plant";
    }

    const double mean_n_raw_contact = meanValue(n_raw_contact_sum, census_frames);
    const double mean_n_support_contact = meanValue(n_support_contact_sum, census_frames);
    const double commanded_world_speed = meanRate(commanded_world_sum, slip_samples);
    const double uncommanded_slip_speed = meanRate(uncommanded_slip_sum, slip_samples);
    const double contact_world_speed = meanRate(contact_world_sum, slip_samples);
    const double mean_peak_normal_impulse_ns = meanValue(peak_normal_impulse_sum, telemetry_frames);
    const double mean_peak_friction_impulse_ns = meanValue(peak_friction_impulse_sum, telemetry_frames);
    const double mean_sum_friction_impulse_world_x =
        meanValue(sum_friction_impulse_world_x_sum, telemetry_frames);
    const double mean_sum_friction_impulse_world_z =
        meanValue(sum_friction_impulse_world_z_sum, telemetry_frames);
    const double mean_sum_abs_friction_impulse_world_x =
        meanValue(sum_abs_friction_impulse_world_x_sum, telemetry_frames);
    const double mean_sum_abs_friction_impulse_world_z =
        meanValue(sum_abs_friction_impulse_world_z_sum, telemetry_frames);
    const double mean_sum_friction_impulse_world_y =
        meanValue(sum_friction_impulse_world_y_sum, telemetry_frames);
    const double mean_contact_delta_vx = meanValue(contact_delta_vx_sum, telemetry_frames);
    const double mean_contact_delta_vz = meanValue(contact_delta_vz_sum, telemetry_frames);
    const double mean_abs_horiz_friction_impulse =
        mean_sum_abs_friction_impulse_world_x + mean_sum_abs_friction_impulse_world_z;
    const double expected_dvx_from_world_friction =
        mean_sum_friction_impulse_world_x / kHexapodMassKg;
    const double friction_axis_x_fraction =
        mean_abs_horiz_friction_impulse > 1.0e-9
            ? mean_sum_abs_friction_impulse_world_x / mean_abs_horiz_friction_impulse
            : 0.0;
    const double friction_horizontal_capture =
        mean_peak_friction_impulse_ns > 1.0e-9
            ? mean_abs_horiz_friction_impulse / mean_peak_friction_impulse_ns
            : 0.0;
    const double friction_x_cancellation =
        mean_sum_abs_friction_impulse_world_x > 1.0e-9
            ? std::abs(mean_sum_friction_impulse_world_x) / mean_sum_abs_friction_impulse_world_x
            : 0.0;
    const double jacobian_com_ratio =
        std::abs(expected_dvx_from_world_friction) > 1.0e-5
            ? mean_contact_delta_vx / expected_dvx_from_world_friction
            : 0.0;
    const double mean_friction_to_normal_impulse_ratio =
        meanValue(friction_to_normal_ratio_sum, friction_ratio_samples);
    const double mean_servo_torque_utilization =
        meanValue(servo_torque_utilization_sum, telemetry_frames);
    const double mean_cone_residual = meanValue(cone_residual_sum, telemetry_frames);
    const double mean_max_contact_penetration = meanValue(max_contact_penetration_sum, telemetry_frames);
    const double progress_frac =
        commanded_translation > 1.0e-9 ? command_progress / commanded_translation : 0.0;
    const double support_contact_fraction =
        meanValue(stroke_support.contact_sum, stroke_support.samples);
    const double slip_ratio =
        commanded_world_speed > 1.0e-6 ? uncommanded_slip_speed / commanded_world_speed : 0.0;
    const double mean_body_vx_mps = meanValue(body_vx_sum, census_frames);
    const double mean_support_foot_world_vx_mps = meanRate(support_foot_world_x_sum, support_foot_x_samples);
    const double friction_com_coupling_ratio =
        expected_com_delta_v_sum > 1.0e-9 ? last_body_vx / expected_com_delta_v_sum : 0.0;
    const char* named_coupling = "unknown";
    if (expected_com_delta_v_sum > 0.05) {
        named_coupling = std::abs(friction_com_coupling_ratio) < 0.10 ? "decoupled" : "coupled";
    }
    const char* named_mapping = "unknown";
    if (mean_peak_friction_impulse_ns > 1.0e-4) {
        if (friction_horizontal_capture < 0.20) {
            named_mapping = "tangent_not_world";
        } else if (friction_axis_x_fraction < 0.20) {
            named_mapping = "tangent_not_stroke";
        } else if (friction_x_cancellation < 0.20) {
            named_mapping = "opposing_tangents";
        } else if (std::abs(jacobian_com_ratio) < 0.10) {
            named_mapping = "jacobian_decoupled";
        } else if (
            std::abs(mean_contact_delta_vx) > 1.0e-5
            && std::abs(mean_body_vx_mps) < 0.10 * std::abs(mean_contact_delta_vx) / dt_s) {
            named_mapping = "write_or_servo_absorb";
        } else {
            named_mapping = "applied";
        }
    }

    std::array<double, kNumLegs> mean_leg_friction_world_x{};
    std::array<double, kNumLegs> mean_leg_friction_world_z{};
    std::array<double, kNumLegs> mean_leg_pinocchio_drift_tx{};
    std::array<double, kNumLegs> mean_leg_world_slip_tx{};
    std::array<double, kNumLegs> mean_leg_world_slip_ty{};
    std::array<double, kNumLegs> mean_leg_contact_count{};
    std::array<double, kNumLegs> mean_leg_tibia_vx{};
    std::array<double, kNumLegs> mean_leg_spin_vx{};
    std::array<double, kNumLegs> mean_leg_t0_x{};
    std::array<double, kNumLegs> mean_leg_foot_vx{};
    std::array<double, kNumLegs> mean_leg_foot_x{};
    std::array<double, kNumLegs> mean_leg_foot_pos_vx{};
    std::array<double, kNumLegs> mean_leg_foot_vz{};
    std::array<double, kNumLegs> mean_leg_foot_z{};
    std::array<double, kNumLegs> mean_leg_foot_pos_vz{};
    std::array<double, kNumLegs> mean_leg_cmd_foot_vx{};
    std::array<double, kNumLegs> mean_leg_fk_foot_vx{};
    std::array<double, kNumLegs> mean_leg_expected_sim_vz{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        mean_leg_friction_world_x[idx] =
            meanValue(leg_friction_world_x_sum[idx], telemetry_frames);
        mean_leg_friction_world_z[idx] =
            meanValue(leg_friction_world_z_sum[idx], telemetry_frames);
        mean_leg_pinocchio_drift_tx[idx] =
            meanValue(leg_pinocchio_drift_tx_sum[idx], telemetry_frames);
        mean_leg_world_slip_tx[idx] = meanValue(leg_world_slip_tx_sum[idx], telemetry_frames);
        mean_leg_world_slip_ty[idx] = meanValue(leg_world_slip_ty_sum[idx], telemetry_frames);
        mean_leg_contact_count[idx] = meanValue(leg_contact_count_sum[idx], telemetry_frames);
        mean_leg_tibia_vx[idx] = meanValue(leg_tibia_vx_sum[idx], telemetry_frames);
        mean_leg_spin_vx[idx] = meanValue(leg_spin_vx_sum[idx], telemetry_frames);
        mean_leg_t0_x[idx] = meanValue(leg_t0_x_sum[idx], telemetry_frames);
        mean_leg_foot_vx[idx] = meanValue(leg_foot_vx_sum[idx], telemetry_frames);
        mean_leg_foot_x[idx] = meanValue(leg_foot_x_sum[idx], telemetry_frames);
        mean_leg_foot_pos_vx[idx] = meanValue(leg_foot_pos_vx_sum[idx], telemetry_frames);
        mean_leg_foot_vz[idx] = meanValue(leg_foot_vz_sum[idx], telemetry_frames);
        mean_leg_foot_z[idx] = meanValue(leg_foot_z_sum[idx], telemetry_frames);
        mean_leg_foot_pos_vz[idx] = meanValue(leg_foot_pos_vz_sum[idx], telemetry_frames);
        mean_leg_cmd_foot_vx[idx] = meanRate(leg_cmd_foot_vx_sum[idx], leg_cmd_fk_samples[idx]);
        mean_leg_fk_foot_vx[idx] = meanRate(leg_fk_foot_vx_sum[idx], leg_cmd_fk_samples[idx]);
        // Server world X = -sim Z. A forward skate in server -X is sim +Z.
        mean_leg_expected_sim_vz[idx] = -mean_leg_fk_foot_vx[idx];
    }
    std::array<double, kNumLegs> mean_leg_fk_minus_mapped_x{};
    std::array<double, kNumLegs> mean_leg_fk_minus_mapped_y{};
    std::array<double, kNumLegs> stroke_fk_dx{};
    std::array<double, kNumLegs> stroke_mapped_dx{};
    std::array<double, kNumLegs> stroke_align_dx{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        mean_leg_fk_minus_mapped_x[idx] =
            meanValue(leg_fk_minus_mapped_x_sum[idx], leg_fk_sphere_samples[idx]);
        mean_leg_fk_minus_mapped_y[idx] =
            meanValue(leg_fk_minus_mapped_y_sum[idx], leg_fk_sphere_samples[idx]);
        if (have_stroke_start_xyz[idx]) {
            stroke_fk_dx[idx] = stroke_end_fk_x[idx] - stroke_start_fk_x[idx];
            stroke_mapped_dx[idx] = stroke_end_mapped_x[idx] - stroke_start_mapped_x[idx];
            stroke_align_dx[idx] = stroke_end_align_x[idx] - stroke_start_align_x[idx];
        }
    }
    int planted_support = 0;
    int drift_slip_disagree = 0;
    int drift_slip_agree = 0;
    int friction_pos = 0;
    int friction_neg = 0;
    double planted_contact_count_sum = 0.0;
    constexpr double kSignEps = 1.0e-4;
    auto signOf = [](double value) {
        if (value > kSignEps) {
            return 1;
        }
        if (value < -kSignEps) {
            return -1;
        }
        return 0;
    };
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50) {
            continue;
        }
        ++planted_support;
        planted_contact_count_sum += mean_leg_contact_count[idx];
        const int drift_sign = signOf(mean_leg_pinocchio_drift_tx[idx]);
        const int slip_sign = signOf(mean_leg_world_slip_tx[idx]);
        if (drift_sign != 0 && slip_sign != 0) {
            if (drift_sign != slip_sign) {
                ++drift_slip_disagree;
            } else {
                ++drift_slip_agree;
            }
        }
        const int fx_sign = signOf(mean_leg_friction_world_x[idx]);
        if (fx_sign > 0) {
            ++friction_pos;
        } else if (fx_sign < 0) {
            ++friction_neg;
        }
    }
    const double mean_contacts_per_planted_tibia =
        planted_support > 0 ? planted_contact_count_sum / planted_support : 0.0;
    const int comparable_drift_slip = drift_slip_agree + drift_slip_disagree;
    const char* named_tangent_census = "unknown";
    if (comparable_drift_slip > 0 && drift_slip_disagree * 2 >= comparable_drift_slip) {
        named_tangent_census = "jacobian_parity";
    } else if (mean_contacts_per_planted_tibia > 1.2) {
        named_tangent_census = "dual_tibia_contact";
    } else if (comparable_drift_slip > 0 && drift_slip_agree * 2 > comparable_drift_slip
               && friction_pos > 0 && friction_neg > 0) {
        named_tangent_census = "same_j_opposite_lambda";
    }

    struct PlantedSignCensus {
        int pos = 0;
        int neg = 0;
        int counted() const { return pos + neg; }
        bool same() const { return counted() >= 2 && (pos == 0 || neg == 0); }
        bool split() const { return pos > 0 && neg > 0; }
    };
    auto censusPlantedSigns = [&](const std::array<double, kNumLegs>& values) {
        PlantedSignCensus census{};
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50) {
                continue;
            }
            const int sign = signOf(values[idx]);
            if (sign > 0) {
                ++census.pos;
            } else if (sign < 0) {
                ++census.neg;
            }
        }
        return census;
    };
    std::array<double, kNumLegs> mean_leg_contact_vx{};
    int spin_dominates_tibia = 0;
    int planted_spin_legs = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        mean_leg_contact_vx[idx] = mean_leg_tibia_vx[idx] + mean_leg_spin_vx[idx];
        if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50) {
            continue;
        }
        ++planted_spin_legs;
        if (std::abs(mean_leg_spin_vx[idx]) > std::abs(mean_leg_tibia_vx[idx])) {
            ++spin_dominates_tibia;
        }
    }
    const PlantedSignCensus tibia_signs = censusPlantedSigns(mean_leg_tibia_vx);
    const PlantedSignCensus foot_signs = censusPlantedSigns(mean_leg_foot_vx);
    const PlantedSignCensus t0_signs = censusPlantedSigns(mean_leg_t0_x);
    const PlantedSignCensus spin_signs = censusPlantedSigns(mean_leg_spin_vx);
    const PlantedSignCensus contact_vx_signs = censusPlantedSigns(mean_leg_contact_vx);
    const bool linear_or_foot_same = tibia_signs.same() || foot_signs.same();
    const char* named_slip_split = "unknown";
    if (linear_or_foot_same && t0_signs.split()) {
        named_slip_split = "t0_flip";
    } else if (linear_or_foot_same && t0_signs.same() && spin_signs.split()
               && planted_spin_legs > 0 && spin_dominates_tibia * 2 > planted_spin_legs) {
        named_slip_split = "spin_offset";
    } else if (tibia_signs.split()) {
        named_slip_split = "tibia_linear_opposite";
    } else if (foot_signs.same() && contact_vx_signs.split()) {
        named_slip_split = "sphere_vs_contact";
    }

    const PlantedSignCensus cmd_signs = censusPlantedSigns(mean_leg_cmd_foot_vx);
    const PlantedSignCensus fk_signs = censusPlantedSigns(mean_leg_fk_foot_vx);
    const PlantedSignCensus pos_vx_signs = censusPlantedSigns(mean_leg_foot_pos_vx);
    auto plantedCommandScaleSame = [&](const std::array<double, kNumLegs>& values,
                                       const PlantedSignCensus& signs) {
        if (!signs.same()) {
            return false;
        }
        int planted = 0;
        int strong = 0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50) {
                continue;
            }
            ++planted;
            if (std::abs(values[idx]) >= 0.50 * vx_mps) {
                ++strong;
            }
        }
        return planted > 0 && strong * 2 >= planted;
    };
    auto plantedTinyMajority = [&](const std::array<double, kNumLegs>& values) {
        int planted = 0;
        int tiny = 0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50) {
                continue;
            }
            ++planted;
            if (std::abs(values[idx]) < 0.20 * vx_mps) {
                ++tiny;
            }
        }
        return planted > 0 && tiny * 2 >= planted;
    };
    const bool cmd_or_fk_command_scale =
        plantedCommandScaleSame(mean_leg_cmd_foot_vx, cmd_signs)
        || plantedCommandScaleSame(mean_leg_fk_foot_vx, fk_signs);
    const char* named_fk_physics = "unknown";
    if (cmd_signs.split() || fk_signs.split()) {
        named_fk_physics = "true_ik_split";
    } else if (
        cmd_or_fk_command_scale
        && plantedCommandScaleSame(mean_leg_foot_pos_vx, pos_vx_signs)
        && (foot_signs.split() || plantedTinyMajority(mean_leg_foot_vx))) {
        named_fk_physics = "qv_stale";
    } else if (
        cmd_or_fk_command_scale
        && plantedCommandScaleSame(mean_leg_foot_pos_vx, pos_vx_signs)
        && plantedCommandScaleSame(mean_leg_foot_vx, foot_signs)
        && pos_vx_signs.split() && foot_signs.split()) {
        named_fk_physics = "ik_physics_axis";
    } else if (
        cmd_or_fk_command_scale
        && (plantedTinyMajority(mean_leg_foot_pos_vx) || pos_vx_signs.split())) {
        named_fk_physics = "fk_vs_sphere";
    }

    const PlantedSignCensus pos_vz_signs = censusPlantedSigns(mean_leg_foot_pos_vz);
    const PlantedSignCensus foot_vz_signs = censusPlantedSigns(mean_leg_foot_vz);
    const PlantedSignCensus expected_vz_signs = censusPlantedSigns(mean_leg_expected_sim_vz);
    const PlantedSignCensus fz_signs = censusPlantedSigns(mean_leg_friction_world_z);
    const PlantedSignCensus ty_signs = censusPlantedSigns(mean_leg_world_slip_ty);
    const char* named_stroke_friction = "unknown";
    if (fz_signs.split()) {
        named_stroke_friction = "opposing_z";
    } else if (fz_signs.same()) {
        named_stroke_friction = "aligned_z";
    }
    (void)foot_vz_signs;
    (void)ty_signs;
    const char* named_stroke_axis = "unknown";
    if (cmd_or_fk_command_scale && plantedCommandScaleSame(mean_leg_foot_pos_vz, pos_vz_signs)
        && pos_vz_signs.same() && expected_vz_signs.same()
        && pos_vz_signs.pos == expected_vz_signs.pos) {
        named_stroke_axis = "sphere_follows_stroke";
    } else if (
        cmd_or_fk_command_scale && plantedCommandScaleSame(mean_leg_foot_pos_vz, pos_vz_signs)
        && pos_vz_signs.same() && expected_vz_signs.same()
        && pos_vz_signs.pos != expected_vz_signs.pos) {
        named_stroke_axis = "sphere_opposite_stroke";
    } else if (cmd_or_fk_command_scale && pos_vz_signs.split()) {
        named_stroke_axis = "sphere_split_stroke";
    } else if (cmd_or_fk_command_scale && plantedTinyMajority(mean_leg_foot_pos_vz)) {
        named_stroke_axis = "sphere_tiny_stroke";
    } else if (
        cmd_or_fk_command_scale
        && (plantedTinyMajority(mean_leg_foot_pos_vx) || pos_vx_signs.split())
        && plantedCommandScaleSame(mean_leg_foot_pos_vz, pos_vz_signs)) {
        named_stroke_axis = "axis_mix";
    }

    const PlantedSignCensus fk_dx_signs = censusPlantedSigns(stroke_fk_dx);
    const PlantedSignCensus mapped_dx_signs = censusPlantedSigns(stroke_mapped_dx);
    int planted_pos_match = 0;
    int planted_pos_offset_only = 0;
    int planted_pos_compared = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        if (!kSupportLegs[idx] || mean_leg_contact_count[idx] < 0.50
            || leg_fk_sphere_samples[idx] == 0) {
            continue;
        }
        ++planted_pos_compared;
        const double abs_x = std::abs(mean_leg_fk_minus_mapped_x[idx]);
        const double abs_y = std::abs(mean_leg_fk_minus_mapped_y[idx]);
        if (abs_x < 0.025 && abs_y < 0.025) {
            ++planted_pos_match;
        } else if (abs_x < 0.12 && abs_y < 0.12) {
            ++planted_pos_offset_only;
        }
    }
    const char* named_fk_sphere_pos = "unknown";
    if (planted_pos_compared >= 2 && planted_pos_match == planted_pos_compared) {
        named_fk_sphere_pos = "match";
    } else if (
        cmd_or_fk_command_scale && plantedCommandScaleSame(stroke_fk_dx, fk_dx_signs)
        && plantedCommandScaleSame(stroke_mapped_dx, mapped_dx_signs) && fk_dx_signs.same()
        && mapped_dx_signs.same() && fk_dx_signs.pos != mapped_dx_signs.pos) {
        named_fk_sphere_pos = "opposite_stroke";
    } else if (planted_pos_compared >= 2
               && (planted_pos_match + planted_pos_offset_only) == planted_pos_compared
               && planted_pos_match < planted_pos_compared) {
        named_fk_sphere_pos = "constant_offset";
    } else if (planted_pos_compared >= 2) {
        named_fk_sphere_pos = "diverge";
    }

    auto plantedRestMatch = [&](const std::array<double, kNumLegs>& dx,
                                const std::array<double, kNumLegs>& dy) {
        int compared = 0;
        int match = 0;
        for (int leg = 0; leg < kNumLegs; ++leg) {
            const std::size_t idx = static_cast<std::size_t>(leg);
            if (!kSupportLegs[idx] || !have_stroke_start_xyz[idx]
                || mean_leg_contact_count[idx] < 0.50) {
                continue;
            }
            ++compared;
            if (std::abs(dx[idx]) < 0.025 && std::abs(dy[idx]) < 0.025) {
                ++match;
            }
        }
        return compared >= 2 && match == compared;
    };
    const bool rest_bridge_match =
        plantedRestMatch(rest_fk_minus_bridge_x, rest_fk_minus_bridge_y);
    const bool rest_align_match = plantedRestMatch(rest_fk_minus_align_x, rest_fk_minus_align_y);
    const char* named_rest_c = "rest_both_offset";
    if (rest_bridge_match && !rest_align_match) {
        named_rest_c = "rest_bridge_match";
    } else if (rest_align_match && !rest_bridge_match) {
        named_rest_c = "rest_align_match";
    } else if (rest_bridge_match && rest_align_match) {
        named_rest_c = "rest_bridge_match";
    }

    const PlantedSignCensus align_dx_signs = censusPlantedSigns(stroke_align_dx);
    const bool stroke_fk_scale = plantedCommandScaleSame(stroke_fk_dx, fk_dx_signs);
    const bool stroke_bridge_scale = plantedCommandScaleSame(stroke_mapped_dx, mapped_dx_signs);
    const bool stroke_align_scale = plantedCommandScaleSame(stroke_align_dx, align_dx_signs);
    const bool stroke_bridge_follows =
        stroke_fk_scale && stroke_bridge_scale && fk_dx_signs.same() && mapped_dx_signs.same()
        && fk_dx_signs.pos == mapped_dx_signs.pos;
    const bool stroke_align_follows =
        stroke_fk_scale && stroke_align_scale && fk_dx_signs.same() && align_dx_signs.same()
        && fk_dx_signs.pos == align_dx_signs.pos;
    const bool stroke_bridge_opposite =
        stroke_fk_scale && stroke_bridge_scale && fk_dx_signs.same() && mapped_dx_signs.same()
        && fk_dx_signs.pos != mapped_dx_signs.pos;
    const bool stroke_align_opposite =
        stroke_fk_scale && stroke_align_scale && fk_dx_signs.same() && align_dx_signs.same()
        && fk_dx_signs.pos != align_dx_signs.pos;
    const bool stroke_90 = stroke_fk_scale && plantedTinyMajority(stroke_align_dx) && stroke_bridge_scale;
    const char* named_stroke_c = "unknown";
    if (stroke_90) {
        named_stroke_c = "stroke_90";
    } else if (stroke_bridge_follows) {
        named_stroke_c = "stroke_bridge_follows";
    } else if (stroke_align_follows) {
        named_stroke_c = "stroke_align_follows";
    } else if (stroke_bridge_opposite) {
        named_stroke_c = "stroke_bridge_opposite";
    } else if (stroke_align_opposite) {
        named_stroke_c = "stroke_align_opposite";
    }

    const char* named_frame_c = "unknown";
    if (rest_align_match && stroke_align_follows) {
        named_frame_c = "align_c_match";
    } else if (rest_bridge_match && stroke_bridge_follows) {
        named_frame_c = "bridge_c_match";
    } else if (rest_align_match && stroke_90) {
        named_frame_c = "align_rest_stroke_90";
    } else if (stroke_bridge_opposite && stroke_align_opposite) {
        named_frame_c = "both_opposite";
    } else if (!rest_bridge_match && !rest_align_match
               && (stroke_bridge_opposite || stroke_align_opposite)) {
        named_frame_c = "both_opposite";
    }

    const bool body_bridge_match =
        plantedRestMatch(rest_body_minus_bridge_x, rest_body_minus_bridge_y);
    const bool body_align_match =
        plantedRestMatch(rest_body_minus_align_x, rest_body_minus_align_y);
    const char* named_body_c = "body_both_offset";
    if (body_align_match && !body_bridge_match) {
        named_body_c = "body_align_match";
    } else if (body_bridge_match && !body_align_match) {
        named_body_c = "body_bridge_match";
    } else if (body_bridge_match && body_align_match) {
        named_body_c = "body_bridge_match";
    }
    if (body_align_match && stroke_90) {
        named_frame_c = "align_rest_stroke_90";
    }

    auto classifyOffset = [](const double fx, const double fy, const double mx, const double my) {
        const double dx = fx - mx;
        const double dy = fy - my;
        const double err = std::hypot(dx, dy);
        if (err < 0.035) {
            return "match";
        }
        const double ccw = std::hypot(mx + fy, my - fx);
        const double cw = std::hypot(mx - fy, my + fx);
        if (ccw < 0.050 || cw < 0.050) {
            return "swap_90";
        }
        if (err > 0.050 && std::abs(std::abs(dx) - std::abs(dy)) < 0.25 * err) {
            return "diag";
        }
        return "other";
    };
    std::array<const char*, kNumLegs> rest_leg_class{};
    rest_leg_class.fill("none");
    int rest_n_match = 0;
    int rest_n_swap = 0;
    int rest_n_diag = 0;
    int rest_n_other = 0;
    int rest_n_planted = 0;
    double residual_dot_sum = 0.0;
    double residual_ref_x = 0.0;
    double residual_ref_y = 0.0;
    bool have_residual_ref = false;
    int residual_same_dir = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        if (!kSupportLegs[idx] || !have_stroke_start_xyz[idx]
            || mean_leg_contact_count[idx] < 0.50) {
            continue;
        }
        ++rest_n_planted;
        const char* cls = classifyOffset(
            rest_body_x[idx], rest_body_y[idx], rest_sphere_align_x[idx], rest_sphere_align_y[idx]);
        rest_leg_class[idx] = cls;
        if (std::strcmp(cls, "match") == 0) {
            ++rest_n_match;
        } else if (std::strcmp(cls, "swap_90") == 0) {
            ++rest_n_swap;
        } else if (std::strcmp(cls, "diag") == 0) {
            ++rest_n_diag;
        } else {
            ++rest_n_other;
        }
        const double dx = rest_body_minus_align_x[idx];
        const double dy = rest_body_minus_align_y[idx];
        if (!have_residual_ref && std::hypot(dx, dy) > 0.025) {
            residual_ref_x = dx;
            residual_ref_y = dy;
            have_residual_ref = true;
        }
        if (have_residual_ref) {
            residual_dot_sum += dx * residual_ref_x + dy * residual_ref_y;
            if (dx * residual_ref_x + dy * residual_ref_y > 0.0) {
                ++residual_same_dir;
            }
        }
    }
    const char* left_cls = rest_leg_class[1];
    const char* left_cls_b = rest_leg_class[5];
    const char* right_cls = rest_leg_class[2];
    const bool left_agree =
        std::strcmp(left_cls, "none") != 0 && std::strcmp(left_cls, left_cls_b) == 0;
    const bool left_right_split =
        left_agree && std::strcmp(right_cls, "none") != 0 && std::strcmp(left_cls, right_cls) != 0;
    const char* named_rest_offset = "unknown";
    if (rest_n_planted >= 2 && rest_n_match == rest_n_planted) {
        named_rest_offset = "match";
    } else if (left_right_split) {
        named_rest_offset = "left_right";
    } else if (rest_n_planted >= 2 && (rest_n_swap + rest_n_diag) == rest_n_planted) {
        named_rest_offset = "swap_90";
    } else if (
        rest_n_planted >= 2 && rest_n_other * 2 >= rest_n_planted && have_residual_ref
        && residual_same_dir == rest_n_planted
        && residual_dot_sum > 0.0) {
        named_rest_offset = "translation";
    } else if (rest_n_planted >= 2 && (rest_n_match > 0 || rest_n_swap > 0 || rest_n_diag > 0)
               && (rest_n_match + rest_n_swap + rest_n_diag + rest_n_other) == rest_n_planted
               && (rest_n_match + rest_n_swap + rest_n_diag) < rest_n_planted) {
        named_rest_offset = "per_leg_split";
    } else if (rest_n_planted >= 2 && rest_n_match > 0
               && rest_n_match < rest_n_planted) {
        named_rest_offset = "per_leg_split";
    }

    std::array<const char*, kNumLegs> plant_leg_class{};
    plant_leg_class.fill("none");
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const std::size_t idx = static_cast<std::size_t>(leg);
        if (!kSupportLegs[idx] || !have_stroke_start_xyz[idx]) {
            continue;
        }
        plant_leg_class[idx] = classifyOffset(
            captured_world[idx].x,
            captured_world[idx].y,
            captured_world[idx].x - plant_minus_align_x[idx],
            captured_world[idx].y - plant_minus_align_y[idx]);
    }

    const double max_coxa_err = max_joint_tracking_error_rad[static_cast<std::size_t>(COXA)];
    const double max_femur_err = max_joint_tracking_error_rad[static_cast<std::size_t>(FEMUR)];
    const double max_tibia_err = max_joint_tracking_error_rad[static_cast<std::size_t>(TIBIA)];
    const char* named_tracking = "tracking_mixed";
    const double tracking_peak = std::max(max_coxa_err, std::max(max_femur_err, max_tibia_err));
    if (tracking_peak >= 0.15) {
        if (max_coxa_err >= 2.0 * max_femur_err && max_coxa_err >= 2.0 * max_tibia_err) {
            named_tracking = "tracking_coxa";
        } else if (max_femur_err >= 2.0 * max_coxa_err && max_femur_err >= 2.0 * max_tibia_err) {
            named_tracking = "tracking_femur";
        } else if (max_tibia_err >= 2.0 * max_coxa_err && max_tibia_err >= 2.0 * max_femur_err) {
            named_tracking = "tracking_tibia";
        }
    }

    const bool have_impulse = friction_ratio_samples > 0 && mean_peak_normal_impulse_ns > 1.0e-12;
    const char* named_h4_cause = "unknown";
    if (std::strcmp(named_unload_cause, "unloaded") == 0 && progress_frac < 0.50) {
        if (support_contact_fraction < 0.50 || mean_n_support_contact < 2.0
            || (have_impulse && mean_peak_normal_impulse_ns < 1.0e-5)) {
            named_h4_cause = "light_normal";
        } else if ((mean_servo_torque_utilization >= 0.80
                    || max_servo_tracking_error_rad >= 0.15
                    || mean_max_contact_penetration >= 0.020)
                   && slip_ratio >= 0.35) {
            named_h4_cause = "servo_cone";
        } else if (contact_world_speed >= 0.50 * vx_mps && slip_ratio < 0.35) {
            named_h4_cause = "coulomb_skate";
        }
    }

    std::cout << "{\"suite\":\"physics_sim_tripod_stroke_probe\""
              << ",\"solver_mode\":\""
              << (solver.mode == physics_sim::PhysicsSolverMode::LegacyPgs
                      ? "legacy-pgs"
                      : "pinocchio-proximal")
              << "\",\"metrics\":{"
              << "\"stroke_frames\":" << stroke_frames
              << ",\"foot_radius_m\":" << kFootRadiusM
              << ",\"raised_world_z_target_m\":" << raised_world_z
              << ",\"named_unload_cause\":\"" << named_unload_cause << "\""
              << ",\"named_h4_cause\":\"" << named_h4_cause << "\""
              << ",\"named_coupling\":\"" << named_coupling << "\""
              << ",\"named_mapping\":\"" << named_mapping << "\""
              << ",\"named_tangent_census\":\"" << named_tangent_census << "\""
              << ",\"named_slip_split\":\"" << named_slip_split << "\""
              << ",\"named_fk_physics\":\"" << named_fk_physics << "\""
              << ",\"named_stroke_axis\":\"" << named_stroke_axis << "\""
              << ",\"named_stroke_friction\":\"" << named_stroke_friction << "\""
              << ",\"named_fk_sphere_pos\":\"" << named_fk_sphere_pos << "\""
              << ",\"named_rest_c\":\"" << named_rest_c << "\""
              << ",\"named_stroke_c\":\"" << named_stroke_c << "\""
              << ",\"named_frame_c\":\"" << named_frame_c << "\""
              << ",\"named_body_c\":\"" << named_body_c << "\""
              << ",\"named_rest_offset\":\"" << named_rest_offset << "\""
              << ",\"named_tracking\":\"" << named_tracking << "\""
              << ",\"stroke_vx_mps\":" << vx_mps
              << ",\"servo_torque_scale\":" << torque_scale
              << ",\"hexapod_mass_kg\":" << kHexapodMassKg
              << ",\"command_progress_m\":" << command_progress
              << ",\"commanded_translation_m\":" << commanded_translation
              << ",\"mean_n_raw_contact\":" << mean_n_raw_contact
              << ",\"mean_n_support_contact\":" << mean_n_support_contact
              << ",\"clean_tripod_frames\":" << clean_tripod_frames
              << ",\"clean_tripod_frame_fraction\":"
              << meanValue(static_cast<double>(clean_tripod_frames), census_frames)
              << ",\"mean_abs_body_pitch_rad\":" << meanValue(abs_pitch_sum, census_frames)
              << ",\"mean_cartesian_opposition_speed_mps\":"
              << meanRate(cartesian_opposition_sum, cartesian_samples)
              << ",\"mean_contact_commanded_world_speed_mps\":" << commanded_world_speed
              << ",\"mean_contact_uncommanded_slip_speed_mps\":" << uncommanded_slip_speed
              << ",\"mean_contact_world_speed_mps\":" << contact_world_speed
              << ",\"mean_peak_normal_impulse_ns\":" << mean_peak_normal_impulse_ns
              << ",\"mean_peak_friction_impulse_ns\":" << mean_peak_friction_impulse_ns
              << ",\"mean_friction_to_normal_impulse_ratio\":"
              << mean_friction_to_normal_impulse_ratio
              << ",\"mean_servo_torque_utilization\":" << mean_servo_torque_utilization
              << ",\"max_servo_tracking_error_rad\":" << max_servo_tracking_error_rad
              << ",\"max_coxa_tracking_error_rad\":" << max_coxa_err
              << ",\"max_femur_tracking_error_rad\":" << max_femur_err
              << ",\"max_tibia_tracking_error_rad\":" << max_tibia_err
              << ",\"mean_cone_residual\":" << mean_cone_residual
              << ",\"mean_max_contact_penetration\":" << mean_max_contact_penetration
              << ",\"mean_body_vx_mps\":" << mean_body_vx_mps
              << ",\"mean_support_foot_world_vx_mps\":" << mean_support_foot_world_vx_mps
              << ",\"expected_com_delta_v_from_friction_mps\":" << expected_com_delta_v_sum
              << ",\"friction_com_coupling_ratio\":" << friction_com_coupling_ratio
              << ",\"mean_sum_friction_impulse_world_x_ns\":" << mean_sum_friction_impulse_world_x
              << ",\"mean_sum_friction_impulse_world_z_ns\":" << mean_sum_friction_impulse_world_z
              << ",\"mean_sum_abs_friction_impulse_world_x_ns\":" << mean_sum_abs_friction_impulse_world_x
              << ",\"mean_sum_abs_friction_impulse_world_z_ns\":" << mean_sum_abs_friction_impulse_world_z
              << ",\"mean_sum_friction_impulse_world_y_ns\":" << mean_sum_friction_impulse_world_y
              << ",\"mean_contact_delta_vx_mps\":" << mean_contact_delta_vx
              << ",\"mean_contact_delta_vz_mps\":" << mean_contact_delta_vz
              << ",\"friction_axis_x_fraction\":" << friction_axis_x_fraction
              << ",\"friction_horizontal_capture\":" << friction_horizontal_capture
              << ",\"friction_x_cancellation\":" << friction_x_cancellation
              << ",\"jacobian_com_ratio\":" << jacobian_com_ratio
              << ",\"mean_contacts_per_planted_tibia\":" << mean_contacts_per_planted_tibia
              << ",\"drift_slip_disagree_legs\":" << drift_slip_disagree
              << ",\"drift_slip_agree_legs\":" << drift_slip_agree
              << ",\"mean_leg_friction_impulse_world_x_ns\":["
              << mean_leg_friction_world_x[0] << "," << mean_leg_friction_world_x[1] << ","
              << mean_leg_friction_world_x[2] << "," << mean_leg_friction_world_x[3] << ","
              << mean_leg_friction_world_x[4] << "," << mean_leg_friction_world_x[5] << "]"
              << ",\"mean_leg_pinocchio_drift_tx_mps\":["
              << mean_leg_pinocchio_drift_tx[0] << "," << mean_leg_pinocchio_drift_tx[1] << ","
              << mean_leg_pinocchio_drift_tx[2] << "," << mean_leg_pinocchio_drift_tx[3] << ","
              << mean_leg_pinocchio_drift_tx[4] << "," << mean_leg_pinocchio_drift_tx[5] << "]"
              << ",\"mean_leg_world_slip_tx_mps\":["
              << mean_leg_world_slip_tx[0] << "," << mean_leg_world_slip_tx[1] << ","
              << mean_leg_world_slip_tx[2] << "," << mean_leg_world_slip_tx[3] << ","
              << mean_leg_world_slip_tx[4] << "," << mean_leg_world_slip_tx[5] << "]"
              << ",\"mean_leg_contact_count\":["
              << mean_leg_contact_count[0] << "," << mean_leg_contact_count[1] << ","
              << mean_leg_contact_count[2] << "," << mean_leg_contact_count[3] << ","
              << mean_leg_contact_count[4] << "," << mean_leg_contact_count[5] << "]"
              << ",\"mean_leg_tibia_vx_mps\":["
              << mean_leg_tibia_vx[0] << "," << mean_leg_tibia_vx[1] << ","
              << mean_leg_tibia_vx[2] << "," << mean_leg_tibia_vx[3] << ","
              << mean_leg_tibia_vx[4] << "," << mean_leg_tibia_vx[5] << "]"
              << ",\"mean_leg_spin_vx_mps\":["
              << mean_leg_spin_vx[0] << "," << mean_leg_spin_vx[1] << ","
              << mean_leg_spin_vx[2] << "," << mean_leg_spin_vx[3] << ","
              << mean_leg_spin_vx[4] << "," << mean_leg_spin_vx[5] << "]"
              << ",\"mean_leg_t0_x\":["
              << mean_leg_t0_x[0] << "," << mean_leg_t0_x[1] << ","
              << mean_leg_t0_x[2] << "," << mean_leg_t0_x[3] << ","
              << mean_leg_t0_x[4] << "," << mean_leg_t0_x[5] << "]"
              << ",\"mean_leg_foot_vx_mps\":["
              << mean_leg_foot_vx[0] << "," << mean_leg_foot_vx[1] << ","
              << mean_leg_foot_vx[2] << "," << mean_leg_foot_vx[3] << ","
              << mean_leg_foot_vx[4] << "," << mean_leg_foot_vx[5] << "]"
              << ",\"mean_leg_foot_x_m\":["
              << mean_leg_foot_x[0] << "," << mean_leg_foot_x[1] << ","
              << mean_leg_foot_x[2] << "," << mean_leg_foot_x[3] << ","
              << mean_leg_foot_x[4] << "," << mean_leg_foot_x[5] << "]"
              << ",\"mean_leg_foot_pos_vx_mps\":["
              << mean_leg_foot_pos_vx[0] << "," << mean_leg_foot_pos_vx[1] << ","
              << mean_leg_foot_pos_vx[2] << "," << mean_leg_foot_pos_vx[3] << ","
              << mean_leg_foot_pos_vx[4] << "," << mean_leg_foot_pos_vx[5] << "]"
              << ",\"mean_leg_cmd_foot_vx_mps\":["
              << mean_leg_cmd_foot_vx[0] << "," << mean_leg_cmd_foot_vx[1] << ","
              << mean_leg_cmd_foot_vx[2] << "," << mean_leg_cmd_foot_vx[3] << ","
              << mean_leg_cmd_foot_vx[4] << "," << mean_leg_cmd_foot_vx[5] << "]"
              << ",\"mean_leg_fk_foot_vx_mps\":["
              << mean_leg_fk_foot_vx[0] << "," << mean_leg_fk_foot_vx[1] << ","
              << mean_leg_fk_foot_vx[2] << "," << mean_leg_fk_foot_vx[3] << ","
              << mean_leg_fk_foot_vx[4] << "," << mean_leg_fk_foot_vx[5] << "]"
              << ",\"mean_leg_expected_sim_vz_mps\":["
              << mean_leg_expected_sim_vz[0] << "," << mean_leg_expected_sim_vz[1] << ","
              << mean_leg_expected_sim_vz[2] << "," << mean_leg_expected_sim_vz[3] << ","
              << mean_leg_expected_sim_vz[4] << "," << mean_leg_expected_sim_vz[5] << "]"
              << ",\"mean_leg_foot_vz_mps\":["
              << mean_leg_foot_vz[0] << "," << mean_leg_foot_vz[1] << ","
              << mean_leg_foot_vz[2] << "," << mean_leg_foot_vz[3] << ","
              << mean_leg_foot_vz[4] << "," << mean_leg_foot_vz[5] << "]"
              << ",\"mean_leg_foot_z_m\":["
              << mean_leg_foot_z[0] << "," << mean_leg_foot_z[1] << ","
              << mean_leg_foot_z[2] << "," << mean_leg_foot_z[3] << ","
              << mean_leg_foot_z[4] << "," << mean_leg_foot_z[5] << "]"
              << ",\"mean_leg_foot_pos_vz_mps\":["
              << mean_leg_foot_pos_vz[0] << "," << mean_leg_foot_pos_vz[1] << ","
              << mean_leg_foot_pos_vz[2] << "," << mean_leg_foot_pos_vz[3] << ","
              << mean_leg_foot_pos_vz[4] << "," << mean_leg_foot_pos_vz[5] << "]"
              << ",\"mean_leg_friction_impulse_world_z_ns\":["
              << mean_leg_friction_world_z[0] << "," << mean_leg_friction_world_z[1] << ","
              << mean_leg_friction_world_z[2] << "," << mean_leg_friction_world_z[3] << ","
              << mean_leg_friction_world_z[4] << "," << mean_leg_friction_world_z[5] << "]"
              << ",\"mean_leg_world_slip_ty_mps\":["
              << mean_leg_world_slip_ty[0] << "," << mean_leg_world_slip_ty[1] << ","
              << mean_leg_world_slip_ty[2] << "," << mean_leg_world_slip_ty[3] << ","
              << mean_leg_world_slip_ty[4] << "," << mean_leg_world_slip_ty[5] << "]"
              << ",\"mean_leg_fk_minus_mapped_x_m\":["
              << mean_leg_fk_minus_mapped_x[0] << "," << mean_leg_fk_minus_mapped_x[1] << ","
              << mean_leg_fk_minus_mapped_x[2] << "," << mean_leg_fk_minus_mapped_x[3] << ","
              << mean_leg_fk_minus_mapped_x[4] << "," << mean_leg_fk_minus_mapped_x[5] << "]"
              << ",\"mean_leg_fk_minus_mapped_y_m\":["
              << mean_leg_fk_minus_mapped_y[0] << "," << mean_leg_fk_minus_mapped_y[1] << ","
              << mean_leg_fk_minus_mapped_y[2] << "," << mean_leg_fk_minus_mapped_y[3] << ","
              << mean_leg_fk_minus_mapped_y[4] << "," << mean_leg_fk_minus_mapped_y[5] << "]"
              << ",\"stroke_fk_dx_m\":["
              << stroke_fk_dx[0] << "," << stroke_fk_dx[1] << ","
              << stroke_fk_dx[2] << "," << stroke_fk_dx[3] << ","
              << stroke_fk_dx[4] << "," << stroke_fk_dx[5] << "]"
              << ",\"stroke_mapped_dx_m\":["
              << stroke_mapped_dx[0] << "," << stroke_mapped_dx[1] << ","
              << stroke_mapped_dx[2] << "," << stroke_mapped_dx[3] << ","
              << stroke_mapped_dx[4] << "," << stroke_mapped_dx[5] << "]"
              << ",\"stroke_align_dx_m\":["
              << stroke_align_dx[0] << "," << stroke_align_dx[1] << ","
              << stroke_align_dx[2] << "," << stroke_align_dx[3] << ","
              << stroke_align_dx[4] << "," << stroke_align_dx[5] << "]"
              << ",\"rest_fk_minus_bridge_x_m\":["
              << rest_fk_minus_bridge_x[0] << "," << rest_fk_minus_bridge_x[1] << ","
              << rest_fk_minus_bridge_x[2] << "," << rest_fk_minus_bridge_x[3] << ","
              << rest_fk_minus_bridge_x[4] << "," << rest_fk_minus_bridge_x[5] << "]"
              << ",\"rest_fk_minus_bridge_y_m\":["
              << rest_fk_minus_bridge_y[0] << "," << rest_fk_minus_bridge_y[1] << ","
              << rest_fk_minus_bridge_y[2] << "," << rest_fk_minus_bridge_y[3] << ","
              << rest_fk_minus_bridge_y[4] << "," << rest_fk_minus_bridge_y[5] << "]"
              << ",\"rest_fk_minus_align_x_m\":["
              << rest_fk_minus_align_x[0] << "," << rest_fk_minus_align_x[1] << ","
              << rest_fk_minus_align_x[2] << "," << rest_fk_minus_align_x[3] << ","
              << rest_fk_minus_align_x[4] << "," << rest_fk_minus_align_x[5] << "]"
              << ",\"rest_fk_minus_align_y_m\":["
              << rest_fk_minus_align_y[0] << "," << rest_fk_minus_align_y[1] << ","
              << rest_fk_minus_align_y[2] << "," << rest_fk_minus_align_y[3] << ","
              << rest_fk_minus_align_y[4] << "," << rest_fk_minus_align_y[5] << "]"
              << ",\"rest_body_minus_bridge_x_m\":["
              << rest_body_minus_bridge_x[0] << "," << rest_body_minus_bridge_x[1] << ","
              << rest_body_minus_bridge_x[2] << "," << rest_body_minus_bridge_x[3] << ","
              << rest_body_minus_bridge_x[4] << "," << rest_body_minus_bridge_x[5] << "]"
              << ",\"rest_body_minus_bridge_y_m\":["
              << rest_body_minus_bridge_y[0] << "," << rest_body_minus_bridge_y[1] << ","
              << rest_body_minus_bridge_y[2] << "," << rest_body_minus_bridge_y[3] << ","
              << rest_body_minus_bridge_y[4] << "," << rest_body_minus_bridge_y[5] << "]"
              << ",\"rest_body_minus_align_x_m\":["
              << rest_body_minus_align_x[0] << "," << rest_body_minus_align_x[1] << ","
              << rest_body_minus_align_x[2] << "," << rest_body_minus_align_x[3] << ","
              << rest_body_minus_align_x[4] << "," << rest_body_minus_align_x[5] << "]"
              << ",\"rest_body_minus_align_y_m\":["
              << rest_body_minus_align_y[0] << "," << rest_body_minus_align_y[1] << ","
              << rest_body_minus_align_y[2] << "," << rest_body_minus_align_y[3] << ","
              << rest_body_minus_align_y[4] << "," << rest_body_minus_align_y[5] << "]"
              << ",\"rest_leg_class\":[\""
              << rest_leg_class[0] << "\",\"" << rest_leg_class[1] << "\",\""
              << rest_leg_class[2] << "\",\"" << rest_leg_class[3] << "\",\""
              << rest_leg_class[4] << "\",\"" << rest_leg_class[5] << "\"]"
              << ",\"plant_leg_class\":[\""
              << plant_leg_class[0] << "\",\"" << plant_leg_class[1] << "\",\""
              << plant_leg_class[2] << "\",\"" << plant_leg_class[3] << "\",\""
              << plant_leg_class[4] << "\",\"" << plant_leg_class[5] << "\"]"
              << ",\"plant_minus_align_x_m\":["
              << plant_minus_align_x[0] << "," << plant_minus_align_x[1] << ","
              << plant_minus_align_x[2] << "," << plant_minus_align_x[3] << ","
              << plant_minus_align_x[4] << "," << plant_minus_align_x[5] << "]"
              << ",\"plant_minus_align_y_m\":["
              << plant_minus_align_y[0] << "," << plant_minus_align_y[1] << ","
              << plant_minus_align_y[2] << "," << plant_minus_align_y[3] << ","
              << plant_minus_align_y[4] << "," << plant_minus_align_y[5] << "]"
              << ",\"plant_minus_bridge_x_m\":["
              << plant_minus_bridge_x[0] << "," << plant_minus_bridge_x[1] << ","
              << plant_minus_bridge_x[2] << "," << plant_minus_bridge_x[3] << ","
              << plant_minus_bridge_x[4] << "," << plant_minus_bridge_x[5] << "]"
              << ",\"plant_minus_bridge_y_m\":["
              << plant_minus_bridge_y[0] << "," << plant_minus_bridge_y[1] << ","
              << plant_minus_bridge_y[2] << "," << plant_minus_bridge_y[3] << ","
              << plant_minus_bridge_y[4] << "," << plant_minus_bridge_y[5] << "]"
              << ",\"telemetry_frames\":" << telemetry_frames;
    emitGroup("raise_warmup_raised", raise_warmup_raised);
    emitGroup("raise_warmup_support", raise_warmup_support);
    emitGroup("stroke_raised", stroke_raised);
    emitGroup("stroke_support", stroke_support);
    std::cout << "}}\n";
    (void)emit_metrics_json;
    return 0;
#endif
}
