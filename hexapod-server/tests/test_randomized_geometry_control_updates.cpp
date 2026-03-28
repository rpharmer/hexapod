#include "control_pipeline.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "reach_envelope.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>

namespace {

constexpr int kGeometrySamples = 64;
constexpr int kStepsPerGeometry = 12;

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool finiteVec(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool finiteJointTargets(const JointTargets& targets) {
    for (const auto& leg : targets.leg_states) {
        for (const auto& joint : leg.joint_state) {
            if (!std::isfinite(joint.pos_rad.value)) {
                return false;
            }
        }
    }
    return true;
}

bool finiteLegTargets(const LegTargets& targets) {
    for (const auto& foot : targets.feet) {
        if (!finiteVec(foot.pos_body_m.raw()) || !finiteVec(foot.vel_body_mps.raw())) {
            return false;
        }
    }
    return true;
}

bool finiteRotationMatrix(const Mat3& m) {
    for (const auto& row : m.m) {
        for (double entry : row) {
            if (!std::isfinite(entry)) {
                return false;
            }
        }
    }

    const double det = det3x3(
        m.m[0][0], m.m[0][1], m.m[0][2],
        m.m[1][0], m.m[1][1], m.m[1][2],
        m.m[2][0], m.m[2][1], m.m[2][2]);
    return std::isfinite(det) && std::fabs(det - 1.0) < 1e-6;
}

HexapodGeometry randomGeometry(std::mt19937& rng) {
    HexapodGeometry geo = geometry_config::buildDefaultHexapodGeometry();

    std::uniform_real_distribution<double> coxa_dist(0.032, 0.060);
    std::uniform_real_distribution<double> femur_dist(0.048, 0.092);
    std::uniform_real_distribution<double> tibia_dist(0.080, 0.150);
    std::uniform_real_distribution<double> mount_jitter_deg(-14.0, 14.0);
    std::uniform_real_distribution<double> offset_jitter_xy(-0.015, 0.015);
    std::uniform_real_distribution<double> offset_jitter_z(-0.010, 0.004);
    std::uniform_real_distribution<double> servo_attach_jitter_deg(-18.0, 18.0);

    const double coxa = coxa_dist(rng);
    const double femur = femur_dist(rng);
    const double tibia = tibia_dist(rng);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        auto& leg_geo = geo.legGeometry[leg];
        leg_geo.coxaLength = LengthM{coxa};
        leg_geo.femurLength = LengthM{femur};
        leg_geo.tibiaLength = LengthM{tibia};

        leg_geo.mountAngle = AngleRad{
            leg_geo.mountAngle.value + deg2rad(mount_jitter_deg(rng))};
        leg_geo.bodyCoxaOffset.x += offset_jitter_xy(rng);
        leg_geo.bodyCoxaOffset.y += offset_jitter_xy(rng);
        leg_geo.bodyCoxaOffset.z += offset_jitter_z(rng);

        leg_geo.servo.coxaOffset.value += deg2rad(servo_attach_jitter_deg(rng));
        leg_geo.servo.femurOffset.value += deg2rad(servo_attach_jitter_deg(rng));
        leg_geo.servo.tibiaOffset.value += deg2rad(servo_attach_jitter_deg(rng));
    }

    geo.toBottom = LengthM{0.028 + std::uniform_real_distribution<double>(0.0, 0.020)(rng)};
    return geo;
}

bool runRandomizedGeometryStress() {
    std::mt19937 rng(0xBADC0DEu);

    std::uniform_real_distribution<double> speed_dist(0.0, 0.12);
    std::uniform_real_distribution<double> heading_dist(-kPi, kPi);
    std::uniform_real_distribution<double> body_xy_dist(-0.035, 0.035);
    std::uniform_real_distribution<double> body_z_dist(0.15, 0.24);
    std::uniform_real_distribution<double> roll_pitch_dist(-0.12, 0.12);
    std::uniform_real_distribution<double> yaw_dist(-0.30, 0.30);
    std::uniform_real_distribution<double> yaw_rate_dist(-0.45, 0.45);

    const HexapodGeometry original = geometry_config::kHexapodGeometry;

    for (int sample = 0; sample < kGeometrySamples; ++sample) {
        const HexapodGeometry geo = randomGeometry(rng);
        geometry_config::kHexapodGeometry = geo;

        ControlPipeline pipeline;
        LegFK fk;

        RobotState estimated{};
        estimated.timestamp_us = now_us();
        estimated.foot_contacts = {true, true, true, true, true, true};
        estimated.has_measured_body_pose_state = true;

        for (int leg = 0; leg < kNumLegs; ++leg) {
            LegState neutral{};
            neutral.joint_state[0].pos_rad = AngleRad{0.0};
            neutral.joint_state[1].pos_rad = AngleRad{-0.55};
            neutral.joint_state[2].pos_rad = AngleRad{-0.95};
            estimated.leg_states[leg] = geo.legGeometry[leg].servo.toServoAngles(neutral);
        }

        SafetyState safety{};
        safety.inhibit_motion = false;
        safety.torque_cut = false;
        safety.support_contact_count = 6;
        safety.stability_margin_m = 0.03;

        for (int step = 0; step < kStepsPerGeometry; ++step) {
            MotionIntent intent{};
            intent.requested_mode = RobotMode::WALK;
            intent.gait = GaitType::TRIPOD;
            intent.speed_mps = LinearRateMps{speed_dist(rng)};
            intent.heading_rad = AngleRad{heading_dist(rng)};
            intent.body_pose_setpoint.body_trans_m = Vec3{body_xy_dist(rng), body_xy_dist(rng), body_z_dist(rng)};
            intent.body_pose_setpoint.orientation_rad = Vec3{roll_pitch_dist(rng), roll_pitch_dist(rng), yaw_dist(rng)};
            intent.body_pose_setpoint.angular_velocity_radps = Vec3{0.0, 0.0, yaw_rate_dist(rng)};
            intent.timestamp_us = now_us();

            const Mat3 body_rotation = Mat3::rotZ(intent.body_pose_setpoint.orientation_rad.z) *
                                       Mat3::rotY(intent.body_pose_setpoint.orientation_rad.y) *
                                       Mat3::rotX(intent.body_pose_setpoint.orientation_rad.x);

            const PipelineStepResult result = pipeline.runStep(
                estimated, intent, safety, DurationSec{0.02}, true, static_cast<uint64_t>(step));

            if (!expect(finiteRotationMatrix(body_rotation), "body rotation matrix must stay finite and orthonormal") ||
                !expect(finiteJointTargets(result.joint_targets), "joint targets must stay finite") ||
                !expect(finiteLegTargets(result.leg_targets), "leg targets must stay finite")) {
                geometry_config::kHexapodGeometry = original;
                return false;
            }

            for (int leg = 0; leg < kNumLegs; ++leg) {
                const LegGeometry& leg_geo = geo.legGeometry[leg];
                const Vec3 relative = result.leg_targets.feet[leg].pos_body_m - leg_geo.bodyCoxaOffset;
                const Vec3 foot_leg = Mat3::rotZ(-leg_geo.mountAngle.value) * relative;
                const double utilization = kinematics::legReachUtilization(foot_leg, leg_geo);
                if (!expect(std::isfinite(utilization), "reach utilization must stay finite") ||
                    !expect(utilization <= 1.0 + 1e-6, "reach clamp invariant must hold")) {
                    geometry_config::kHexapodGeometry = original;
                    return false;
                }
            }

            estimated.leg_states = result.joint_targets.leg_states;
            estimated.timestamp_us = now_us();

            const LegTargets fk_out = fk.solve(estimated, safety);
            if (!expect(finiteLegTargets(fk_out), "forward kinematics outputs must stay finite")) {
                geometry_config::kHexapodGeometry = original;
                return false;
            }
        }
    }

    geometry_config::kHexapodGeometry = original;
    return true;
}

} // namespace

int main() {
    if (!runRandomizedGeometryStress()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
