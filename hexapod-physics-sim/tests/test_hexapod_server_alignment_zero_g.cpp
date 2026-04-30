#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "config/geometry_config.hpp"
#include "hardware/physics_sim_joint_wire_mapping.hpp"

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

::Vec3 bodyLocalSimToServer(const minphys3d::Vec3& v) {
    return ::Vec3{static_cast<double>(v.x), static_cast<double>(v.z), static_cast<double>(v.y)};
}

minphys3d::Vec3 worldAnchorOnBodyA(const World& world, std::uint32_t joint_id) {
    const ServoJoint& joint = world.GetServoJoint(joint_id);
    const Body& body_a = world.GetBody(joint.a);
    return body_a.position + Rotate(body_a.orientation, joint.localAnchorA);
}

minphys3d::Vec3 footContactPointWorld(const Body& tibia) {
    for (const CompoundChild& child : tibia.compoundChildren) {
        if (child.shape == ShapeType::Sphere) {
            const minphys3d::Vec3 center_world = tibia.position + Rotate(tibia.orientation, child.localPosition);
            const minphys3d::Vec3 foot_axis_world = Rotate(tibia.orientation, minphys3d::Vec3{1.0f, 0.0f, 0.0f});
            return center_world + foot_axis_world * child.radius;
        }
    }
    return tibia.position;
}

double maxAbsComponent(const ::Vec3& v) {
    return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
}

struct ExpectedLegPose {
    ::Vec3 coxa_anchor_body{};
    ::Vec3 coxa_femur_joint_body{};
    ::Vec3 femur_tibia_joint_body{};
    ::Vec3 foot_center_body{};
};

ExpectedLegPose computeExpectedLegPose(const ::LegGeometry& leg, const ::LegState& mech_leg) {
    const double q1 = mech_leg.joint_state[COXA].pos_rad.value;
    const double q2 = mech_leg.joint_state[FEMUR].pos_rad.value;
    const double q3 = mech_leg.joint_state[TIBIA].pos_rad.value;

    const ::Mat3 R_body_from_leg = ::Mat3::rotZ(leg.mountAngle.value);

    const ::Vec3 local_coxa_tip{
        leg.coxaLength.value * std::cos(q1),
        leg.coxaLength.value * std::sin(q1),
        0.0,
    };

    const ::Vec3 local_femur_tip{
        leg.coxaLength.value * std::cos(q1) + leg.femurLength.value * std::cos(q1) * std::cos(q2),
        leg.coxaLength.value * std::sin(q1) + leg.femurLength.value * std::sin(q1) * std::cos(q2),
        leg.femurLength.value * std::sin(q2),
    };

    const ::Vec3 local_foot_tip{
        local_femur_tip.x + leg.tibiaLength.value * std::cos(q1) * std::cos(q2 + q3),
        local_femur_tip.y + leg.tibiaLength.value * std::sin(q1) * std::cos(q2 + q3),
        local_femur_tip.z + leg.tibiaLength.value * std::sin(q2 + q3),
    };

    ExpectedLegPose pose{};
    pose.coxa_anchor_body = leg.bodyCoxaOffset;
    pose.coxa_femur_joint_body = leg.bodyCoxaOffset + (R_body_from_leg * local_coxa_tip);
    pose.femur_tibia_joint_body = leg.bodyCoxaOffset + (R_body_from_leg * local_femur_tip);
    pose.foot_center_body = leg.bodyCoxaOffset + (R_body_from_leg * local_foot_tip);
    return pose;
}

} // namespace

int main() {
    World world({0.0f, 0.0f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxZeroGravityHexapodServos(world, scene);

    const Body& chassis = world.GetBody(scene.body);
    const ::HexapodGeometry server_geo = geometry_config::buildDefaultHexapodGeometry();

    bool ok = true;
    double max_anchor_error_m = 0.0;
    double max_foot_error_m = 0.0;

    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        const LegLinkIds& sim_leg = scene.legs[leg];
        const ::LegGeometry& leg_geo = server_geo.legGeometry[leg];
        const ::ServoCalibration& cal = leg_geo.servo;

        const float sim_q0 = world.GetServoJointAngle(sim_leg.bodyToCoxaJoint);
        const float sim_q1 = world.GetServoJointAngle(sim_leg.coxaToFemurJoint);
        const float sim_q2 = world.GetServoJointAngle(sim_leg.femurToTibiaJoint);
        const ::LegState mech =
            physics_sim_joint_wire_mapping::jointMechanicalFromSimWireAngles(cal,
                                                                             static_cast<int>(leg),
                                                                             sim_q0,
                                                                             sim_q1,
                                                                             sim_q2);
        const ExpectedLegPose expected = computeExpectedLegPose(leg_geo, mech);

        const ::Vec3 actual_body_to_coxa =
            bodyLocalSimToServer(Rotate(Conjugate(chassis.orientation),
                                        worldAnchorOnBodyA(world, sim_leg.bodyToCoxaJoint) - chassis.position));
        const ::Vec3 actual_coxa_to_femur =
            bodyLocalSimToServer(Rotate(Conjugate(chassis.orientation),
                                        worldAnchorOnBodyA(world, sim_leg.coxaToFemurJoint) - chassis.position));
        const ::Vec3 actual_femur_to_tibia =
            bodyLocalSimToServer(Rotate(Conjugate(chassis.orientation),
                                        worldAnchorOnBodyA(world, sim_leg.femurToTibiaJoint) - chassis.position));
        const ::Vec3 actual_foot_contact =
            bodyLocalSimToServer(Rotate(Conjugate(chassis.orientation),
                                        footContactPointWorld(world.GetBody(sim_leg.tibia)) - chassis.position));

        const ::Vec3 err_body_to_coxa = actual_body_to_coxa - expected.coxa_anchor_body;
        const ::Vec3 err_coxa_to_femur = actual_coxa_to_femur - expected.coxa_femur_joint_body;
        const ::Vec3 err_femur_to_tibia = actual_femur_to_tibia - expected.femur_tibia_joint_body;
        const ::Vec3 err_foot = actual_foot_contact - expected.foot_center_body;

        max_anchor_error_m = std::max(max_anchor_error_m,
                                      std::max({maxAbsComponent(err_body_to_coxa),
                                                maxAbsComponent(err_coxa_to_femur),
                                                maxAbsComponent(err_femur_to_tibia)}));
        max_foot_error_m = std::max(max_foot_error_m, maxAbsComponent(err_foot));

        constexpr double kAnchorEps = 1.0e-4;
        constexpr double kFootEps = 1.0e-4;
        if (!expect(maxAbsComponent(err_body_to_coxa) <= kAnchorEps,
                    "body-to-coxa joint anchor should match the server geometry") ||
            !expect(maxAbsComponent(err_coxa_to_femur) <= kAnchorEps,
                    "coxa-to-femur joint anchor should match the server geometry") ||
            !expect(maxAbsComponent(err_femur_to_tibia) <= kAnchorEps,
                    "femur-to-tibia joint anchor should match the server geometry") ||
            !expect(maxAbsComponent(err_foot) <= kFootEps,
                    "foot contact point should match the server geometry")) {
            std::cerr << "leg=" << leg
                      << " sim_wire=(" << sim_q0 << "," << sim_q1 << "," << sim_q2 << ")"
                      << " body_to_coxa_err=(" << err_body_to_coxa.x << "," << err_body_to_coxa.y << ","
                      << err_body_to_coxa.z << ")"
                      << " coxa_to_femur_err=(" << err_coxa_to_femur.x << "," << err_coxa_to_femur.y << ","
                      << err_coxa_to_femur.z << ")"
                      << " femur_to_tibia_err=(" << err_femur_to_tibia.x << "," << err_femur_to_tibia.y << ","
                      << err_femur_to_tibia.z << ")"
                      << " foot_err=(" << err_foot.x << "," << err_foot.y << "," << err_foot.z << ")\n";
            ok = false;
        }
    }

    if (!ok) {
        std::cout << "test_hexapod_server_alignment_zero_g assessment detected a frame mismatch; see per-leg errors above\n";
    }
    std::cout << "test_hexapod_server_alignment_zero_g ok max_anchor_error_m=" << max_anchor_error_m
              << " max_foot_error_m=" << max_foot_error_m << '\n';
    return 0;
}
