#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

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

minphys3d::Vec3 footCentreWorld(const Body& tibia) {
    for (const CompoundChild& child : tibia.compoundChildren) {
        if (child.shape == ShapeType::Sphere) {
            // Server FK's 104 mm tibia reaches the sphere CENTRE. The rigid
            // shaft is shortened by the 18 mm radius; adding that radius again
            // compares an axial surface point with a kinematic centre.
            return tibia.position + Rotate(tibia.orientation, child.localPosition);
        }
    }
    return tibia.position;
}

double maxAbsComponent(const ::Vec3& v) {
    return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)});
}

const std::array<::Vec3, 6> kExpectedFootCentrePositionsBody{{
    {0.152965, -0.115852, -0.202887},
    {-0.152965, -0.115852, -0.202887},
    {0.230989, -0.115852, 0.0},
    {-0.230989, -0.115852, 0.0},
    {0.152965, -0.115852, 0.202887},
    {-0.152965, -0.115852, 0.202887},
}};

}  // namespace

int main() {
    World world({0.0, 0.0, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    const Body& chassis = world.GetBody(scene.body);
    bool ok = true;
    double max_foot_error_m = 0.0;

    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        const LegLinkIds& sim_leg = scene.legs[leg];
        const ::Vec3 actual_foot_contact =
            Rotate(Conjugate(chassis.orientation),
                   footCentreWorld(world.GetBody(sim_leg.tibia)) - chassis.position);
        const ::Vec3 expected_foot_contact = kExpectedFootCentrePositionsBody[leg];
        const ::Vec3 err_foot = actual_foot_contact - expected_foot_contact;

        constexpr double kFootEps = 1.0e-4;
        max_foot_error_m = std::max(max_foot_error_m, maxAbsComponent(err_foot));

        if (!expect(maxAbsComponent(err_foot) <= kFootEps, "initial foot sphere centre should match server FK layout")) {
            std::cerr << "leg=" << leg
                      << " actual_foot=(" << actual_foot_contact.x << "," << actual_foot_contact.y << ","
                      << actual_foot_contact.z << ")"
                      << " expected_foot=(" << expected_foot_contact.x << "," << expected_foot_contact.y << ","
                      << expected_foot_contact.z << ")"
                      << " err=(" << err_foot.x << "," << err_foot.y << "," << err_foot.z << ")\n";
            ok = false;
        }
    }

    if (!ok) {
        std::cout << "test_hexapod_initial_layout_matches_server detected a layout mismatch; see per-leg errors above\n";
    }
    std::cout << "test_hexapod_initial_layout_matches_server ok max_foot_error_m=" << max_foot_error_m << '\n';
    return ok ? 0 : 1;
}
