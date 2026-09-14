#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

bool expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

Vec3 FootSphereCenterWorld(const Body& tibia) {
    for (const CompoundChild& child : tibia.compoundChildren) {
        if (child.shape == ShapeType::Sphere) {
            return tibia.position + Rotate(tibia.orientation, child.localPosition);
        }
    }
    return tibia.position;
}

} // namespace

int main() {
    World world({0.0, -9.81, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);

    bool ok = true;

    ok = expect(std::abs(kCoxaLength - 0.043) < 1.0e-6, "coxa length should match the server config") && ok;
    ok = expect(std::abs(kFemurLength - 0.060) < 1.0e-6, "femur length should match the server config") && ok;
    ok = expect(std::abs(kTibiaLength - physics_sim::kHexapodTibiaLinkLengthM) < 1.0e-6,
                "rigid tibia link should be shortened to make room for the foot sphere") &&
         ok;
    ok = expect(std::abs(physics_sim::kHexapodFootRadiusM - 0.018) < 1.0e-6,
                "foot radius should match the built-in physics-sim geometry") &&
         ok;
    ok = expect(std::abs((kTibiaLength + physics_sim::kHexapodFootRadiusM) - 0.104) < 1.0e-6,
                "rigid tibia plus foot radius should preserve the server tibia length") &&
         ok;

    ok = expect(std::abs(ComputeStandingBodyHeight() - world.GetBody(scene.body).position.y) < 1.0e-6,
                "standing body height should match the spawned chassis height") &&
         ok;

    for (std::size_t leg_index = 0; leg_index < scene.legs.size(); ++leg_index) {
        const Body& tibia = world.GetBody(scene.legs[leg_index].tibia);
        const Vec3 foot_center = FootSphereCenterWorld(tibia);
        const Vec3 shaft_tip =
            tibia.position + Rotate(tibia.orientation, Vec3{kTibiaRenderLength * 0.5, 0.0, 0.0});
        const Real centre_to_shaft_tip = Length(foot_center - shaft_tip);
        if (!expect(std::abs(centre_to_shaft_tip - physics_sim::kHexapodFootRadiusM) < 1.0e-5,
                    "foot sphere center should sit one radius beyond the rigid tibia shaft")) {
            std::cerr << "leg=" << leg_index << " centre_to_shaft_tip=" << centre_to_shaft_tip << '\n';
            ok = false;
        }
    }

    return ok ? 0 : 1;
}
