#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>

#include "demo/frame_sink.cpp"
#include "demo/scenes.cpp"
#include "minphys3d/demo/hexapod_stability.hpp"

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

std::optional<std::size_t> LegIndexForTibia(const HexapodSceneObjects& scene, std::uint32_t body_id) {
    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        if (scene.legs[leg].tibia == body_id) {
            return leg;
        }
    }
    return std::nullopt;
}

} // namespace

int main() {
    World world({0.0f, -9.81f, 0.0f});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    RelaxBuiltInHexapodServos(world, scene);
    ApplyHexapodPoseHoldStabilityTuning(world, scene);

    std::array<bool, 6> have_reference{};
    std::array<Vec3, 6> reference_positions{};
    std::array<float, 6> max_drift{};
    std::array<int, 6> settled_contact_frames{};

    constexpr float kFrameDt = 1.0f / 240.0f;
    for (int frame = 0; frame < 360; ++frame) {
        world.Step(kFrameDt, 24);
        for (const Manifold& manifold : world.DebugManifolds()) {
            const bool a_is_ground = manifold.a == scene.plane;
            const bool b_is_ground = manifold.b == scene.plane;
            if (a_is_ground == b_is_ground || manifold.contacts.empty()) {
                continue;
            }
            const std::uint32_t body_id = a_is_ground ? manifold.b : manifold.a;
            const std::optional<std::size_t> leg_index = LegIndexForTibia(scene, body_id);
            if (!leg_index.has_value()) {
                continue;
            }

            Vec3 point_sum{};
            for (const Contact& contact : manifold.contacts) {
                point_sum = point_sum + contact.point;
            }
            const Vec3 contact_point = point_sum * (1.0f / static_cast<float>(manifold.contacts.size()));

            if (frame >= 120) {
                if (!have_reference[*leg_index]) {
                    have_reference[*leg_index] = true;
                    reference_positions[*leg_index] = contact_point;
                }
                max_drift[*leg_index] =
                    std::max(max_drift[*leg_index], Length(contact_point - reference_positions[*leg_index]));
                ++settled_contact_frames[*leg_index];
            }
        }
    }

    for (std::size_t leg = 0; leg < scene.legs.size(); ++leg) {
        std::cout << "leg=" << leg
                  << " settled_contact_frames=" << settled_contact_frames[leg]
                  << " max_contact_point_drift_m=" << max_drift[leg]
                  << '\n';
    }

    return 0;
}
