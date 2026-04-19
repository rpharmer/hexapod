#include "physics_sim_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstring>
#include <iostream>

int main() {
    physics_sim::StateCorrection in{};
    in.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection);
    in.sequence_id = 17;
    in.timestamp_us = 123456789u;
    in.flags = physics_sim::kStateCorrectionPoseValid | physics_sim::kStateCorrectionContactValid;
    in.correction_strength = 0.75f;
    in.body_position = {1.0f, 2.0f, 3.0f};
    in.body_orientation = {0.9239f, 0.0f, 0.3827f, 0.0f};
    in.body_linear_velocity = {0.1f, 0.2f, 0.3f};
    in.body_angular_velocity = {0.4f, 0.5f, 0.6f};
    in.foot_contact_phase[0] = static_cast<std::uint8_t>(physics_sim::ContactPhase::ConfirmedStance);
    in.foot_contact_confidence[0] = 1.0f;
    in.foot_ground_height_m[0] = 1.75f;
    in.foot_ground_confidence[0] = 0.9f;
    in.terrain_normal = {0.0f, 1.0f, 0.0f};
    in.terrain_height_m = 1.75f;

    std::array<std::byte, physics_sim::kStateCorrectionBytes> bytes{};
    std::memcpy(bytes.data(), &in, sizeof(in));

    physics_sim::StateCorrection out{};
    if (!physics_sim::tryDecodeStateCorrection(bytes.data(), bytes.size(), out)) {
        std::cerr << "decode failed\n";
        return 1;
    }
    if (out.sequence_id != in.sequence_id || out.timestamp_us != in.timestamp_us) {
        std::cerr << "header mismatch\n";
        return 2;
    }
    if (out.flags != in.flags || out.correction_strength != in.correction_strength) {
        std::cerr << "scalar fields mismatch\n";
        return 3;
    }
    if (out.body_position != in.body_position || out.body_orientation != in.body_orientation) {
        std::cerr << "pose fields mismatch\n";
        return 4;
    }
    if (out.foot_contact_phase[0] != in.foot_contact_phase[0] ||
        out.foot_contact_confidence[0] != in.foot_contact_confidence[0] ||
        out.foot_ground_height_m[0] != in.foot_ground_height_m[0] ||
        out.foot_ground_confidence[0] != in.foot_ground_confidence[0]) {
        std::cerr << "contact fields mismatch\n";
        return 5;
    }
    if (out.terrain_height_m != in.terrain_height_m || out.terrain_normal != in.terrain_normal) {
        std::cerr << "terrain fields mismatch\n";
        return 6;
    }

    std::cout << "test_state_correction_protocol ok bytes=" << physics_sim::kStateCorrectionBytes << "\n";
    return 0;
}
