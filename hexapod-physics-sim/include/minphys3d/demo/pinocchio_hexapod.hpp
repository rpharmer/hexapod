#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"

namespace minphys3d::demo {

/// Whole-tree floating-base model mirroring the built-in minphys3d hexapod.
/// Pinocchio types are hidden so legacy-only builds do not inherit that dependency.
class PinocchioHexapodModel {
public:
    PinocchioHexapodModel(
        const World& world,
        const HexapodSceneObjects& scene,
        const std::array<std::uint32_t, 18>& servo_joint_ids);
    ~PinocchioHexapodModel();

    PinocchioHexapodModel(PinocchioHexapodModel&&) noexcept;
    PinocchioHexapodModel& operator=(PinocchioHexapodModel&&) noexcept;
    PinocchioHexapodModel(const PinocchioHexapodModel&) = delete;
    PinocchioHexapodModel& operator=(const PinocchioHexapodModel&) = delete;

    std::size_t configurationSize() const;
    std::size_t velocitySize() const;
    std::size_t jointCount() const;

    bool readState(const World& world, std::vector<double>& q, std::vector<double>& v) const;
    bool writeState(World& world, const std::vector<double>& q, const std::vector<double>& v);
    bool computeFreeAcceleration(
        const std::vector<double>& q,
        const std::vector<double>& v,
        const std::vector<double>& tau,
        std::vector<double>& ddq);
    bool computeDenseAcceleration(
        const std::vector<double>& q,
        const std::vector<double>& v,
        const std::vector<double>& tau,
        std::vector<double>& ddq);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace minphys3d::demo
