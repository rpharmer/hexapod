#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"

namespace minphys3d::demo {

enum class ProximalStepStatus : std::uint8_t {
    Healthy = 0,
    RecoveredRetry = 1,
    HeldLastGood = 2,
    UnsupportedIsland = 3,
};

enum class ProximalFailureReason : std::uint8_t {
    None = 0,
    InvalidDt,
    ReadState,
    NonFiniteState,
    NonFiniteMass,
    NonFiniteAcceleration,
    UnsupportedIsland,
    SolverNotConverged,
    NonFiniteImpulse,
    NonFiniteVelocity,
    SpeedLimit,
    NonFiniteConfiguration,
    WriteState,
    NonFiniteEnergy,
};

struct ProximalSolverSettings {
    int maxIterations = 50;
    double proximalMu = 1.0e-6;
    double absoluteTolerance = 1.0e-8;
    double relativeTolerance = 1.0e-6;
    double contactRegularization = 1.0e-10;
    double maxLinearSpeed = 2.0;
    double maxAngularSpeed = 10.0;
    // Suppress restitution for low-speed settling impacts. This is a solver-level
    // threshold; material restitution values are still preserved per contact.
    double restitutionVelocityCutoff = 0.2;
};

struct ProximalStepDiagnostics {
    ProximalStepStatus status = ProximalStepStatus::Healthy;
    ProximalFailureReason failureReason = ProximalFailureReason::None;
    int iterations = 0;
    double primalResidual = 0.0;
    double dualResidual = 0.0;
    double complementarityResidual = 0.0;
    double peakNormalImpulse = 0.0;
    double peakFrictionImpulse = 0.0;
    double peakActuatorImpulse = 0.0;
    double peakServoTorqueUtilization = 0.0;
    double preIntegrationLinearSpeed = 0.0;
    double preIntegrationAngularSpeed = 0.0;
    double mechanicalEnergyDelta = 0.0;
    double actuatorWork = 0.0;
    std::size_t contactManifoldCount = 0;
    std::size_t contactConstraintCount = 0;
    std::size_t robotRobotManifoldCount = 0;
    std::size_t externalManifoldCount = 0;
    std::size_t contactPointCount = 0;
    std::size_t duplicateContactCount = 0;
    double delassusConditionEstimate = 0.0;
    double delassusMinEigenvalue = 0.0;
    double delassusMaxEigenvalue = 0.0;
    std::uint64_t contactSetSignature = 0;
    std::uint64_t warmStartResets = 0;
    std::uint64_t retries = 0;
    std::uint64_t rollbackCount = 0;
    std::uint64_t heldStateCount = 0;
    std::uint64_t unsupportedIslandCount = 0;
    std::uint64_t worstContactId = 0;
};

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

    /// Compare the production articulated Delassus operator with the dense
    /// constraint-Cholesky oracle for a pair of non-degenerate contacts.
    bool validateDelassusOracle(
        const World& world,
        double tolerance,
        double& maxRelativeError);

    /// Advance the complete floating-base tree using minphys3d manifolds and
    /// Pinocchio's articulated Delassus + proximal ADMM contact solver.
    bool stepProximal(
        World& world,
        double dt,
        const ProximalSolverSettings& settings,
        ProximalStepDiagnostics& diagnostics);

    void resetWarmStarts();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace minphys3d::demo
