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
    ExtremePenetration,
};

struct ProximalSolverSettings {
    int maxIterations = 50;
    double proximalMu = 1.0e-6;
    // ADMM stop. Keep this tight: 1e-3 lets ADMM exit before stance impulses
    // can hold the body up (stand height error ~10 cm).
    double absoluteTolerance = 1.0e-8;
    // NCP accept floor, and the ADMM stop used on sliding contacts (tangential
    // free speed > 2 cm/s). Standing contacts keep `absoluteTolerance` (1e-8)
    // so the body does not sag. Do not raise the standing ADMM stop to 1e-3.
    double ncpAbsoluteTolerance = 1.0e-3;
    double relativeTolerance = 1.0e-6;
    double contactRegularization = 1.0e-10;
    double maxLinearSpeed = 2.0;
    double maxAngularSpeed = 10.0;
    double maxContactPenetration = 0.05;
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
    double ncpDualResidual = 0.0;
    double ncpComplementarityResidual = 0.0;
    double coneResidual = 0.0;
    double peakNormalImpulse = 0.0;
    double peakFrictionImpulse = 0.0;
    // Signed contact-space friction mapped through the contact frame into
    // sim-world X/Z (Y-up). Summed over contacts in this substep.
    double sumFrictionImpulseWorldX = 0.0;
    double sumFrictionImpulseWorldZ = 0.0;
    double sumAbsFrictionImpulseWorldX = 0.0;
    double sumAbsFrictionImpulseWorldZ = 0.0;
    double sumFrictionImpulseWorldY = 0.0;
    // Free-flyer linear velocity increment from M^-1 J^T λ, mapped from the
    // LOCAL v convention into sim-world X/Z.
    double contactDeltaVx = 0.0;
    double contactDeltaVz = 0.0;
    // Per-leg census (scene/server order). Friction is summed over contacts on
    // that tibia; drift/slip are the last constraint on that tibia this substep.
    std::array<double, 6> legFrictionImpulseWorldX{};
    std::array<double, 6> legFrictionImpulseWorldZ{};
    std::array<double, 6> legPinocchioDriftTx{};
    std::array<double, 6> legWorldSlipTx{};
    std::array<double, 6> legWorldSlipTy{};
    std::array<std::uint8_t, 6> legContactCount{};
    std::array<double, 6> legTibiaVx{};
    std::array<double, 6> legSpinVx{};
    std::array<double, 6> legT0x{};
    std::array<double, 6> legFootVx{};
    std::array<double, 6> legFootX{};
    std::array<double, 6> legFootPosVx{};
    std::array<double, 6> legFootVz{};
    std::array<double, 6> legFootZ{};
    std::array<double, 6> legFootPosVz{};
    double peakStructuralImpulse = 0.0;
    double peakActuatorImpulse = 0.0;
    double peakServoTorqueUtilization = 0.0;
    double preIntegrationLinearSpeed = 0.0;
    double preIntegrationAngularSpeed = 0.0;
    double maxContactPenetration = 0.0;
    double mechanicalEnergyDelta = 0.0;
    double actuatorWork = 0.0;
    double dynamicsTimeMs = 0.0;
    double contactSetupTimeMs = 0.0;
    double collisionTimeMs = 0.0;
    double constraintAssemblyTimeMs = 0.0;
    double delassusTimeMs = 0.0;
    double admmTimeMs = 0.0;
    double integrationTimeMs = 0.0;
    double totalStepTimeMs = 0.0;
    std::size_t contactManifoldCount = 0;
    std::size_t contactConstraintCount = 0;
    std::size_t robotRobotManifoldCount = 0;
    std::size_t externalManifoldCount = 0;
    std::size_t contactPointCount = 0;
    std::size_t duplicateContactCount = 0;
    double delassusConditionEstimate = 0.0;
    double delassusMinEigenvalue = 0.0;
    double delassusMaxEigenvalue = 0.0;
    double admmRho = 0.0;
    std::uint64_t contactSetSignature = 0;
    std::uint64_t warmStartResets = 0;
    std::uint64_t retries = 0;
    std::uint64_t rollbackCount = 0;
    std::uint64_t heldStateCount = 0;
    std::uint64_t unsupportedIslandCount = 0;
    std::uint64_t worstContactId = 0;
    bool admmConverged = false;
    bool ncpPhysicallyConverged = false;
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

    /// CRBA diagonals at the initial pose, with other joints instantaneously
    /// locked. Diagnostic only; production PD uses `servoNominalInertias()`.
    std::array<double, 18> servoUnconstrainedInertias() const;

    /// Stance-loaded reflected inertias used by PD. Other joints locked, six
    /// feet at the initial pose; clamped to `[M_ii, 1.5 M_ii]`.
    std::array<double, 18> servoNominalInertias() const;

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

    /// Accept an externally corrected minphys3d state as the new rollback
    /// baseline and discard solver history tied to the previous pose.
    bool synchronizeAfterExternalCorrection(const World& world);

    void resetWarmStarts();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace minphys3d::demo
