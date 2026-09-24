#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "minphys3d/core/world.hpp"
#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/math/vec3.hpp"

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

enum class ProximalSpeedLimitFrame : std::uint8_t {
    None = 0,
    Chassis = 1,
    Coxa = 2,
    Femur = 3,
    Tibia = 4,
};

enum class ProximalSpeedLimitSupport : std::uint8_t {
    Unknown = 0,
    Swing = 1,
    Stance = 2,
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
    /// Explicit protocol SolverMode=2 path. Also enabled by
    /// HEXAPOD_PINOCCHIO_COMPLIANT_CONTACT_EXPERIMENT. Healthy Mode 1 steps
    /// never switch to this law after a rigid reject. Last-resort NCP recovery
    /// may run the same cone QP once, logged, and only if residual/impulse/
    /// speed guards pass; see ncpCcpRecovery.
    bool compliantContact = false;
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
    double compliantProjectedResidual = 0.0;
    /// True when this sample applied last-resort cone-QP recovery after a
    /// rigid NCP miss. Status is still RecoveredRetry; the write is a real
    /// integrated state, not HeldLastGood.
    bool ncpCcpRecovery = false;
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
    std::array<double, 6> legNormalImpulse{};
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
    double chassisPreIntegrationAngularSpeed = 0.0;
    double maxLinkPreIntegrationAngularSpeed = 0.0;
    double peakPdAbsError = 0.0;
    ProximalSpeedLimitFrame speedLimitFrame = ProximalSpeedLimitFrame::None;
    // Internal attribution; no wire-protocol field added.
    int speedLimitLegIndex = -1;
    ProximalSpeedLimitSupport speedLimitSupport = ProximalSpeedLimitSupport::Unknown;
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
    std::uint64_t worstComplementarityContactId = 0;
    std::size_t uniqueRobotJointCount = 0;
    std::size_t tracedContactCount = 0;
    std::array<std::uint64_t, 8> tracedContactId{};
    std::array<std::uint8_t, 8> tracedContactLeg{};
    std::array<int, 8> tracedContactJoint{};
    std::array<double, 8> tracedContactComp{};
    std::array<double, 8> tracedContactDual{};
    std::array<double, 8> tracedContactCone{};
    std::array<double, 8> tracedContactPen{};
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

    /// Command packet interval (seconds). Reduced-contact CRBA is applied at
    /// the production ~5 ms bus; off-rate frozen replay keeps spawn inertias.
    void setCommandInterval(double seconds);

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

    /// Check the actual proposed pose/tangent velocity against every link's
    /// WORLD_ALIGNED speed bounds. Does not write world or rollback state.
    bool validateDynamicState(
        const std::vector<double>& q, const std::vector<double>& v,
        const ProximalSolverSettings& settings, ProximalStepDiagnostics& diagnostics);

    /// Validate before writing any body. Used for integrated candidates; raw
    /// writeState remains available for model mapping and fault-injection tests.
    bool writeValidatedState(
        World& world, const std::vector<double>& q, const std::vector<double>& v,
        const ProximalSolverSettings& settings, ProximalStepDiagnostics& diagnostics);

    /// Accept a safe externally corrected state as the rollback baseline.
    /// Rejected corrections restore the previous baseline, never replace it.
    bool synchronizeAfterExternalCorrection(
        World& world, const ProximalSolverSettings& settings = {});

    /// Test-only file IPC: dump/restore diagnostic cutpoints when request files
    /// appear. Polled at the start of each proximal step. Never overwrites an
    /// existing dump path. Restore JSON is never unlinked.
    void maybePollCutpointFileIpc(World& world);

    void resetWarmStarts();

    struct MechanicalEnergyBreakdown {
        double generalizedKinetic = 0.0;
        double bodyKinetic = 0.0;
        double armatureKinetic = 0.0;
        double potential = 0.0;
    };

    struct WarmStartAudit {
        std::size_t count = 0;
        std::size_t framedCount = 0;
        double impulseNorm = 0.0;
    };

    /// 0.5 vᵀMv (WORLD CRBA, including armature) versus rigid-body KE plus
    /// 0.5 armature ⊙ ω². Host tests and energy-ledger audits.
    bool computeMechanicalEnergyBreakdown(
        const World& world,
        const std::vector<double>& q,
        const std::vector<double>& v,
        MechanicalEnergyBreakdown& out) const;

    /// Reconstruct LOCAL_WORLD_ALIGNED twist of a bound robot body from q/v.
    bool computeLinkWorldTwist(
        const std::vector<double>& q,
        const std::vector<double>& v,
        std::uint32_t bodyId,
        Vec3& linearWorld,
        Vec3& angularWorld) const;

    /// LOCAL_WORLD_ALIGNED angular Jacobian (3 × nv), column-major.
    bool computeLinkAngularJacobian(
        const std::vector<double>& q,
        std::uint32_t bodyId,
        std::vector<double>& columnMajor3xNv) const;

    struct ImplicitDampingOracleResult {
        std::vector<double> vFree{};
        std::vector<double> explicitVfree{};
        std::vector<double> tauChosen{};
        std::vector<double> availableAtVin{};
        std::vector<double> availableAtVfree{};
        std::vector<double> positionTorque{};
        std::vector<double> dampingDiag{};
        std::vector<double> mass{};
        std::vector<double> bias{};
        bool unsaturated = true;
        bool envelopeHoldsAtVin = true;
        bool envelopeHoldsAtVfree = true;
        bool unlimitedEnvelopeHolds = true;
        bool usedUnlimitedDampingBrake = false;
        double actuatorWork = 0.0;
        double ldltRelativeError = 0.0;
        double maxDelassusAbsDiff = 0.0;
        double explicitAbaRelativeError = 0.0;
    };

    /// Test-local dense H = M + h D oracle (WORLD CRBA + armature). Does not
    /// replace production ABA. Optional Jacobian is column-major (rows × nv).
    bool computeImplicitDampingOracle(
        const std::vector<double>& q,
        const std::vector<double>& v,
        const std::array<double, 18>& servoErrors,
        const std::array<double, 18>& effectiveInertias,
        double dt,
        ImplicitDampingOracleResult& out,
        double gainScale = 1.0,
        const std::vector<double>* constraintJacobian = nullptr,
        std::size_t jacobianRows = 0) const;

    WarmStartAudit debugWarmStartAudit() const;
    void debugCaptureWarmStarts();
    void debugRestoreCapturedWarmStarts();
    /// Rotate stored contact tangent bases around n and transport λ into the
    /// new frame. Host tests use this to prove apply-time world transport.
    void debugRotateWarmStartTangentBasis(double radians);

    /// Test-only dump of physical q/v, warm starts, load-bearing mask,
    /// effective inertias and reduced-support blend.
    bool dumpDiagnosticCutpoint(const char* path) const;

    /// Test-only load of schema-1 `stand_cutpoint`. Frozen files that omit the
    /// warm-start `frame` 3×3 restore with `have_frame=false`.
    bool restoreDiagnosticCutpoint(World& world, const char* path);

    /// Write the 18 wire `targetAngle` values without advancing time.
    bool applyServoTargets(World& world, const std::array<double, 18>& targets);

    struct DiagnosticHistoryContact {
        std::uint64_t id = 0;
        std::array<double, 3> normal{};
        std::array<double, 3> impulse{};
    };

    /// Fields already present on a prefailure `accepted_history` sample.
    struct DiagnosticHistorySample {
        std::vector<double> q{};
        std::vector<double> v{};
        std::array<double, 18> targets{};
        std::array<double, 18> effectiveInertias{};
        double subDt = 0.0;
        double commandDt = 0.0;
        std::uint8_t loadBearingMask = 0;
        double reducedSupportBlend = 0.0;
        std::vector<DiagnosticHistoryContact> contacts{};
    };

    /// Test-only restore of one accepted-history sample. Warm-start velocity is
    /// not in the fixture (zero). Frame is reconstructed from the dumped normal.
    bool restoreAcceptedHistorySample(World& world, const DiagnosticHistorySample& sample);

    /// Read-only test audit of the last accepted substep (not a wire field).
    /// Errors/torques are evaluated before integration; inertias and gain are
    /// those actually used, including any retry/load scaling.
    struct ServoBalanceSample {
        std::array<double, 18> errors{}, appliedTorques{}, effectiveInertias{};
        double gainScale = 1.0;
        double dampingGainScale = 1.0;
        double subDt = 0.0;
    };
    bool debugLastServoBalance(ServoBalanceSample& out) const;
    /// Nominal gains at the accepted state, excluding temporary retry scaling.
    /// Zero before a successful step. No predicted contact state or gravity oracle.
    std::array<double, 18> servoStiffnessNmPerRad() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace minphys3d::demo
