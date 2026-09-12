#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace {

using namespace minphys3d;
using namespace minphys3d::demo;

std::array<std::uint32_t, 18> SceneJointOrder(const HexapodSceneObjects& scene) {
    std::array<std::uint32_t, 18> ids{};
    std::size_t out = 0;
    for (const LegLinkIds& leg : scene.legs) {
        ids[out++] = leg.bodyToCoxaJoint;
        ids[out++] = leg.coxaToFemurJoint;
        ids[out++] = leg.femurToTibiaJoint;
    }
    return ids;
}

bool CheckSelectedFootSupport(
    const std::array<bool, 6>& enabledLegs, const char* label) {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    for (const std::uint32_t bodyId : scene.body_ids) {
        if (bodyId != scene.plane) {
            world.GetBody(bodyId).collisionMask = 0;
        }
    }
    std::size_t enabledCount = 0;
    for (std::size_t leg = 0; leg < enabledLegs.size(); ++leg) {
        if (enabledLegs[leg]) {
            world.GetBody(scene.legs[leg].tibia).collisionMask = 0xFFFFFFFFU;
            ++enabledCount;
        }
    }

    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    int consecutiveExpectedSupport = 0;
    for (int step = 0; step < 240; ++step) {
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
            std::cerr << label << " support solve failed step=" << step
                      << " reason=" << static_cast<int>(diagnostics.failureReason)
                      << " constraints=" << diagnostics.contactConstraintCount
                      << " iterations=" << diagnostics.iterations << "\n";
            return false;
        }
        if (diagnostics.contactConstraintCount > enabledCount) {
            std::cerr << label << " accepted contacts from a masked body constraints="
                      << diagnostics.contactConstraintCount
                      << " enabled=" << enabledCount << "\n";
            return false;
        }
        if (diagnostics.contactConstraintCount == enabledCount
            && diagnostics.peakNormalImpulse > 0.0
            && diagnostics.coneResidual <= 1.0e-8
            && std::isfinite(diagnostics.ncpDualResidual)
            && std::isfinite(diagnostics.ncpComplementarityResidual)) {
            ++consecutiveExpectedSupport;
            if (consecutiveExpectedSupport >= 3) {
                return true;
            }
        } else {
            consecutiveExpectedSupport = 0;
        }
    }
    std::cerr << label << " never established three consecutive selected-contact solves expected="
              << enabledCount << "\n";
    return false;
}

bool CheckExternalCorrectionSynchronization() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));

    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        std::cerr << "failed to read external-correction test state\n";
        return false;
    }
    const double correctedX = q[0] + 0.007;
    q[0] = correctedX;
    if (!model.writeState(world, q, v)
        || !model.synchronizeAfterExternalCorrection(world)) {
        std::cerr << "failed to synchronize externally corrected state\n";
        return false;
    }

    world.GetBody(scene.body).velocity.x = std::numeric_limits<double>::quiet_NaN();
    ProximalSolverSettings settings{};
    ProximalStepDiagnostics diagnostics{};
    if (model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)
        || diagnostics.status != ProximalStepStatus::HeldLastGood
        || !model.readState(world, q, v)
        || std::abs(q[0] - correctedX) > 1.0e-9) {
        std::cerr << "rollback did not preserve externally corrected pose\n";
        return false;
    }
    return true;
}

int Run() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    const auto joints = SceneJointOrder(scene);
    PinocchioHexapodModel model(world, scene, joints);
    if (model.configurationSize() != 25 || model.velocitySize() != 24 || model.jointCount() != 18) {
        std::cerr << "unexpected whole-body dimensions nq=" << model.configurationSize()
                  << " nv=" << model.velocitySize() << " joints=" << model.jointCount() << "\n";
        return 1;
    }

    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        std::cerr << "failed to read initial state\n";
        return 1;
    }
    const std::vector<double> qInitial = q;
    const std::vector<double> vInitial = v;
    for (std::size_t i = 7; i < qInitial.size(); ++i) {
        if (std::abs(qInitial[i]) > 1.0e-12) {
            std::cerr << "initial articulated coordinate is not zero-relative index="
                      << i << " value=" << qInitial[i] << "\n";
            return 1;
        }
    }
    if (!model.writeState(world, q, v) || !model.readState(world, q, v)) {
        std::cerr << "initial state round trip failed\n";
        return 1;
    }
    double maxRoundTrip = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
        maxRoundTrip = std::max(maxRoundTrip, std::abs(q[i] - qInitial[i]));
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        maxRoundTrip = std::max(maxRoundTrip, std::abs(v[i] - vInitial[i]));
    }
    if (maxRoundTrip > 1.0e-9) {
        std::cerr << "initial round-trip error=" << maxRoundTrip << "\n";
        return 1;
    }

    double delassusError = 0.0;
    if (!model.validateDelassusOracle(world, 1.0e-8, delassusError)) {
        std::cerr << "rigid Delassus/Cholesky oracle mismatch error=" << delassusError << "\n";
        return 1;
    }

    if (!CheckExternalCorrectionSynchronization()) {
        return 1;
    }

    if (!CheckSelectedFootSupport(
            {true, false, false, false, false, false}, "single-foot")
        || !CheckSelectedFootSupport(
            {true, false, true, false, true, false}, "tripod")
        || !CheckSelectedFootSupport(
            {true, true, true, true, true, true}, "all-six")) {
        return 1;
    }

    ProximalSolverSettings proximalSettings{};
    proximalSettings.maxIterations = 50;
    const bool traceContacts = std::getenv("PINOCCHIO_TRACE_CONTACTS") != nullptr;
    ProximalStepDiagnostics proximalDiagnostics{};
    int standingSteps = 600;
    if (const char* value = std::getenv("PINOCCHIO_STANDING_STEPS")) {
        standingSteps = std::max(1, std::atoi(value));
    }
    std::uint64_t previousSignature = 0;
    bool observedFlatGroundSupport = false;
    for (int step = 0; step < standingSteps; ++step) {
        if (!model.stepProximal(world, 1.0 / 240.0, proximalSettings, proximalDiagnostics)) {
            std::cerr << "proximal standing step failed status="
                      << static_cast<int>(proximalDiagnostics.status)
                      << " step=" << step
                      << " residual=" << proximalDiagnostics.primalResidual
                      << " dual=" << proximalDiagnostics.dualResidual
                      << " comp=" << proximalDiagnostics.complementarityResidual
                      << " iterations=" << proximalDiagnostics.iterations
                      << " manifolds=" << proximalDiagnostics.contactManifoldCount
                      << " constraints=" << proximalDiagnostics.contactConstraintCount
                      << " duplicates=" << proximalDiagnostics.duplicateContactCount
                      << " cond=" << proximalDiagnostics.delassusConditionEstimate
                      << " signature=" << proximalDiagnostics.contactSetSignature
                      << " pre_v=" << proximalDiagnostics.preIntegrationLinearSpeed
                      << " pre_w=" << proximalDiagnostics.preIntegrationAngularSpeed
                      << " retries=" << proximalDiagnostics.retries << "\n";
            return 1;
        }
        if (proximalDiagnostics.contactConstraintCount > 0) {
            observedFlatGroundSupport = true;
            if (proximalDiagnostics.contactConstraintCount > scene.legs.size()
                || proximalDiagnostics.externalManifoldCount > scene.legs.size()
                || proximalDiagnostics.robotRobotManifoldCount != 0) {
                std::cerr << "flat-ground contact arbitration failed constraints="
                          << proximalDiagnostics.contactConstraintCount
                          << " external_manifolds="
                          << proximalDiagnostics.externalManifoldCount
                          << " robot_manifolds="
                          << proximalDiagnostics.robotRobotManifoldCount << "\n";
                return 1;
            }
        }
        if (proximalDiagnostics.iterations > proximalSettings.maxIterations) {
            std::cerr << "configured ADMM iteration cap exceeded iterations="
                      << proximalDiagnostics.iterations
                      << " cap=" << proximalSettings.maxIterations << "\n";
            return 1;
        }
        if (traceContacts && proximalDiagnostics.contactSetSignature != previousSignature) {
            std::vector<double> stateQ;
            std::vector<double> stateV;
            model.readState(world, stateQ, stateV);
            std::cerr << "contact set step=" << step
                      << " body_y=" << (stateQ.size() > 1 ? stateQ[1] : 0.0)
                      << " manifolds=" << proximalDiagnostics.contactManifoldCount
                      << " constraints=" << proximalDiagnostics.contactConstraintCount
                      << " signature=" << proximalDiagnostics.contactSetSignature
                      << " cond=" << proximalDiagnostics.delassusConditionEstimate << "\n";
            previousSignature = proximalDiagnostics.contactSetSignature;
        }
    }
    if (!observedFlatGroundSupport) {
        std::cerr << "standing run never established flat-ground support\n";
        return 1;
    }

    ProximalStepDiagnostics coarseDtDiagnostics{};
    ProximalStepDiagnostics fineDtDiagnostics{};
    if (!model.stepProximal(world, 1.0 / 120.0, proximalSettings, coarseDtDiagnostics)
        || !model.stepProximal(world, 1.0 / 480.0, proximalSettings, fineDtDiagnostics)) {
        std::cerr << "timestep-change warm start did not remain usable\n";
        return 1;
    }

    ProximalStepDiagnostics invalidDtDiagnostics{};
    if (model.stepProximal(
            world,
            std::numeric_limits<double>::quiet_NaN(),
            proximalSettings,
            invalidDtDiagnostics)
        || invalidDtDiagnostics.status != ProximalStepStatus::HeldLastGood
        || invalidDtDiagnostics.failureReason != ProximalFailureReason::InvalidDt) {
        std::cerr << "invalid timestep was not rejected before integration\n";
        return 1;
    }

    world.GetBody(scene.body).velocity.x = std::numeric_limits<double>::quiet_NaN();
    ProximalStepDiagnostics nonFiniteDiagnostics{};
    if (model.stepProximal(world, 1.0 / 240.0, proximalSettings, nonFiniteDiagnostics)
        || nonFiniteDiagnostics.status != ProximalStepStatus::HeldLastGood
        || nonFiniteDiagnostics.failureReason != ProximalFailureReason::ReadState
        || !model.readState(world, q, v)) {
        std::cerr << "non-finite world state was not restored to the last valid pose\n";
        return 1;
    }

    q[1] += 1.0;
    v.assign(model.velocitySize(), 0.0);
    v[0] = 100.0;
    if (!model.writeState(world, q, v)) {
        std::cerr << "failed to inject excessive pre-integration speed\n";
        return 1;
    }
    ProximalStepDiagnostics speedDiagnostics{};
    if (model.stepProximal(world, 1.0 / 240.0, proximalSettings, speedDiagnostics)
        || speedDiagnostics.status != ProximalStepStatus::HeldLastGood
        || speedDiagnostics.failureReason != ProximalFailureReason::SpeedLimit
        || !model.readState(world, q, v)) {
        std::cerr << "excessive speed did not rollback to a readable last-valid state reason="
                  << static_cast<int>(speedDiagnostics.failureReason) << "\n";
        return 1;
    }
    const std::uint64_t retriesBeforeExtremePenetration = speedDiagnostics.retries;

    const std::vector<double> beforeExtremeQ = q;
    std::vector<double> extremeQ = q;
    std::vector<double> extremeV(model.velocitySize(), 0.0);
    extremeQ[1] -= 0.25;
    if (!model.writeState(world, extremeQ, extremeV)) {
        std::cerr << "failed to inject extreme contact penetration\n";
        return 1;
    }
    ProximalStepDiagnostics penetrationDiagnostics{};
    if (model.stepProximal(world, 1.0 / 240.0, proximalSettings, penetrationDiagnostics)
        || penetrationDiagnostics.status != ProximalStepStatus::HeldLastGood
        || penetrationDiagnostics.failureReason != ProximalFailureReason::ExtremePenetration
        || penetrationDiagnostics.maxContactPenetration <= 0.05
        || penetrationDiagnostics.retries != retriesBeforeExtremePenetration
        || !model.readState(world, q, v)) {
        std::cerr << "extreme penetration was not rejected before integration reason="
                  << static_cast<int>(penetrationDiagnostics.failureReason)
                  << " penetration=" << penetrationDiagnostics.maxContactPenetration
                  << " retries_before=" << retriesBeforeExtremePenetration
                  << " retries_after=" << penetrationDiagnostics.retries << "\n";
        return 1;
    }
    double penetrationRollbackError = 0.0;
    for (std::size_t i = 0; i < q.size(); ++i) {
        penetrationRollbackError =
            std::max(penetrationRollbackError, std::abs(q[i] - beforeExtremeQ[i]));
    }
    if (penetrationRollbackError > 2.0e-8) {
        std::cerr << "extreme penetration rollback changed last-good pose error="
                  << penetrationRollbackError << "\n";
        return 1;
    }

    std::mt19937 rng(0x50494e4fU);
    std::uniform_real_distribution<double> qDist(-0.45, 0.45);
    std::uniform_real_distribution<double> vDist(-0.2, 0.2);
    for (int sample = 0; sample < 1000; ++sample) {
        q = qInitial;
        v = vInitial;
        for (std::size_t i = 7; i < q.size(); ++i) q[i] = qDist(rng);
        for (double& value : v) value = vDist(rng);
        const std::vector<double> expectedQ = q;
        const std::vector<double> expectedV = v;
        if (!model.writeState(world, q, v) || !model.readState(world, q, v)) {
            std::cerr << "random round trip failed at sample=" << sample << "\n";
            return 1;
        }
        double maxError = 0.0;
        for (std::size_t i = 0; i < q.size(); ++i) {
            maxError = std::max(maxError, std::abs(q[i] - expectedQ[i]));
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
            maxError = std::max(maxError, std::abs(v[i] - expectedV[i]));
        }
        if (maxError > 2.0e-8) {
            std::cerr << "random round-trip error=" << maxError << " sample=" << sample << "\n";
            return 1;
        }

        std::vector<double> tau(model.velocitySize(), 0.0);
        for (std::size_t i = 6; i < tau.size(); ++i) tau[i] = 0.2 * qDist(rng);
        std::vector<double> aba;
        std::vector<double> dense;
        if (!model.computeFreeAcceleration(q, v, tau, aba)
            || !model.computeDenseAcceleration(q, v, tau, dense)) {
            std::cerr << "dynamics solve failed at sample=" << sample << "\n";
            return 1;
        }
        double maxAbs = 0.0;
        double maxDiff = 0.0;
        for (std::size_t i = 0; i < aba.size(); ++i) {
            maxAbs = std::max(maxAbs, std::abs(dense[i]));
            maxDiff = std::max(maxDiff, std::abs(aba[i] - dense[i]));
        }
        if (maxDiff > 1.0e-9 * std::max(1.0, maxAbs)) {
            std::cerr << "ABA/dense mismatch diff=" << maxDiff << " scale=" << maxAbs
                      << " sample=" << sample << "\n";
            return 1;
        }
    }
    return 0;
}

} // namespace

int main() {
    return Run();
}
