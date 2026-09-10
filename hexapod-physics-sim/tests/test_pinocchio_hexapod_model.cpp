#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
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
    if (!model.validateDelassusOracle(world, 1.0e-6, delassusError)) {
        std::cerr << "rigid Delassus/Cholesky oracle mismatch error=" << delassusError << "\n";
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
