#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

    ProximalSolverSettings proximalSettings{};
    proximalSettings.maxIterations = 50;
    ProximalStepDiagnostics proximalDiagnostics{};
    for (int step = 0; step < 8; ++step) {
        if (!model.stepProximal(world, 1.0 / 240.0, proximalSettings, proximalDiagnostics)) {
            std::cerr << "proximal standing step failed status="
                      << static_cast<int>(proximalDiagnostics.status)
                      << " residual=" << proximalDiagnostics.primalResidual
                      << " dual=" << proximalDiagnostics.dualResidual
                      << " comp=" << proximalDiagnostics.complementarityResidual
                      << " pre_v=" << proximalDiagnostics.preIntegrationLinearSpeed
                      << " pre_w=" << proximalDiagnostics.preIntegrationAngularSpeed
                      << " retries=" << proximalDiagnostics.retries << "\n";
            return 1;
        }
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
