#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "minphys3d/minphys3d.hpp"

namespace {

using minphys3d::Body;
using minphys3d::Contact;
using minphys3d::ContactSolverConfig;
using minphys3d::Length;
using minphys3d::Manifold;
using minphys3d::Quat;
using minphys3d::ShapeType;
using minphys3d::Vec3;
using minphys3d::World;

struct RunMetrics {
    float maxPenetration = 0.0f;
    float contactCountStdDev = 0.0f;
    float contactCountMeanStepDelta = 0.0f;
    float maxImpulseDelta = 0.0f;
    float meanImpulseDelta = 0.0f;
    float settleTimeSeconds = -1.0f;
    int settleStep = -1;
    std::uint64_t reorderEvents = 0;
    std::vector<float> stepMaxPenetration;
};

struct SceneConfig {
    std::string name;
    World world;
    std::vector<std::uint32_t> trackedDynamicBodies;
    float dt = 1.0f / 120.0f;
    int solverIterations = 16;
    int steps = 900;
    bool logStepContactCount = false;
    bool logStepManifoldIds = false;
};

struct ComparisonResult {
    std::string scene;
    RunMetrics scalar;
    RunMetrics block;
    bool pass = true;
    std::vector<std::string> failures;
};

struct RegressionThresholds {
    static constexpr float kMaxAbsolutePenetration = 6.5f;
    static constexpr float kMaxPenetrationRegression = 0.04f;
    static constexpr float kMaxContactStdDevRegression = 0.55f;
    static constexpr float kMaxContactStepDeltaRegression = 0.65f;
    static constexpr float kMaxImpulseDeltaRegression = 0.30f;
    static constexpr float kMaxMeanImpulseDeltaRegression = 0.16f;
    static constexpr float kMaxSettleTimeRegressionSeconds = 1.0f;
};

Body MakePlane() {
    Body plane;
    plane.shape = ShapeType::Plane;
    plane.planeNormal = {0.0f, 1.0f, 0.0f};
    plane.planeOffset = 0.0f;
    plane.staticFriction = 0.95f;
    plane.dynamicFriction = 0.70f;
    plane.restitution = 0.0f;
    return plane;
}

ContactSolverConfig MakeSolverConfig(bool useBlockSolver) {
    ContactSolverConfig cfg;
    cfg.useSplitImpulse = true;
    cfg.penetrationSlop = 0.005f;
    cfg.splitImpulseCorrectionFactor = 0.85f;
    cfg.penetrationBiasFactor = 0.05f;
    cfg.restitutionVelocityCutoff = 0.5f;
    cfg.staticFrictionSpeedThreshold = 0.0f;
    cfg.staticToDynamicTransitionSpeed = 0.2f;
    cfg.useBlockSolver = useBlockSolver;
    return cfg;
}

std::uint64_t MakePersistentPointKey(const Manifold& manifold, const Contact& contact, std::uint8_t ordinal) {
    const std::uint32_t lo = std::min(manifold.a, manifold.b);
    const std::uint32_t hi = std::max(manifold.a, manifold.b);
    const std::uint64_t pairKey = (static_cast<std::uint64_t>(lo) << 32) | hi;
    const std::uint64_t manifoldId = (pairKey << 8) ^ static_cast<std::uint64_t>(manifold.manifoldType);
    return (manifoldId * 1469598103934665603ull)
         ^ (contact.featureKey * 1099511628211ull)
         ^ static_cast<std::uint64_t>(ordinal);
}

RunMetrics RunScene(SceneConfig config, bool useBlockSolver) {
    config.world.SetContactSolverConfig(MakeSolverConfig(useBlockSolver));

    RunMetrics metrics;
    std::vector<float> contactCounts;
    std::vector<float> contactCountStepDeltas;
    std::unordered_map<std::uint64_t, float> lastImpulseByPoint;
    std::uint64_t impulseDeltaCount = 0;
    float impulseDeltaSum = 0.0f;

    float previousContactCount = -1.0f;
    int settledConsecutive = 0;

    for (int step = 0; step < config.steps; ++step) {
        config.world.Step(config.dt, config.solverIterations);

        const std::vector<Manifold>& manifolds = config.world.DebugManifolds();
        float totalContacts = 0.0f;
        float stepMaxPenetration = 0.0f;

        for (const Manifold& manifold : manifolds) {
            std::unordered_map<std::uint64_t, std::uint8_t> ordinalCount;
            for (const Contact& contact : manifold.contacts) {
                metrics.maxPenetration = std::max(metrics.maxPenetration, std::max(contact.penetration, 0.0f));
                stepMaxPenetration = std::max(stepMaxPenetration, std::max(contact.penetration, 0.0f));
                totalContacts += 1.0f;

                const std::uint8_t ordinal = ordinalCount[contact.featureKey]++;
                const std::uint64_t pointKey = MakePersistentPointKey(manifold, contact, ordinal);
                const float impulse = contact.normalImpulseSum;

                const auto it = lastImpulseByPoint.find(pointKey);
                if (it != lastImpulseByPoint.end()) {
                    const float delta = std::abs(impulse - it->second);
                    metrics.maxImpulseDelta = std::max(metrics.maxImpulseDelta, delta);
                    impulseDeltaSum += delta;
                    ++impulseDeltaCount;
                }
                lastImpulseByPoint[pointKey] = impulse;
            }
        }

        contactCounts.push_back(totalContacts);
        metrics.stepMaxPenetration.push_back(stepMaxPenetration);
        if (config.logStepContactCount) {
            std::cout << "[manifold-reorder-step] solver=" << (useBlockSolver ? "block" : "scalar")
                      << " step=" << step
                      << " contact_count=" << totalContacts << "\n";
        }
        if (config.logStepManifoldIds) {
            std::cout << "[manifold-reorder-step] solver=" << (useBlockSolver ? "block" : "scalar")
                      << " step=" << step
                      << " manifold_ids=";
            bool first = true;
            for (const Manifold& manifold : manifolds) {
                const std::uint32_t lo = std::min(manifold.a, manifold.b);
                const std::uint32_t hi = std::max(manifold.a, manifold.b);
                if (!first) {
                    std::cout << ",";
                }
                first = false;
                std::cout << lo << ":" << hi << ":" << static_cast<unsigned>(manifold.manifoldType);
            }
            if (first) {
                std::cout << "none";
            }
            std::cout << "\n";
        }
        if (previousContactCount >= 0.0f) {
            contactCountStepDeltas.push_back(std::abs(totalContacts - previousContactCount));
        }
        previousContactCount = totalContacts;

        bool allSleeping = !config.trackedDynamicBodies.empty();
        for (std::uint32_t id : config.trackedDynamicBodies) {
            const Body& body = config.world.GetBody(id);
            allSleeping = allSleeping && body.isSleeping;
        }
        if (allSleeping) {
            ++settledConsecutive;
            if (metrics.settleStep < 0 && settledConsecutive >= 30) {
                metrics.settleStep = step - 29;
                metrics.settleTimeSeconds = static_cast<float>(metrics.settleStep) * config.dt;
            }
        } else {
            settledConsecutive = 0;
        }
    }

#ifndef NDEBUG
    metrics.reorderEvents = config.world.GetSolverTelemetry().reorderDetected;
#endif

    const float count = static_cast<float>(contactCounts.size());
    if (count > 0.0f) {
        float mean = 0.0f;
        for (float v : contactCounts) {
            mean += v;
        }
        mean /= count;

        float variance = 0.0f;
        for (float v : contactCounts) {
            const float d = v - mean;
            variance += d * d;
        }
        variance /= count;
        metrics.contactCountStdDev = std::sqrt(std::max(variance, 0.0f));
    }

    if (!contactCountStepDeltas.empty()) {
        float deltaMean = 0.0f;
        for (float d : contactCountStepDeltas) {
            deltaMean += d;
        }
        metrics.contactCountMeanStepDelta = deltaMean / static_cast<float>(contactCountStepDeltas.size());
    }

    if (impulseDeltaCount > 0) {
        metrics.meanImpulseDelta = impulseDeltaSum / static_cast<float>(impulseDeltaCount);
    }

    return metrics;
}

ComparisonResult CompareScene(const SceneConfig& source) {
    ComparisonResult out;
    out.scene = source.name;

    out.scalar = RunScene(source, false);
    out.block = RunScene(source, true);

    const auto fail = [&](bool condition, const std::string& reason) {
        if (condition) {
            out.pass = false;
            out.failures.push_back(reason);
        }
    };

    fail(out.scalar.maxPenetration > RegressionThresholds::kMaxAbsolutePenetration,
         "scalar max penetration exceeded absolute threshold");
    fail(out.block.maxPenetration > RegressionThresholds::kMaxAbsolutePenetration,
         "block max penetration exceeded absolute threshold");

    fail((out.block.maxPenetration - out.scalar.maxPenetration) > RegressionThresholds::kMaxPenetrationRegression,
         "block penetration regression beyond tolerance");
    fail((out.block.contactCountStdDev - out.scalar.contactCountStdDev) > RegressionThresholds::kMaxContactStdDevRegression,
         "block contact-count stddev regression beyond tolerance");
    fail((out.block.contactCountMeanStepDelta - out.scalar.contactCountMeanStepDelta) > RegressionThresholds::kMaxContactStepDeltaRegression,
         "block contact-count step delta regression beyond tolerance");
    fail((out.block.maxImpulseDelta - out.scalar.maxImpulseDelta) > RegressionThresholds::kMaxImpulseDeltaRegression,
         "block max impulse continuity regression beyond tolerance");
    fail((out.block.meanImpulseDelta - out.scalar.meanImpulseDelta) > RegressionThresholds::kMaxMeanImpulseDeltaRegression,
         "block mean impulse continuity regression beyond tolerance");

    if (out.scalar.settleTimeSeconds >= 0.0f && out.block.settleTimeSeconds >= 0.0f) {
        fail((out.block.settleTimeSeconds - out.scalar.settleTimeSeconds)
                 > RegressionThresholds::kMaxSettleTimeRegressionSeconds,
             "block settle-time regression beyond tolerance");
    }

    return out;
}

SceneConfig BuildTwoPointBoxRestingOnPlane() {
    SceneConfig cfg;
    cfg.name = "2-point box resting on plane";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.55f, 0.20f, 0.35f};
    box.mass = 2.0f;
    box.position = {0.0f, 1.3f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9659258f, 0.0f, 0.0f, 0.2588190f});
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 840;
    return cfg;
}

SceneConfig BuildTiltedBoxSettling() {
    SceneConfig cfg;
    cfg.name = "tilted box settling";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.45f, 0.25f, 0.35f};
    box.mass = 2.4f;
    box.position = {0.15f, 1.6f, -0.08f};
    box.orientation = minphys3d::Normalize(Quat{0.9537169f, 0.1331883f, 0.2317487f, 0.1331883f});
    box.angularVelocity = {0.5f, -0.2f, 0.35f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 920;
    return cfg;
}

SceneConfig BuildSlightlyOffsetBoxStacks() {
    SceneConfig cfg;
    cfg.name = "slightly offset box stacks";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body bottom;
    bottom.shape = ShapeType::Box;
    bottom.halfExtents = {0.45f, 0.25f, 0.45f};
    bottom.mass = 3.0f;
    bottom.position = {0.0f, 0.9f, 0.0f};
    const auto bottomId = cfg.world.CreateBody(bottom);

    Body middle = bottom;
    middle.mass = 2.2f;
    middle.position = {0.05f, 1.55f, -0.03f};
    const auto middleId = cfg.world.CreateBody(middle);

    Body top = bottom;
    top.mass = 1.4f;
    top.position = {0.10f, 2.20f, 0.02f};
    const auto topId = cfg.world.CreateBody(top);

    cfg.trackedDynamicBodies = {bottomId, middleId, topId};
    cfg.steps = 1000;
    return cfg;
}

SceneConfig BuildSlidingBoxComingToRest() {
    SceneConfig cfg;
    cfg.name = "sliding box coming to rest";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.35f, 0.20f, 0.30f};
    box.mass = 1.8f;
    box.position = {-1.4f, 0.9f, 0.0f};
    box.velocity = {3.5f, 0.0f, 0.35f};
    box.staticFriction = 0.75f;
    box.dynamicFriction = 0.55f;
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 900;
    return cfg;
}

SceneConfig BuildManifoldReorderStressCase() {
    SceneConfig cfg;
    cfg.name = "manifold reorder stress case";
    cfg.world = World({0.0f, -9.81f, 0.0f});
    cfg.world.CreateBody(MakePlane());

    Body box;
    box.shape = ShapeType::Box;
    box.halfExtents = {0.60f, 0.18f, 0.28f};
    box.mass = 2.6f;
    box.position = {0.0f, 1.25f, 0.0f};
    box.orientation = minphys3d::Normalize(Quat{0.9848077f, 0.0f, 0.1736482f, 0.0f});
    box.velocity = {0.40f, 0.0f, -0.35f};
    box.angularVelocity = {0.0f, 1.0f, 0.0f};
    const auto boxId = cfg.world.CreateBody(box);

    cfg.trackedDynamicBodies = {boxId};
    cfg.steps = 960;
    cfg.logStepContactCount = false;
    cfg.logStepManifoldIds = false;
    return cfg;
}

std::string ToJson(const std::vector<ComparisonResult>& results) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n  \"suite\": \"block_solver_regression\",\n  \"scenes\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const ComparisonResult& r = results[i];
        out << "    {\n";
        out << "      \"name\": \"" << r.scene << "\",\n";
        out << "      \"pass\": " << (r.pass ? "true" : "false") << ",\n";
        out << "      \"scalar\": {\n";
        out << "        \"max_penetration\": " << r.scalar.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.scalar.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.scalar.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.scalar.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.scalar.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.scalar.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.scalar.reorderEvents << "\n";
        out << "      },\n";
        out << "      \"block\": {\n";
        out << "        \"max_penetration\": " << r.block.maxPenetration << ",\n";
        out << "        \"contact_count_stddev\": " << r.block.contactCountStdDev << ",\n";
        out << "        \"contact_count_mean_step_delta\": " << r.block.contactCountMeanStepDelta << ",\n";
        out << "        \"max_impulse_delta\": " << r.block.maxImpulseDelta << ",\n";
        out << "        \"mean_impulse_delta\": " << r.block.meanImpulseDelta << ",\n";
        out << "        \"settle_time_seconds\": " << r.block.settleTimeSeconds << ",\n";
        out << "        \"reorder_events\": " << r.block.reorderEvents << "\n";
        out << "      },\n";
        out << "      \"failures\": [";
        for (std::size_t fi = 0; fi < r.failures.size(); ++fi) {
            if (fi > 0) out << ", ";
            out << "\"" << r.failures[fi] << "\"";
        }
        out << "]\n";
        out << "    }" << (i + 1 == results.size() ? "\n" : ",\n");
    }
    const ContactSolverConfig cfg = MakeSolverConfig(false);
    out << "  ],\n";
    out << "  \"solver_config\": {\n";
    out << "    \"use_split_impulse\": " << (cfg.useSplitImpulse ? "true" : "false") << ",\n";
    out << "    \"split_impulse_correction_factor\": " << cfg.splitImpulseCorrectionFactor << ",\n";
    out << "    \"penetration_bias_factor\": " << cfg.penetrationBiasFactor << ",\n";
    out << "    \"penetration_slop\": " << cfg.penetrationSlop << ",\n";
    out << "    \"restitution_velocity_cutoff\": " << cfg.restitutionVelocityCutoff << ",\n";
    out << "    \"static_friction_speed_threshold\": " << cfg.staticFrictionSpeedThreshold << ",\n";
    out << "    \"static_to_dynamic_transition_speed\": " << cfg.staticToDynamicTransitionSpeed << "\n";
    out << "  }\n}\n";
    return out.str();
}

void PrintHumanSummary(const ComparisonResult& result) {
    const auto printRun = [&](const char* label, const RunMetrics& m) {
        std::cout << "  " << label
                  << " | pen=" << std::fixed << std::setprecision(4) << m.maxPenetration
                  << " contact_stddev=" << m.contactCountStdDev
                  << " contact_dstep=" << m.contactCountMeanStepDelta
                  << " impulse_max_d=" << m.maxImpulseDelta
                  << " impulse_mean_d=" << m.meanImpulseDelta
                  << " settle_s=" << m.settleTimeSeconds
                  << " reorder=" << m.reorderEvents << "\n";
    };

    std::cout << (result.pass ? "PASS" : "FAIL") << " | " << result.scene << "\n";
    printRun("scalar", result.scalar);
    printRun("block ", result.block);
    for (const std::string& failure : result.failures) {
        std::cout << "    - " << failure << "\n";
    }

    if (result.scene == "slightly offset box stacks") {
        std::vector<std::pair<int, float>> divergingFrames;
        const std::size_t frameCount = std::min(result.scalar.stepMaxPenetration.size(), result.block.stepMaxPenetration.size());
        for (std::size_t i = 0; i < frameCount; ++i) {
            const float delta = result.block.stepMaxPenetration[i] - result.scalar.stepMaxPenetration[i];
            if (delta > 0.20f) {
                divergingFrames.emplace_back(static_cast<int>(i), delta);
            }
        }
        std::sort(divergingFrames.begin(), divergingFrames.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second > rhs.second;
        });
        std::cout << "    divergence_frames(block-pen - scalar-pen > 0.20):";
        if (divergingFrames.empty()) {
            std::cout << " none\n";
        } else {
            const std::size_t limit = std::min<std::size_t>(12, divergingFrames.size());
            for (std::size_t i = 0; i < limit; ++i) {
                std::cout << " [step=" << divergingFrames[i].first
                          << " delta=" << std::fixed << std::setprecision(4) << divergingFrames[i].second << "]";
            }
            std::cout << "\n";
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string metricsPath = "tests/block_solver_metrics.json";
    bool logReorderContactCounts = false;
    bool logReorderManifoldIds = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--metrics-out" && i + 1 < argc) {
            metricsPath = argv[++i];
        } else if (arg == "--log-reorder-contact-count") {
            logReorderContactCounts = true;
        } else if (arg == "--log-reorder-manifold-ids") {
            logReorderManifoldIds = true;
        }
    }

    std::vector<SceneConfig> scenes;
    SceneConfig reorderStress = BuildManifoldReorderStressCase();
    reorderStress.logStepContactCount = logReorderContactCounts;
    reorderStress.logStepManifoldIds = logReorderManifoldIds;
    scenes.push_back(std::move(reorderStress));
    scenes.push_back(BuildTwoPointBoxRestingOnPlane());
    scenes.push_back(BuildTiltedBoxSettling());
    scenes.push_back(BuildSlightlyOffsetBoxStacks());
    scenes.push_back(BuildSlidingBoxComingToRest());

    std::vector<ComparisonResult> results;
    results.reserve(scenes.size());

    bool allPass = true;
    for (const SceneConfig& scene : scenes) {
        ComparisonResult result = CompareScene(scene);
        PrintHumanSummary(result);
        allPass = allPass && result.pass;
        results.push_back(std::move(result));
    }

    const std::string json = ToJson(results);
    std::ofstream out(metricsPath);
    if (out) {
        out << json;
    }
    std::cout << "\nMetrics JSON written to: " << metricsPath << "\n";
    std::cout << "Block solver regression suite: " << (allPass ? "PASS" : "FAIL") << "\n";

    return allPass ? 0 : 1;
}
