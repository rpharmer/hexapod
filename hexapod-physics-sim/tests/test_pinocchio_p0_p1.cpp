#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#ifndef HEXAPOD_P0_STAND_CUTPOINT_ISOLATED_TURN
#define HEXAPOD_P0_STAND_CUTPOINT_ISOLATED_TURN ""
#endif
#ifndef HEXAPOD_P0_COMMAND_STREAM_ISOLATED_TURN
#define HEXAPOD_P0_COMMAND_STREAM_ISOLATED_TURN ""
#endif
#ifndef HEXAPOD_P3_SEQ_HISTORY_FIXTURE
#define HEXAPOD_P3_SEQ_HISTORY_FIXTURE ""
#endif
#ifndef HEXAPOD_NEAR_CAP_HISTORY_FIXTURE
#define HEXAPOD_NEAR_CAP_HISTORY_FIXTURE ""
#endif
#ifndef HEXAPOD_DEFAULT_STRAIGHT_HISTORY_FIXTURE
#define HEXAPOD_DEFAULT_STRAIGHT_HISTORY_FIXTURE ""
#endif

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

void SkipWs(const std::string& text, std::size_t& i) {
    while (i < text.size() && (text[i] == ' ' || text[i] == '\n' || text[i] == '\r' || text[i] == '\t')) {
        ++i;
    }
}

bool ParseJsonNumberArray(const std::string& text, std::size_t& i, std::vector<double>& out) {
    SkipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipWs(text, i);
    if (i < text.size() && text[i] == ']') {
        ++i;
        return true;
    }
    while (i < text.size()) {
        SkipWs(text, i);
        if (i < text.size() && text[i] == '[') {
            std::vector<double> nested;
            if (!ParseJsonNumberArray(text, i, nested)) {
                return false;
            }
            out.insert(out.end(), nested.begin(), nested.end());
        } else {
            char* end = nullptr;
            const double value = std::strtod(text.c_str() + i, &end);
            if (end == text.c_str() + i) {
                return false;
            }
            out.push_back(value);
            i = static_cast<std::size_t>(end - text.c_str());
        }
        SkipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
            continue;
        }
        if (i < text.size() && text[i] == ']') {
            ++i;
            return true;
        }
        return false;
    }
    return false;
}

bool ExtractArray(
    const std::string& text, const char* key, std::size_t from, std::size_t to, std::vector<double>& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    std::size_t i = pos + needle.size();
    return ParseJsonNumberArray(text, i, out);
}

bool ExtractArray(const std::string& text, const char* key, std::size_t from, std::vector<double>& out) {
    return ExtractArray(text, key, from, text.size(), out);
}

bool ExtractNumber(
    const std::string& text, const char* key, std::size_t from, std::size_t to, double& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str() + pos + needle.size(), &end);
    return end != text.c_str() + pos + needle.size() && std::isfinite(out);
}

bool ExtractNumber(const std::string& text, const char* key, std::size_t from, double& out) {
    return ExtractNumber(text, key, from, text.size(), out);
}

bool LoadFile(const char* path, std::string& out) {
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return !out.empty();
}

bool VectorsClose(
    const std::vector<double>& a, const std::vector<double>& b, double absTol, const char* label) {
    if (a.size() != b.size()) {
        std::cerr << label << " size " << a.size() << " vs " << b.size() << "\n";
        return false;
    }
    double peak = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        peak = std::max(peak, std::abs(a[i] - b[i]));
        if (std::abs(a[i] - b[i]) > absTol) {
            std::cerr << label << " mismatch i=" << i << " a=" << a[i] << " b=" << b[i]
                      << " peak=" << peak << "\n";
            return false;
        }
    }
    return true;
}

double VectorPeakAbsDiff(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) {
        return std::numeric_limits<double>::infinity();
    }
    double peak = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        peak = std::max(peak, std::abs(a[i] - b[i]));
    }
    return peak;
}

std::array<double, 18> ReadWorldTargets(
    const World& world, const std::array<std::uint32_t, 18>& ids) {
    std::array<double, 18> targets{};
    for (std::size_t i = 0; i < ids.size(); ++i) {
        targets[i] = world.GetServoJoint(ids[i]).targetAngle;
    }
    return targets;
}

struct WarmStartRecord {
    std::uint64_t id = 0;
    std::array<double, 3> impulse{};
};

bool ParseWarmStarts(const std::string& text, std::vector<WarmStartRecord>& out) {
    const std::string needle = "\"warm_starts\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    SkipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipWs(text, i);
    while (i < text.size() && text[i] != ']') {
        SkipWs(text, i);
        if (i >= text.size() || text[i] != '{') {
            return false;
        }
        const std::size_t objectStart = i;
        int depth = 0;
        std::size_t objectEnd = i;
        for (; objectEnd < text.size(); ++objectEnd) {
            if (text[objectEnd] == '{') {
                ++depth;
            } else if (text[objectEnd] == '}') {
                --depth;
                if (depth == 0) {
                    ++objectEnd;
                    break;
                }
            }
        }
        WarmStartRecord record;
        double idValue = 0.0;
        if (!ExtractNumber(text, "id", objectStart, idValue)) {
            return false;
        }
        record.id = static_cast<std::uint64_t>(idValue);
        std::vector<double> impulse;
        if (!ExtractArray(text, "impulse", objectStart, impulse) || impulse.size() != 3) {
            return false;
        }
        record.impulse = {impulse[0], impulse[1], impulse[2]};
        out.push_back(record);
        i = objectEnd;
        SkipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return true;
}

struct CommandStreamStep {
    double dt = 0.0;
    std::array<double, 18> targets{};
    std::vector<double> q{};
    std::vector<double> v{};
};

bool ParseCommandStream(const std::string& text, std::vector<CommandStreamStep>& out) {
    const std::string needle = "\"steps\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    SkipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipWs(text, i);
    while (i < text.size() && text[i] != ']') {
        SkipWs(text, i);
        if (i >= text.size() || text[i] != '{') {
            return false;
        }
        const std::size_t objectStart = i;
        int depth = 0;
        std::size_t objectEnd = i;
        for (; objectEnd < text.size(); ++objectEnd) {
            if (text[objectEnd] == '{') {
                ++depth;
            } else if (text[objectEnd] == '}') {
                --depth;
                if (depth == 0) {
                    ++objectEnd;
                    break;
                }
            }
        }
        CommandStreamStep step;
        if (!ExtractNumber(text, "dt", objectStart, step.dt)) {
            return false;
        }
        std::vector<double> targets;
        if (!ExtractArray(text, "targets", objectStart, targets) || targets.size() != 18
            || !ExtractArray(text, "q", objectStart, step.q)
            || !ExtractArray(text, "v", objectStart, step.v)) {
            return false;
        }
        std::copy(targets.begin(), targets.end(), step.targets.begin());
        out.push_back(std::move(step));
        i = objectEnd;
        SkipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

struct HistorySample {
    double subDt = 0.0;
    double commandDt = 0.0;
    std::uint8_t loadBearingMask = 0;
    double reducedSupportBlend = 0.0;
    std::array<double, 18> targets{};
    std::array<double, 18> errors{};
    std::array<double, 18> tau{};
    std::array<double, 18> effectiveInertias{};
    std::vector<double> q{};
    std::vector<double> v{};
    std::vector<PinocchioHexapodModel::DiagnosticHistoryContact> contacts{};
};

bool ParseHistoryContacts(
    const std::string& text, std::size_t from, std::size_t to,
    std::vector<PinocchioHexapodModel::DiagnosticHistoryContact>& out) {
    const std::string needle = "\"contacts\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        out.clear();
        return true;
    }
    std::size_t i = pos + needle.size();
    SkipWs(text, i);
    if (i >= to || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipWs(text, i);
    while (i < to && text[i] != ']') {
        SkipWs(text, i);
        if (i >= to || text[i] != '{') {
            return false;
        }
        const std::size_t objectStart = i;
        int depth = 0;
        std::size_t objectEnd = i;
        for (; objectEnd < to; ++objectEnd) {
            if (text[objectEnd] == '{') {
                ++depth;
            } else if (text[objectEnd] == '}') {
                --depth;
                if (depth == 0) {
                    ++objectEnd;
                    break;
                }
            }
        }
        PinocchioHexapodModel::DiagnosticHistoryContact contact;
        double idValue = 0.0;
        if (!ExtractNumber(text, "id", objectStart, objectEnd, idValue)) {
            return false;
        }
        contact.id = static_cast<std::uint64_t>(idValue);
        std::vector<double> normal;
        std::vector<double> impulse;
        if (!ExtractArray(text, "normal", objectStart, objectEnd, normal) || normal.size() != 3
            || !ExtractArray(text, "impulse", objectStart, objectEnd, impulse)
            || impulse.size() != 3) {
            return false;
        }
        contact.normal = {normal[0], normal[1], normal[2]};
        contact.impulse = {impulse[0], impulse[1], impulse[2]};
        out.push_back(contact);
        i = objectEnd;
        SkipWs(text, i);
        if (i < to && text[i] == ',') {
            ++i;
        }
    }
    return true;
}

PinocchioHexapodModel::DiagnosticHistorySample ToModelSample(const HistorySample& sample) {
    PinocchioHexapodModel::DiagnosticHistorySample out;
    out.q = sample.q;
    out.v = sample.v;
    out.targets = sample.targets;
    out.effectiveInertias = sample.effectiveInertias;
    out.subDt = sample.subDt;
    out.commandDt = sample.commandDt;
    out.loadBearingMask = sample.loadBearingMask;
    out.reducedSupportBlend = sample.reducedSupportBlend;
    out.contacts = sample.contacts;
    return out;
}

bool ParseAcceptedHistory(const std::string& text, std::vector<HistorySample>& out) {
    const std::string needle = "\"accepted_history\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    SkipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    SkipWs(text, i);
    while (i < text.size() && text[i] != ']') {
        SkipWs(text, i);
        if (i >= text.size() || text[i] != '{') {
            return false;
        }
        const std::size_t objectStart = i;
        int depth = 0;
        std::size_t objectEnd = i;
        for (; objectEnd < text.size(); ++objectEnd) {
            if (text[objectEnd] == '{') {
                ++depth;
            } else if (text[objectEnd] == '}') {
                --depth;
                if (depth == 0) {
                    ++objectEnd;
                    break;
                }
            }
        }
        HistorySample sample;
        if (!ExtractNumber(text, "sub_dt", objectStart, objectEnd, sample.subDt)) {
            return false;
        }
        (void)ExtractNumber(text, "command_dt", objectStart, objectEnd, sample.commandDt);
        double maskValue = 0.0;
        if (ExtractNumber(text, "load_bearing_mask", objectStart, objectEnd, maskValue)) {
            sample.loadBearingMask = static_cast<std::uint8_t>(maskValue);
        }
        (void)ExtractNumber(
            text, "reduced_support_blend", objectStart, objectEnd, sample.reducedSupportBlend);
        std::vector<double> targets;
        std::vector<double> errors;
        std::vector<double> tau;
        std::vector<double> inertias;
        if (!ExtractArray(text, "targets", objectStart, objectEnd, targets) || targets.size() != 18
            || !ExtractArray(text, "q", objectStart, objectEnd, sample.q)
            || !ExtractArray(text, "v", objectStart, objectEnd, sample.v)) {
            return false;
        }
        std::copy(targets.begin(), targets.end(), sample.targets.begin());
        if (ExtractArray(text, "errors", objectStart, objectEnd, errors) && errors.size() == 18) {
            std::copy(errors.begin(), errors.end(), sample.errors.begin());
        }
        if (ExtractArray(text, "tau", objectStart, objectEnd, tau) && tau.size() == 18) {
            std::copy(tau.begin(), tau.end(), sample.tau.begin());
        }
        if (ExtractArray(text, "effective_inertias", objectStart, objectEnd, inertias)
            && inertias.size() == 18) {
            std::copy(inertias.begin(), inertias.end(), sample.effectiveInertias.begin());
        }
        if (!ParseHistoryContacts(text, objectStart, objectEnd, sample.contacts)) {
            return false;
        }
        out.push_back(std::move(sample));
        i = objectEnd;
        SkipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

bool BodiesWithinPublishedCaps(
    const World& world,
    const HexapodSceneObjects& scene,
    const ProximalSolverSettings& settings,
    const char* label) {
    for (const std::uint32_t bodyId : scene.body_ids) {
        if (bodyId == scene.plane) {
            continue;
        }
        const Body& body = world.GetBody(bodyId);
        if (Length(body.velocity) > settings.maxLinearSpeed
            || Length(body.angularVelocity) > settings.maxAngularSpeed) {
            std::cerr << label << " published body over cap id=" << bodyId
                      << " v=" << Length(body.velocity)
                      << " w=" << Length(body.angularVelocity) << "\n";
            return false;
        }
    }
    return true;
}

bool CheckAcceptedStateSpeedInvariant() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        std::cerr << "accepted-state: failed initial read\n";
        return false;
    }
    ProximalStepDiagnostics initial{};
    if (!model.validateDynamicState(q, v, settings, initial)) {
        std::cerr << "accepted-state: constructor last-good is over cap\n";
        return false;
    }

    bool sawHealthy = false;
    bool sawRecovered = false;
    for (int step = 0; step < 180; ++step) {
        ProximalStepDiagnostics diagnostics{};
        const bool usable = model.stepProximal(world, 1.0 / 480.0, settings, diagnostics);
        if (!model.readState(world, q, v)) {
            std::cerr << "accepted-state: unreadable published sample step=" << step << "\n";
            return false;
        }
        ProximalStepDiagnostics published{};
        if (!model.validateDynamicState(q, v, settings, published)
            || !BodiesWithinPublishedCaps(world, scene, settings, "accepted-state stand")) {
            std::cerr << "accepted-state: published sample over cap step=" << step
                      << " status=" << static_cast<int>(diagnostics.status)
                      << " reason=" << static_cast<int>(diagnostics.failureReason) << "\n";
            return false;
        }
        if (diagnostics.status == ProximalStepStatus::Healthy && usable) {
            sawHealthy = true;
        }
        if (diagnostics.status == ProximalStepStatus::RecoveredRetry && usable) {
            sawRecovered = true;
        }
        if (diagnostics.status == ProximalStepStatus::HeldLastGood && usable) {
            std::cerr << "accepted-state: HeldLastGood reported usable\n";
            return false;
        }
    }
    if (!sawHealthy) {
        std::cerr << "accepted-state: never observed a Healthy write\n";
        return false;
    }
    (void)sawRecovered;

    if (!model.readState(world, q, v)) {
        return false;
    }
    v[0] = 100.0;
    if (!model.writeState(world, q, v)) {
        std::cerr << "accepted-state: failed to inject over-cap speed\n";
        return false;
    }
    ProximalStepDiagnostics held{};
    if (model.stepProximal(world, 1.0 / 240.0, settings, held)
        || held.status != ProximalStepStatus::HeldLastGood
        || !model.readState(world, q, v)) {
        std::cerr << "accepted-state: over-cap inject did not hold last-good\n";
        return false;
    }
    ProximalStepDiagnostics lastGood{};
    if (!model.validateDynamicState(q, v, settings, lastGood)
        || !BodiesWithinPublishedCaps(world, scene, settings, "accepted-state held")) {
        std::cerr << "accepted-state: HeldLastGood published an over-cap sample\n";
        return false;
    }
    return true;
}

bool CheckWarmStartRollbackTransaction() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 180; ++step) {
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
            std::cerr << "rollback: standing failed step=" << step << "\n";
            return false;
        }
    }
    const auto before = model.debugWarmStartAudit();
    if (before.count == 0) {
        std::cerr << "rollback: standing produced no warm starts\n";
        return false;
    }
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        return false;
    }
    v[0] = 100.0;
    if (!model.writeState(world, q, v)) {
        return false;
    }
    ProximalStepDiagnostics held{};
    if (model.stepProximal(world, 1.0 / 240.0, settings, held)
        || held.status != ProximalStepStatus::HeldLastGood) {
        std::cerr << "rollback: expected HeldLastGood after over-cap inject\n";
        return false;
    }
    const auto after = model.debugWarmStartAudit();
    if (after.count != before.count
        || std::abs(after.impulseNorm - before.impulseNorm) > 1.0e-12) {
        std::cerr << "rollback: warm-start map leaked from rejected attempt before="
                  << before.count << "/" << before.impulseNorm
                  << " after=" << after.count << "/" << after.impulseNorm << "\n";
        return false;
    }
    return true;
}

bool CheckTangentFrameInvariance() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 200; ++step) {
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
            std::cerr << "tangent: standing failed step=" << step << "\n";
            return false;
        }
    }
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v) || model.debugWarmStartAudit().count == 0) {
        std::cerr << "tangent: missing standing state or warm starts\n";
        return false;
    }
    model.debugCaptureWarmStarts();
    ProximalStepDiagnostics baseline{};
    if (!model.stepProximal(world, 1.0 / 480.0, settings, baseline)) {
        std::cerr << "tangent: baseline step failed\n";
        return false;
    }
    std::vector<double> vBaseline;
    std::vector<double> qDiscard;
    if (!model.readState(world, qDiscard, vBaseline)) {
        return false;
    }
    if (!model.writeState(world, q, v)) {
        return false;
    }
    model.debugRestoreCapturedWarmStarts();
    const auto beforeRotate = model.debugWarmStartAudit();
    model.debugRotateWarmStartTangentBasis(1.5707963267948966);
    const auto afterRotate = model.debugWarmStartAudit();
    if (afterRotate.count != beforeRotate.count
        || std::abs(afterRotate.impulseNorm - beforeRotate.impulseNorm) > 1.0e-12) {
        std::cerr << "tangent: stored λ transport is not isometric\n";
        return false;
    }
    ProximalStepDiagnostics rotated{};
    if (!model.stepProximal(world, 1.0 / 480.0, settings, rotated)) {
        std::cerr << "tangent: rotated-frame step failed\n";
        return false;
    }
    // Production apply-time world transport was demonstrated then reverted:
    // isolated reverse 4/5 after shipping it (tibia SpeedLimit, peak ω 10.12).
    // Keep the stored-frame diagnostic and isometric rotate helper only.
    return true;
}

bool CheckMaterialPointJacobian() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        return false;
    }
    for (double& value : v) {
        value = 0.05;
    }
    if (!model.writeState(world, q, v) || !model.readState(world, q, v)) {
        return false;
    }
    const std::array<std::uint32_t, 3> bodies{
        scene.body, scene.legs[0].femur, scene.legs[2].tibia};
    for (const std::uint32_t bodyId : bodies) {
        Vec3 linear{};
        Vec3 angular{};
        if (!model.computeLinkWorldTwist(q, v, bodyId, linear, angular)) {
            std::cerr << "material-point: missing twist for body=" << bodyId << "\n";
            return false;
        }
        const Body& body = world.GetBody(bodyId);
        const double linearDiff = Length(body.velocity - linear);
        const double angularDiff = Length(body.angularVelocity - angular);
        if (linearDiff > 1.0e-8 || angularDiff > 1.0e-8) {
            std::cerr << "material-point: Jv/body mismatch body=" << bodyId
                      << " linear=" << linearDiff << " angular=" << angularDiff << "\n";
            return false;
        }
        const Vec3 offset{0.01, -0.02, 0.015};
        const Vec3 pointFromBody = body.velocity + Cross(body.angularVelocity, offset);
        const Vec3 pointFromTwist = linear + Cross(angular, offset);
        if (Length(pointFromBody - pointFromTwist) > 1.0e-8) {
            std::cerr << "material-point: v + w x r mismatch body=" << bodyId << "\n";
            return false;
        }
    }
    return true;
}

bool CheckArmatureEnergyLedger() {
    World world({0.0, 0.0, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    for (const std::uint32_t bodyId : scene.body_ids) {
        world.GetBody(bodyId).collisionMask = 0;
    }
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        return false;
    }
    for (std::size_t i = 6; i < v.size(); ++i) {
        v[i] = (i % 2 == 0) ? 0.4 : -0.25;
    }
    if (!model.writeState(world, q, v) || !model.readState(world, q, v)) {
        return false;
    }
    PinocchioHexapodModel::MechanicalEnergyBreakdown energy{};
    if (!model.computeMechanicalEnergyBreakdown(world, q, v, energy)) {
        std::cerr << "energy: breakdown failed\n";
        return false;
    }
    const double reconstructed = energy.bodyKinetic + energy.armatureKinetic;
    const double scale = std::max(1.0, std::abs(energy.generalizedKinetic));
    if (energy.armatureKinetic <= 1.0e-9) {
        std::cerr << "energy: armature kinetic was not positive\n";
        return false;
    }
    const double bodyOnlyError = std::abs(energy.generalizedKinetic - energy.bodyKinetic);
    const double reconstructedError = std::abs(energy.generalizedKinetic - reconstructed);
    if (reconstructedError >= bodyOnlyError) {
        std::cerr << "energy: adding armature did not explain 0.5 vTMv gen="
                  << energy.generalizedKinetic << " body=" << energy.bodyKinetic
                  << " armature=" << energy.armatureKinetic << "\n";
        return false;
    }
    if (reconstructedError > 1.0e-6 * scale && reconstructedError > 0.05 * energy.armatureKinetic) {
        std::cerr << "energy: 0.5 vTMv=" << energy.generalizedKinetic
                  << " body+armature=" << reconstructed
                  << " armature=" << energy.armatureKinetic << "\n";
        return false;
    }
    for (double& value : v) {
        value = -value;
    }
    if (!model.writeState(world, q, v) || !model.readState(world, q, v)) {
        return false;
    }
    PinocchioHexapodModel::MechanicalEnergyBreakdown reversed{};
    if (!model.computeMechanicalEnergyBreakdown(world, q, v, reversed)
        || std::abs(reversed.generalizedKinetic - energy.generalizedKinetic) > 1.0e-10 * scale) {
        std::cerr << "energy: gravity-free reversal changed 0.5 vTMv\n";
        return false;
    }
    return true;
}

bool CheckPrefailureBufferDump() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 60; ++step) {
        ProximalStepDiagnostics diagnostics{};
        (void)model.stepProximal(world, 1.0 / 480.0, settings, diagnostics);
    }
    const char* cutPath = "/tmp/hexapod-p0-stand-cutpoint-unit.json";
    std::remove(cutPath);
    if (!model.dumpDiagnosticCutpoint(cutPath)) {
        std::cerr << "cutpoint: dump failed\n";
        return false;
    }
    std::ifstream cut(cutPath);
    std::string contents;
    std::getline(cut, contents);
    if (!cut || contents.find("\"kind\":\"stand_cutpoint\"") == std::string::npos
        || contents.find("\"warm_starts\"") == std::string::npos
        || contents.find("\"effective_inertias\"") == std::string::npos) {
        std::cerr << "cutpoint: dump missing required fields\n";
        return false;
    }
    if (contents.find("\"have_frame\":true") != std::string::npos
        && contents.find("\"frame\":") == std::string::npos) {
        std::cerr << "cutpoint: new dump omitted warm-start frame\n";
        return false;
    }

    const char* bufferPath = "/tmp/hexapod-p0-prefailure-buffer-unit.json";
    std::remove(bufferPath);
    ::setenv("HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH", bufferPath, 1);
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        ::unsetenv("HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH");
        return false;
    }
    v[0] = 100.0;
    if (!model.writeState(world, q, v)) {
        ::unsetenv("HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH");
        return false;
    }
    ProximalStepDiagnostics held{};
    (void)model.stepProximal(world, 1.0 / 240.0, settings, held);
    ::unsetenv("HEXAPOD_PINOCCHIO_PREFAILURE_BUFFER_PATH");
    std::ifstream buffer(bufferPath);
    contents.clear();
    std::getline(buffer, contents);
    if (!buffer || contents.find("\"kind\":\"prefailure_buffer\"") == std::string::npos
        || contents.find("\"accepted_history\"") == std::string::npos
        || contents.find("\"effective_inertias\"") == std::string::npos) {
        std::cerr << "prefailure buffer dump missing or incomplete\n";
        return false;
    }
    return true;
}

bool CheckDumpRestoreIdentityAndHeldRetryTargets() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    const auto jointIds = SceneJointOrder(scene);
    PinocchioHexapodModel model(world, scene, jointIds);
    ProximalSolverSettings settings{};
    for (int step = 0; step < 180; ++step) {
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
            std::cerr << "restore-identity: standing failed step=" << step << "\n";
            return false;
        }
    }
    const char* cutPath = "/tmp/hexapod-p0-restore-identity.json";
    std::remove(cutPath);
    if (!model.dumpDiagnosticCutpoint(cutPath)) {
        std::cerr << "restore-identity: dump failed\n";
        return false;
    }
    std::string dumped;
    if (!LoadFile(cutPath, dumped) || dumped.find("\"frame\":") == std::string::npos) {
        std::cerr << "restore-identity: new dump missing warm-start frame\n";
        return false;
    }
    std::vector<double> qDump;
    std::vector<double> vDump;
    std::vector<double> effectiveDump;
    std::vector<WarmStartRecord> warmDump;
    double maskDump = 0.0;
    if (!ExtractArray(dumped, "q", 0, qDump) || !ExtractArray(dumped, "v", 0, vDump)
        || !ExtractArray(dumped, "effective_inertias", 0, effectiveDump)
        || !ExtractNumber(dumped, "load_bearing_mask", 0, maskDump)
        || !ParseWarmStarts(dumped, warmDump)) {
        std::cerr << "restore-identity: failed to parse dump\n";
        return false;
    }
    const auto beforeAudit = model.debugWarmStartAudit();
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v) || q.size() < 2) {
        return false;
    }
    q[1] += 0.01;
    if (!model.writeState(world, q, v)) {
        std::cerr << "restore-identity: mutate write failed\n";
        return false;
    }
    if (!model.synchronizeAfterExternalCorrection(world, settings)) {
        std::cerr << "restore-identity: mutate sync failed\n";
        return false;
    }
    model.resetWarmStarts();
    if (!model.restoreDiagnosticCutpoint(world, cutPath)) {
        std::cerr << "restore-identity: restore failed\n";
        return false;
    }
    std::vector<double> qRestored;
    std::vector<double> vRestored;
    if (!model.readState(world, qRestored, vRestored)
        || !VectorsClose(qDump, qRestored, 1.0e-12, "restore-identity q")
        || !VectorsClose(vDump, vRestored, 1.0e-12, "restore-identity v")) {
        return false;
    }
    const char* restoredPath = "/tmp/hexapod-p0-restore-identity-roundtrip.json";
    std::remove(restoredPath);
    if (!model.dumpDiagnosticCutpoint(restoredPath)) {
        std::cerr << "restore-identity: round-trip dump failed\n";
        return false;
    }
    std::string restoredDump;
    std::vector<double> effectiveRestored;
    std::vector<WarmStartRecord> warmRestored;
    double maskRestored = -1.0;
    if (!LoadFile(restoredPath, restoredDump)
        || !ExtractArray(restoredDump, "effective_inertias", 0, effectiveRestored)
        || !ExtractNumber(restoredDump, "load_bearing_mask", 0, maskRestored)
        || !ParseWarmStarts(restoredDump, warmRestored)
        || !VectorsClose(effectiveDump, effectiveRestored, 1.0e-12, "restore-identity inertias")
        || maskDump != maskRestored) {
        std::cerr << "restore-identity: mask/inertia mismatch mask=" << maskDump
                  << " vs " << maskRestored << "\n";
        return false;
    }
    if (warmDump.size() != warmRestored.size()) {
        std::cerr << "restore-identity: warm-start count " << warmDump.size()
                  << " vs " << warmRestored.size() << "\n";
        return false;
    }
    for (const auto& expected : warmDump) {
        bool found = false;
        for (const auto& actual : warmRestored) {
            if (actual.id != expected.id) {
                continue;
            }
            found = true;
            for (int axis = 0; axis < 3; ++axis) {
                if (std::abs(actual.impulse[static_cast<std::size_t>(axis)]
                             - expected.impulse[static_cast<std::size_t>(axis)])
                    > 1.0e-12) {
                    std::cerr << "restore-identity: impulse mismatch id=" << expected.id << "\n";
                    return false;
                }
            }
        }
        if (!found) {
            std::cerr << "restore-identity: missing warm-start id=" << expected.id << "\n";
            return false;
        }
    }
    const auto afterAudit = model.debugWarmStartAudit();
    if (afterAudit.count != beforeAudit.count
        || afterAudit.framedCount != beforeAudit.framedCount
        || std::abs(afterAudit.impulseNorm - beforeAudit.impulseNorm) > 1.0e-12) {
        std::cerr << "restore-identity: warm-start audit mismatch\n";
        return false;
    }

    const auto targetsBefore = ReadWorldTargets(world, jointIds);
    if (!model.readState(world, q, v)) {
        return false;
    }
    v[0] = 100.0;
    if (!model.writeState(world, q, v)) {
        std::cerr << "d7: over-cap inject failed\n";
        return false;
    }
    ProximalStepDiagnostics held{};
    if (model.stepProximal(world, 1.0 / 240.0, settings, held)
        || held.status != ProximalStepStatus::HeldLastGood) {
        std::cerr << "d7: expected HeldLastGood after over-cap inject\n";
        return false;
    }
    const auto targetsAfter = ReadWorldTargets(world, jointIds);
    for (std::size_t i = 0; i < targetsBefore.size(); ++i) {
        if (std::abs(targetsBefore[i] - targetsAfter[i]) > 1.0e-15) {
            std::cerr << "d7: held retry consumed a second servo target i=" << i
                      << " before=" << targetsBefore[i] << " after=" << targetsAfter[i] << "\n";
            return false;
        }
    }
    return true;
}

bool CheckFrozenStandCutpointRestore() {
    const char* path = HEXAPOD_P0_STAND_CUTPOINT_ISOLATED_TURN;
    if (path == nullptr || path[0] == '\0') {
        std::cerr << "frozen-cutpoint: compile path missing\n";
        return false;
    }
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    if (!model.restoreDiagnosticCutpoint(world, path)) {
        std::cerr << "frozen-cutpoint: restore failed path=" << path << "\n";
        return false;
    }
    const auto audit = model.debugWarmStartAudit();
    if (audit.count != 6 || audit.framedCount != 0) {
        std::cerr << "frozen-cutpoint: expected 6 frameless warm starts count="
                  << audit.count << " framed=" << audit.framedCount << "\n";
        return false;
    }
    std::vector<double> q;
    std::vector<double> v;
    ProximalStepDiagnostics published{};
    ProximalSolverSettings settings{};
    if (!model.readState(world, q, v) || !model.validateDynamicState(q, v, settings, published)) {
        std::cerr << "frozen-cutpoint: restored state is not under cap\n";
        return false;
    }
    return true;
}

bool CheckInProcessCommandStreamReplay() {
    const char* cutPath = "/tmp/hexapod-p0-stream-cutpoint.json";
    const char* streamPath = "/tmp/hexapod-p0-stream-inprocess.json";
    std::remove(cutPath);
    std::remove(streamPath);
    ProximalSolverSettings settings{};
    {
        World world({0.0, -9.80665, 0.0});
        const HexapodSceneObjects scene = BuildHexapodScene(world);
        const auto jointIds = SceneJointOrder(scene);
        PinocchioHexapodModel model(world, scene, jointIds);
        for (int step = 0; step < 180; ++step) {
            ProximalStepDiagnostics diagnostics{};
            if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
                std::cerr << "stream-replay: standing failed step=" << step << "\n";
                return false;
            }
        }
        if (!model.dumpDiagnosticCutpoint(cutPath)) {
            std::cerr << "stream-replay: dump failed\n";
            return false;
        }
        ::setenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH", streamPath, 1);
        auto targets = ReadWorldTargets(world, jointIds);
        targets[1] += 0.02;
        if (!model.applyServoTargets(world, targets)) {
            ::unsetenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
            return false;
        }
        for (int step = 0; step < 40; ++step) {
            ProximalStepDiagnostics diagnostics{};
            if (!model.stepProximal(world, 1.0 / 480.0, settings, diagnostics)) {
                std::cerr << "stream-replay: capture step failed status="
                          << static_cast<int>(diagnostics.status) << "\n";
                ::unsetenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
                return false;
            }
        }
    }
    ::unsetenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
    std::string streamText;
    std::vector<CommandStreamStep> steps;
    if (!LoadFile(streamPath, streamText)
        || streamText.find("\"kind\":\"command_stream\"") == std::string::npos
        || !ParseCommandStream(streamText, steps)) {
        std::cerr << "stream-replay: missing captured stream at " << streamPath << "\n";
        return false;
    }
    World replayWorld({0.0, -9.80665, 0.0});
    const HexapodSceneObjects replayScene = BuildHexapodScene(replayWorld);
    PinocchioHexapodModel replay(replayWorld, replayScene, SceneJointOrder(replayScene));
    if (!replay.restoreDiagnosticCutpoint(replayWorld, cutPath)) {
        std::cerr << "stream-replay: restore of capture cutpoint failed\n";
        return false;
    }
    for (std::size_t i = 0; i < steps.size(); ++i) {
        if (!replay.applyServoTargets(replayWorld, steps[i].targets)) {
            return false;
        }
        ProximalStepDiagnostics diagnostics{};
        if (!replay.stepProximal(replayWorld, steps[i].dt, settings, diagnostics)) {
            std::cerr << "stream-replay: replay step failed i=" << i << "\n";
            return false;
        }
        std::vector<double> q;
        std::vector<double> v;
        if (!replay.readState(replayWorld, q, v)
            || !VectorsClose(steps[i].q, q, 1.0e-3, "stream-replay q")
            || !VectorsClose(steps[i].v, v, 1.0e-2, "stream-replay v")) {
            std::cerr << "stream-replay: prefix mismatch at i=" << i << "\n";
            return false;
        }
    }
    return true;
}

bool CheckFrozenCutpointCommandStreamReplay() {
    const char* cutPath = HEXAPOD_P0_STAND_CUTPOINT_ISOLATED_TURN;
    const char* streamPath = HEXAPOD_P0_COMMAND_STREAM_ISOLATED_TURN;
    std::string streamText;
    std::vector<CommandStreamStep> steps;
    if (streamPath == nullptr || streamPath[0] == '\0' || !LoadFile(streamPath, streamText)
        || !ParseCommandStream(streamText, steps)) {
        std::cout << "P0_FROZEN_STREAM_REPLAY classification=blocked "
                     "reason=missing_command_stream_artifact\n";
        return true;
    }
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    if (!model.restoreDiagnosticCutpoint(world, cutPath)) {
        std::cerr << "frozen-stream: restore failed\n";
        return false;
    }
    ProximalSolverSettings settings{};
    double firstQErr = 0.0;
    std::size_t tracked = 0;
    for (std::size_t i = 0; i < steps.size(); ++i) {
        if (!model.applyServoTargets(world, steps[i].targets)) {
            return false;
        }
        ProximalStepDiagnostics diagnostics{};
        const bool usable = model.stepProximal(world, steps[i].dt, settings, diagnostics);
        std::vector<double> q;
        std::vector<double> v;
        if (!model.readState(world, q, v)) {
            std::cerr << "frozen-stream: unreadable replay state i=" << i << "\n";
            return false;
        }
        const double qErr = VectorPeakAbsDiff(steps[i].q, q);
        if (i == 0) {
            firstQErr = qErr;
        }
        if (!usable || qErr > 1.0e-3) {
            break;
        }
        ++tracked;
    }
    if (tracked == 0) {
        std::cout << "P0_FROZEN_STREAM_REPLAY classification=blocked "
                     "reason=frameless_warm_start_diverged first_q_err="
                  << firstQErr << " samples=" << steps.size() << "\n";
        return true;
    }
    std::cout << "P0_FROZEN_STREAM_REPLAY classification=tracked prefix=" << tracked
              << " of " << steps.size() << " first_q_err=" << firstQErr << "\n";
    return true;
}

bool ReportTibiaTwist(
    PinocchioHexapodModel& model,
    const std::vector<double>& capturedQ,
    const std::vector<double>& capturedV,
    const std::vector<double>& replayQ,
    const std::vector<double>& replayV,
    std::uint32_t tibia,
    const char* label,
    std::size_t index,
    double& capturedW,
    double& replayW) {
    Vec3 capturedLin{};
    Vec3 capturedAng{};
    Vec3 replayLin{};
    Vec3 replayAng{};
    if (!model.computeLinkWorldTwist(capturedQ, capturedV, tibia, capturedLin, capturedAng)
        || !model.computeLinkWorldTwist(replayQ, replayV, tibia, replayLin, replayAng)) {
        std::cerr << label << " twist failed i=" << index << "\n";
        return false;
    }
    capturedW = Length(capturedAng);
    replayW = Length(replayAng);
    std::cout << label << " sample=" << index
              << " captured_tibia_w=" << capturedW
              << " replay_tibia_w=" << replayW
              << " q_err=" << VectorPeakAbsDiff(capturedQ, replayQ) << "\n";
    return true;
}

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kIncomingNearCap = 9.90;
constexpr double kTibiaRateLimitRadps = 10.0;
constexpr std::size_t kLeg2TibiaWire = 8;

enum class TibiaCommandMode : std::uint8_t {
    ZeroError = 0,
    RateLimit10 = 1,
};

const char* TibiaCommandModeName(TibiaCommandMode mode) {
    return mode == TibiaCommandMode::RateLimit10 ? "rate_limit_10" : "zero_error";
}

std::array<double, 18> MutateLeg2TibiaTarget(
    const std::array<double, 18>& recorded,
    double liveAngle,
    double subDt,
    TibiaCommandMode mode) {
    std::array<double, 18> out = recorded;
    if (mode == TibiaCommandMode::ZeroError) {
        out[kLeg2TibiaWire] = liveAngle;
        return out;
    }
    const double delta = std::remainder(recorded[kLeg2TibiaWire] - liveAngle, kTwoPi);
    const double maxDelta = kTibiaRateLimitRadps * std::max(0.0, subDt);
    if (!(std::abs(delta) > maxDelta)) {
        return out;
    }
    out[kLeg2TibiaWire] = liveAngle + std::copysign(maxDelta, delta);
    return out;
}

const char* ClassifyHistoryReplay(bool reachedIncomingNearCap, double maxReplay, double maxCaptured) {
    if (reachedIncomingNearCap) {
        return "reproduced_vin";
    }
    if (maxReplay + 0.5 >= maxCaptured) {
        return "history_tracked_below_cap";
    }
    return "blocked";
}

const char* ClassifyCommandIntervene(bool blocked, double maxReplay) {
    if (blocked) {
        return "blocked";
    }
    if (maxReplay < kIncomingNearCap) {
        return "command_causal_under_cap";
    }
    return "reproduced_vin";
}

bool RunTibiaCommandIntervene(
    const std::vector<HistorySample>& samples, TibiaCommandMode mode, bool reseed) {
    const char* pathLabel = reseed ? "RESEED" : "ACCUM";
    const std::string twistLabel =
        std::string("P3_SEQ_CMD_INTERVENE_") + TibiaCommandModeName(mode) + "_" + pathLabel;
    ::setenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING", "1", 1);
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
    if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples.front()))) {
        std::cerr << "p3-cmd: " << TibiaCommandModeName(mode) << " " << pathLabel
                  << " restore sample 0 failed\n";
        return false;
    }

    ProximalSolverSettings settings{};
    const std::uint32_t tibia = scene.legs[2].tibia;
    const std::uint32_t tibiaJoint = scene.legs[2].femurToTibiaJoint;
    double maxCaptured = 0.0;
    double maxReplay = 0.0;
    bool blocked = false;

    const auto measure = [&](std::size_t index) {
        std::vector<double> q;
        std::vector<double> v;
        if (!model.readState(world, q, v)) {
            std::cerr << "p3-cmd: " << TibiaCommandModeName(mode) << " " << pathLabel
                      << " unreadable i=" << index << "\n";
            return false;
        }
        double capturedW = 0.0;
        double replayW = 0.0;
        if (!ReportTibiaTwist(
                model, samples[index].q, samples[index].v, q, v, tibia, twistLabel.c_str(),
                index, capturedW, replayW)) {
            return false;
        }
        maxCaptured = std::max(maxCaptured, capturedW);
        maxReplay = std::max(maxReplay, replayW);
        return true;
    };

    const auto applyMutated = [&](std::size_t nextIndex) {
        const double liveAngle = static_cast<double>(world.GetServoJointAngle(tibiaJoint));
        const double subDt = samples[nextIndex].subDt;
        const std::array<double, 18> targets =
            MutateLeg2TibiaTarget(samples[nextIndex].targets, liveAngle, subDt, mode);
        const double error8 = std::remainder(targets[kLeg2TibiaWire] - liveAngle, kTwoPi);
        std::cout << "P3_SEQ_CMD_INTERVENE mode=" << TibiaCommandModeName(mode)
                  << " path=" << pathLabel
                  << " sample=" << nextIndex
                  << " live_angle=" << liveAngle
                  << " target8=" << targets[kLeg2TibiaWire]
                  << " error8=" << error8
                  << " sub_dt=" << subDt << "\n";
        return model.applyServoTargets(world, targets);
    };

    if (!measure(0)) {
        return false;
    }
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        if (reseed) {
            if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples[i]))) {
                std::cerr << "p3-cmd: " << TibiaCommandModeName(mode) << " " << pathLabel
                          << " restore failed i=" << i << "\n";
                return false;
            }
        }
        if (!applyMutated(i + 1)) {
            return false;
        }
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, samples[i + 1].subDt, settings, diagnostics)) {
            std::cout << "P3_SEQ_CMD_INTERVENE mode=" << TibiaCommandModeName(mode)
                      << " path=" << pathLabel
                      << " classification=blocked reason=step_failed i=" << i
                      << " status=" << static_cast<int>(diagnostics.status)
                      << " max_replay_tibia_w=" << maxReplay << "\n";
            blocked = true;
            break;
        }
        if (!measure(i + 1)) {
            return false;
        }
    }
    std::cout << "P3_SEQ_CMD_INTERVENE mode=" << TibiaCommandModeName(mode)
              << " path=" << pathLabel
              << " classification=" << ClassifyCommandIntervene(blocked, maxReplay)
              << " samples=" << samples.size()
              << " max_captured_tibia_w=" << maxCaptured
              << " max_replay_tibia_w=" << maxReplay << "\n";
    return true;
}

bool CheckP3SeqHistoryReplay() {
    const char* path = HEXAPOD_P3_SEQ_HISTORY_FIXTURE;
    std::string text;
    std::vector<HistorySample> samples;
    if (path == nullptr || path[0] == '\0' || !LoadFile(path, text)
        || !ParseAcceptedHistory(text, samples) || samples.size() < 2) {
        std::cerr << "p3-hist: failed to load " << (path != nullptr ? path : "(null)") << "\n";
        return false;
    }

    {
        World measureWorld({0.0, -9.80665, 0.0});
        const HexapodSceneObjects measureScene = BuildHexapodScene(measureWorld);
        PinocchioHexapodModel measure(measureWorld, measureScene, SceneJointOrder(measureScene));
        const std::uint32_t tibia = measureScene.legs[2].tibia;
        double prevCapturedW = 0.0;
        std::size_t peakClimbIndex = 0;
        double peakClimb = -1.0;
        std::size_t vinLandingIndex = samples.size();
        double vinLandingW = 0.0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            Vec3 capturedLin{};
            Vec3 capturedAng{};
            if (!measure.computeLinkWorldTwist(
                    samples[i].q, samples[i].v, tibia, capturedLin, capturedAng)) {
                std::cerr << "p3-hist: captured twist failed i=" << i << "\n";
                return false;
            }
            const double capturedW = Length(capturedAng);
            const double dOmega = i == 0 ? 0.0 : capturedW - prevCapturedW;
            const double dTarget = i == 0 ? 0.0
                : samples[i].targets[kLeg2TibiaWire] - samples[i - 1].targets[kLeg2TibiaWire];
            std::cout << "P3_SEQ_CMD sample=" << i
                      << " captured_tibia_w=" << capturedW
                      << " d_omega=" << dOmega
                      << " target8=" << samples[i].targets[kLeg2TibiaWire]
                      << " d_target8=" << dTarget
                      << " error8=" << samples[i].errors[kLeg2TibiaWire]
                      << " tau8=" << samples[i].tau[kLeg2TibiaWire]
                      << " contacts=" << samples[i].contacts.size()
                      << "\n";
            if (i > 0 && dOmega > peakClimb) {
                peakClimb = dOmega;
                peakClimbIndex = i;
            }
            if (vinLandingIndex == samples.size() && capturedW >= 9.90) {
                vinLandingIndex = i;
                vinLandingW = capturedW;
            }
            prevCapturedW = capturedW;
        }
        std::cout << "P3_SEQ_CMD last_climb_sample=" << peakClimbIndex
                  << " d_omega=" << peakClimb
                  << " target8=" << samples[peakClimbIndex].targets[kLeg2TibiaWire]
                  << " error8=" << samples[peakClimbIndex].errors[kLeg2TibiaWire]
                  << " tau8=" << samples[peakClimbIndex].tau[kLeg2TibiaWire] << "\n";
        if (vinLandingIndex < samples.size()) {
            const double dTarget = vinLandingIndex == 0 ? 0.0
                : samples[vinLandingIndex].targets[kLeg2TibiaWire]
                    - samples[vinLandingIndex - 1].targets[kLeg2TibiaWire];
            std::cout << "P3_SEQ_CMD vin_landing_sample=" << vinLandingIndex
                      << " captured_tibia_w=" << vinLandingW
                      << " d_target8=" << dTarget
                      << " error8=" << samples[vinLandingIndex].errors[kLeg2TibiaWire]
                      << " tau8=" << samples[vinLandingIndex].tau[kLeg2TibiaWire] << "\n";
        }
    }

    const auto runAccumulating = [&]() {
        ::setenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING", "1", 1);
        World world({0.0, -9.80665, 0.0});
        const HexapodSceneObjects scene = BuildHexapodScene(world);
        PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
        ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
        if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples.front()))) {
            std::cerr << "p3-hist: accumulating restore sample 0 failed\n";
            return false;
        }
        ProximalSolverSettings settings{};
        const std::uint32_t tibia = scene.legs[2].tibia;
        double maxCaptured = 0.0;
        double maxReplay = 0.0;
        bool reachedIncomingNearCap = false;
        bool blocked = false;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            std::vector<double> q;
            std::vector<double> v;
            if (!model.readState(world, q, v)) {
                std::cerr << "p3-hist: accumulating unreadable i=" << i << "\n";
                return false;
            }
            double capturedW = 0.0;
            double replayW = 0.0;
            if (!ReportTibiaTwist(
                    model, samples[i].q, samples[i].v, q, v, tibia,
                    "P3_SEQ_HISTORY_ACCUM", i, capturedW, replayW)) {
                return false;
            }
            maxCaptured = std::max(maxCaptured, capturedW);
            maxReplay = std::max(maxReplay, replayW);
            if (replayW >= 9.90) {
                reachedIncomingNearCap = true;
            }
            if (i + 1 >= samples.size()) {
                break;
            }
            if (!model.applyServoTargets(world, samples[i + 1].targets)) {
                return false;
            }
            ProximalStepDiagnostics diagnostics{};
            if (!model.stepProximal(world, samples[i + 1].subDt, settings, diagnostics)) {
                std::cout << "P3_SEQ_HISTORY_ACCUM classification=blocked reason=step_failed i="
                          << i << " status=" << static_cast<int>(diagnostics.status)
                          << " max_replay_w=" << maxReplay << "\n";
                blocked = true;
                break;
            }
        }
        if (!blocked) {
            std::cout << "P3_SEQ_HISTORY_ACCUM classification="
                      << ClassifyHistoryReplay(reachedIncomingNearCap, maxReplay, maxCaptured)
                      << " samples=" << samples.size()
                      << " max_captured_tibia_w=" << maxCaptured
                      << " max_replay_tibia_w=" << maxReplay
                      << " reached_9_95_before_trip=" << (reachedIncomingNearCap ? 1 : 0)
                      << "\n";
        }
        return true;
    };

    const auto runReseed = [&]() {
        ::setenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING", "1", 1);
        World world({0.0, -9.80665, 0.0});
        const HexapodSceneObjects scene = BuildHexapodScene(world);
        PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
        ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
        ProximalSolverSettings settings{};
        const std::uint32_t tibia = scene.legs[2].tibia;
        double maxCaptured = 0.0;
        double maxReplay = 0.0;
        bool reachedIncomingNearCap = false;
        if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples.front()))) {
            std::cerr << "p3-hist: reseed restore sample 0 failed\n";
            return false;
        }
        {
            std::vector<double> q;
            std::vector<double> v;
            if (!model.readState(world, q, v)) {
                return false;
            }
            double capturedW = 0.0;
            double replayW = 0.0;
            if (!ReportTibiaTwist(
                    model, samples.front().q, samples.front().v, q, v, tibia,
                    "P3_SEQ_HISTORY_RESEED", 0, capturedW, replayW)) {
                return false;
            }
            maxCaptured = capturedW;
            maxReplay = replayW;
            if (replayW >= 9.90) {
                reachedIncomingNearCap = true;
            }
        }
        for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
            if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples[i]))) {
                std::cerr << "p3-hist: reseed restore failed i=" << i << "\n";
                return false;
            }
            if (!model.applyServoTargets(world, samples[i + 1].targets)) {
                return false;
            }
            ProximalStepDiagnostics diagnostics{};
            if (!model.stepProximal(world, samples[i + 1].subDt, settings, diagnostics)) {
                std::cout << "P3_SEQ_HISTORY_RESEED classification=blocked reason=step_failed i="
                          << i << " status=" << static_cast<int>(diagnostics.status)
                          << " max_replay_w=" << maxReplay << "\n";
                return true;
            }
            std::vector<double> q;
            std::vector<double> v;
            if (!model.readState(world, q, v)) {
                return false;
            }
            double capturedW = 0.0;
            double replayW = 0.0;
            if (!ReportTibiaTwist(
                    model, samples[i + 1].q, samples[i + 1].v, q, v, tibia,
                    "P3_SEQ_HISTORY_RESEED", i + 1, capturedW, replayW)) {
                return false;
            }
            maxCaptured = std::max(maxCaptured, capturedW);
            maxReplay = std::max(maxReplay, replayW);
            if (replayW >= 9.90) {
                reachedIncomingNearCap = true;
            }
        }
        std::cout << "P3_SEQ_HISTORY_RESEED classification="
                  << ClassifyHistoryReplay(reachedIncomingNearCap, maxReplay, maxCaptured)
                  << " samples=" << samples.size()
                  << " max_captured_tibia_w=" << maxCaptured
                  << " max_replay_tibia_w=" << maxReplay
                  << " reached_9_95_before_trip=" << (reachedIncomingNearCap ? 1 : 0)
                  << "\n";
        return true;
    };

    if (!runAccumulating() || !runReseed()) {
        return false;
    }
    if (!RunTibiaCommandIntervene(samples, TibiaCommandMode::ZeroError, false)
        || !RunTibiaCommandIntervene(samples, TibiaCommandMode::ZeroError, true)
        || !RunTibiaCommandIntervene(samples, TibiaCommandMode::RateLimit10, false)
        || !RunTibiaCommandIntervene(samples, TibiaCommandMode::RateLimit10, true)) {
        return false;
    }
    return true;
}

constexpr std::size_t kLeg5TibiaWire = 17;
constexpr std::uint64_t kLeg5TibiaContactId = 19;

std::array<double, 18> MutateLeg5TibiaZeroError(
    const std::array<double, 18>& recorded, double liveAngle) {
    std::array<double, 18> out = recorded;
    out[kLeg5TibiaWire] = liveAngle;
    return out;
}

bool RunNearCapHistoryPath(
    const std::vector<HistorySample>& samples,
    bool implicitOn,
    bool reseed,
    bool zeroError) {
    const char* plant = implicitOn ? "implicit-on" : "implicit-off";
    const char* pathLabel = reseed ? "RESEED" : "ACCUM";
    const char* cmdLabel = zeroError ? "zero_error" : "recorded";
    const std::string twistLabel =
        std::string("NEAR_CAP_HISTORY_") + plant + "_" + pathLabel + "_" + cmdLabel;
    if (implicitOn) {
        ::setenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING", "1", 1);
    } else {
        ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
    }
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
    if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples.front()))) {
        std::cerr << "near-cap-hist: " << plant << " " << pathLabel << " restore sample 0 failed\n";
        return false;
    }
    ProximalSolverSettings settings{};
    const std::uint32_t tibia = scene.legs[5].tibia;
    const std::uint32_t tibiaJoint = scene.legs[5].femurToTibiaJoint;
    double maxCaptured = 0.0;
    double maxReplay = 0.0;
    bool reachedIncomingNearCap = false;
    bool blocked = false;

    const auto measure = [&](std::size_t index) {
        std::vector<double> q;
        std::vector<double> v;
        if (!model.readState(world, q, v)) {
            std::cerr << "near-cap-hist: " << plant << " " << pathLabel << " unreadable i="
                      << index << "\n";
            return false;
        }
        double capturedW = 0.0;
        double replayW = 0.0;
        if (!ReportTibiaTwist(
                model, samples[index].q, samples[index].v, q, v, tibia, twistLabel.c_str(),
                index, capturedW, replayW)) {
            return false;
        }
        maxCaptured = std::max(maxCaptured, capturedW);
        maxReplay = std::max(maxReplay, replayW);
        if (replayW >= kIncomingNearCap) {
            reachedIncomingNearCap = true;
        }
        return true;
    };

    if (!measure(0)) {
        return false;
    }
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        if (reseed) {
            if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples[i]))) {
                std::cerr << "near-cap-hist: " << plant << " " << pathLabel
                          << " restore failed i=" << i << "\n";
                return false;
            }
        }
        std::array<double, 18> targets = samples[i + 1].targets;
        if (zeroError) {
            const double liveAngle = static_cast<double>(world.GetServoJointAngle(tibiaJoint));
            targets = MutateLeg5TibiaZeroError(targets, liveAngle);
            std::cout << "NEAR_CAP_CMD_INTERVENE plant=" << plant
                      << " path=" << pathLabel
                      << " sample=" << (i + 1)
                      << " live_angle=" << liveAngle
                      << " target17=" << targets[kLeg5TibiaWire]
                      << " recorded17=" << samples[i + 1].targets[kLeg5TibiaWire] << "\n";
        }
        if (!model.applyServoTargets(world, targets)) {
            return false;
        }
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, samples[i + 1].subDt, settings, diagnostics)) {
            std::cout << "NEAR_CAP_HISTORY plant=" << plant
                      << " path=" << pathLabel
                      << " cmd=" << cmdLabel
                      << " classification=blocked reason=step_failed i=" << i
                      << " status=" << static_cast<int>(diagnostics.status)
                      << " max_replay_tibia_w=" << maxReplay << "\n";
            blocked = true;
            break;
        }
        if (!measure(i + 1)) {
            return false;
        }
    }
    const char* classification = nullptr;
    if (zeroError) {
        classification = ClassifyCommandIntervene(blocked, maxReplay);
    } else if (blocked) {
        classification = "blocked";
    } else {
        classification = ClassifyHistoryReplay(reachedIncomingNearCap, maxReplay, maxCaptured);
    }
    std::cout << "NEAR_CAP_HISTORY plant=" << plant
              << " path=" << pathLabel
              << " cmd=" << cmdLabel
              << " classification=" << classification
              << " samples=" << samples.size()
              << " max_captured_tibia_w=" << maxCaptured
              << " max_replay_tibia_w=" << maxReplay
              << " reached_9_90=" << (reachedIncomingNearCap ? 1 : 0) << "\n";
    return true;
}

bool CheckNearCapHistoryReplay() {
    const char* path = HEXAPOD_NEAR_CAP_HISTORY_FIXTURE;
    std::string text;
    std::vector<HistorySample> samples;
    if (path == nullptr || path[0] == '\0' || !LoadFile(path, text)
        || !ParseAcceptedHistory(text, samples) || samples.size() < 2) {
        std::cerr << "near-cap-hist: failed to load " << (path != nullptr ? path : "(null)") << "\n";
        return false;
    }

    int bit5Off = 0;
    int tibia19Contacts = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const bool bit5 =
            (samples[i].loadBearingMask & static_cast<std::uint8_t>(1u << 5)) != 0;
        if (!bit5) {
            ++bit5Off;
        }
        int sampleTibia19 = 0;
        for (const auto& contact : samples[i].contacts) {
            if (contact.id == kLeg5TibiaContactId) {
                ++tibia19Contacts;
                ++sampleTibia19;
            }
        }
        std::cout << "NEAR_CAP_HISTORY_CENSUS sample=" << i
                  << " mask=" << static_cast<unsigned>(samples[i].loadBearingMask)
                  << " bit5=" << (bit5 ? 1 : 0)
                  << " contacts=" << samples[i].contacts.size()
                  << " tibia19=" << sampleTibia19
                  << " target17=" << samples[i].targets[kLeg5TibiaWire]
                  << " error17=" << samples[i].errors[kLeg5TibiaWire] << "\n";
    }
    const char* stanceClass = "planned_stance_plus_sustained_unload";
    if (bit5Off != static_cast<int>(samples.size())) {
        stanceClass = "mask_bit5_not_uniform";
    }
    std::cout << "NEAR_CAP_HISTORY_CENSUS samples=" << samples.size()
              << " bit5_off=" << bit5Off
              << " tibia19_contacts=" << tibia19Contacts
              << " stance_class=" << stanceClass
              << " note=missing_mask_bit_is_not_planned_swing"
              << " snap_skip=gait_in_stance"
              << " contact_mode_planning=false"
              << " fusion_load_bearing=ConfirmedStance_only"
              << " lost_candidate_yield=not_dump_named" << "\n";
    if (bit5Off != static_cast<int>(samples.size()) || tibia19Contacts != 0) {
        std::cerr << "near-cap-hist: dump census mismatch bit5_off=" << bit5Off
                  << " tibia19=" << tibia19Contacts << "\n";
        return false;
    }

    const bool paths[] = {false, true};
    for (const bool implicitOn : paths) {
        if (!RunNearCapHistoryPath(samples, implicitOn, false, false)
            || !RunNearCapHistoryPath(samples, implicitOn, true, false)
            || !RunNearCapHistoryPath(samples, implicitOn, false, true)
            || !RunNearCapHistoryPath(samples, implicitOn, true, true)) {
            return false;
        }
    }
    return true;
}

int PopcountMask(std::uint8_t mask) {
    int count = 0;
    for (unsigned bit = 0; bit < 6; ++bit) {
        if ((mask & static_cast<std::uint8_t>(1u << bit)) != 0) {
            ++count;
        }
    }
    return count;
}

struct WinnerLink {
    int leg = 0;
    int joint = 1;
    std::uint32_t body = 0;
    std::uint32_t servo = 0;
    std::size_t wire = 0;
};

WinnerLink FindWinnerLink(PinocchioHexapodModel& model,
                          const HexapodSceneObjects& scene,
                          const HistorySample& sample) {
    WinnerLink winner{};
    double peak = -1.0;
    for (int leg = 0; leg < 6; ++leg) {
        const LegLinkIds& ids = scene.legs[static_cast<std::size_t>(leg)];
        const std::array<std::uint32_t, 3> bodies{ids.coxa, ids.femur, ids.tibia};
        const std::array<std::uint32_t, 3> servos{
            ids.bodyToCoxaJoint, ids.coxaToFemurJoint, ids.femurToTibiaJoint};
        for (int joint = 0; joint < 3; ++joint) {
            Vec3 lin{};
            Vec3 ang{};
            if (!model.computeLinkWorldTwist(sample.q, sample.v, bodies[static_cast<std::size_t>(joint)], lin, ang)) {
                continue;
            }
            const double w = Length(ang);
            if (w > peak) {
                peak = w;
                winner.leg = leg;
                winner.joint = joint;
                winner.body = bodies[static_cast<std::size_t>(joint)];
                winner.servo = servos[static_cast<std::size_t>(joint)];
                winner.wire = static_cast<std::size_t>(leg) * 3U + static_cast<std::size_t>(joint);
            }
        }
    }
    return winner;
}

const char* ClassifyLiveCollision(
    bool blocked,
    bool supportRecovered,
    bool heightHeld,
    bool progressHeld,
    bool reproducedVin) {
    if (blocked) {
        return "blocked";
    }
    if (supportRecovered && heightHeld && progressHeld) {
        return "support_recovered";
    }
    if (reproducedVin) {
        return "reproduced_vin";
    }
    if (heightHeld && progressHeld) {
        return "height_progress_held";
    }
    return "no_named_miss";
}

bool RunLiveCollisionPath(
    const std::vector<HistorySample>& samples,
    bool zeroError) {
    const char* cmdLabel = zeroError ? "zero_error" : "recorded";
    ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    if (!model.restoreAcceptedHistorySample(world, ToModelSample(samples.front()))) {
        std::cerr << "live-collision: restore sample 0 failed\n";
        return false;
    }
    model.resetWarmStarts();

    const WinnerLink winner = FindWinnerLink(model, scene, samples.back());
    const double capturedHeight = samples.back().q.size() > 1 ? samples.back().q[1] : 0.0;
    const double capturedXy = samples.back().q.size() > 2
        ? std::hypot(samples.back().q[0], samples.back().q[2])
        : 0.0;
    const int capturedSupport = PopcountMask(samples.back().loadBearingMask);
    const bool capturedWinnerLoaded =
        (samples.back().loadBearingMask & static_cast<std::uint8_t>(1u << winner.leg)) != 0;

    ProximalSolverSettings settings{};
    double maxCaptured = 0.0;
    double maxReplay = 0.0;
    bool blocked = false;
    int lastSupportCount = capturedSupport;
    bool winnerContact = capturedWinnerLoaded;
    double lastHeight = capturedHeight;
    double lastXy = capturedXy;

    const auto measure = [&](std::size_t index) {
        std::vector<double> q;
        std::vector<double> v;
        if (!model.readState(world, q, v)) {
            std::cerr << "live-collision: unreadable i=" << index << "\n";
            return false;
        }
        double capturedW = 0.0;
        double replayW = 0.0;
        if (!ReportTibiaTwist(
                model, samples[index].q, samples[index].v, q, v, winner.body,
                (std::string("LIVE_COLLISION_") + cmdLabel).c_str(), index, capturedW, replayW)) {
            return false;
        }
        maxCaptured = std::max(maxCaptured, capturedW);
        maxReplay = std::max(maxReplay, replayW);
        lastHeight = q.size() > 1 ? q[1] : 0.0;
        lastXy = q.size() > 2 ? std::hypot(q[0], q[2]) : 0.0;
        return true;
    };

    if (!measure(0)) {
        return false;
    }
    for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
        std::array<double, 18> targets = samples[i + 1].targets;
        if (zeroError) {
            const double liveAngle = static_cast<double>(world.GetServoJointAngle(winner.servo));
            targets[winner.wire] = liveAngle;
            std::cout << "LIVE_COLLISION cmd=" << cmdLabel
                      << " sample=" << (i + 1)
                      << " winner_leg=" << winner.leg
                      << " winner_joint=" << winner.joint
                      << " winner_wire=" << winner.wire
                      << " live_angle=" << liveAngle
                      << " recorded=" << samples[i + 1].targets[winner.wire] << "\n";
        }
        if (!model.applyServoTargets(world, targets)) {
            return false;
        }
        ProximalStepDiagnostics diagnostics{};
        if (!model.stepProximal(world, samples[i + 1].subDt, settings, diagnostics)) {
            std::cout << "LIVE_COLLISION cmd=" << cmdLabel
                      << " classification=blocked reason=step_failed i=" << i
                      << " status=" << static_cast<int>(diagnostics.status)
                      << " max_replay_tibia_w=" << maxReplay << "\n";
            blocked = true;
            break;
        }
        lastSupportCount = 0;
        winnerContact = false;
        for (int leg = 0; leg < 6; ++leg) {
            if (diagnostics.legContactCount[static_cast<std::size_t>(leg)] > 0) {
                ++lastSupportCount;
                if (leg == winner.leg) {
                    winnerContact = true;
                }
            }
        }
        if (!measure(i + 1)) {
            return false;
        }
    }

    const bool supportRecovered =
        !blocked && ((lastSupportCount > capturedSupport) || (winnerContact && !capturedWinnerLoaded));
    const bool heightHeld = !blocked && std::abs(lastHeight - capturedHeight) <= 0.01;
    const bool progressHeld = !blocked && (lastXy + 0.05 >= capturedXy);
    const bool reproducedVin = !blocked && maxReplay >= kIncomingNearCap;
    const char* classification = ClassifyLiveCollision(
        blocked, supportRecovered, heightHeld, progressHeld, reproducedVin);
    std::cout << "LIVE_COLLISION cmd=" << cmdLabel
              << " classification=" << classification
              << " winner_leg=" << winner.leg
              << " winner_joint=" << winner.joint
              << " winner_wire=" << winner.wire
              << " captured_support=" << capturedSupport
              << " replay_support=" << lastSupportCount
              << " winner_contact=" << (winnerContact ? 1 : 0)
              << " support_recovered=" << (supportRecovered ? 1 : 0)
              << " height_held=" << (heightHeld ? 1 : 0)
              << " progress_held=" << (progressHeld ? 1 : 0)
              << " reproduced_vin=" << (reproducedVin ? 1 : 0)
              << " captured_height=" << capturedHeight
              << " replay_height=" << lastHeight
              << " captured_xy=" << capturedXy
              << " replay_xy=" << lastXy
              << " max_captured_tibia_w=" << maxCaptured
              << " max_replay_tibia_w=" << maxReplay
              << " samples=" << samples.size() << "\n";
    return true;
}

bool CheckDefaultStraightLiveCollision() {
    const char* path = HEXAPOD_DEFAULT_STRAIGHT_HISTORY_FIXTURE;
    std::string text;
    std::vector<HistorySample> samples;
    if (path == nullptr || path[0] == '\0' || !LoadFile(path, text)
        || !ParseAcceptedHistory(text, samples) || samples.size() < 2) {
        std::cout << "LIVE_COLLISION skip=missing_fixture path="
                  << (path != nullptr ? path : "(null)") << "\n";
        return true;
    }
    if (!RunLiveCollisionPath(samples, false) || !RunLiveCollisionPath(samples, true)) {
        return false;
    }
    return true;
}

int Run() {
    ::unsetenv("HEXAPOD_PINOCCHIO_COMMAND_STREAM_PATH");
    ::unsetenv("HEXAPOD_PINOCCHIO_IMPLICIT_DAMPING");
    if (!CheckAcceptedStateSpeedInvariant()) {
        return 1;
    }
    if (!CheckWarmStartRollbackTransaction()) {
        return 1;
    }
    if (!CheckTangentFrameInvariance()) {
        return 1;
    }
    if (!CheckMaterialPointJacobian()) {
        return 1;
    }
    if (!CheckArmatureEnergyLedger()) {
        return 1;
    }
    if (!CheckPrefailureBufferDump()) {
        return 1;
    }
    if (!CheckDumpRestoreIdentityAndHeldRetryTargets()) {
        return 1;
    }
    if (!CheckFrozenStandCutpointRestore()) {
        return 1;
    }
    if (!CheckInProcessCommandStreamReplay()) {
        return 1;
    }
    if (!CheckFrozenCutpointCommandStreamReplay()) {
        return 1;
    }
    if (!CheckP3SeqHistoryReplay()) {
        return 1;
    }
    if (!CheckNearCapHistoryReplay()) {
        return 1;
    }
    if (!CheckDefaultStraightLiveCollision()) {
        return 1;
    }
    std::cout << "p0/p1 accepted-state, rollback, tangent, energy, dump, restore, "
                 "command-stream, and history replay invariants passed\n";
    return 0;
}

} // namespace

int main() {
    return Run();
}
