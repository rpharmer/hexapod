#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include "hexapod_dynamics_constants.hpp"

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

#ifndef HEXAPOD_P3_ABA_OVER_FIXTURE
#define HEXAPOD_P3_ABA_OVER_FIXTURE ""
#endif
#ifndef HEXAPOD_P3_SEQ_ABA_OVER_FIXTURE
#define HEXAPOD_P3_SEQ_ABA_OVER_FIXTURE ""
#endif
#ifndef HEXAPOD_P3_NEAR_CAP_FIXTURE
#define HEXAPOD_P3_NEAR_CAP_FIXTURE ""
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

bool ParseJsonNumberArray(const std::string& text, std::size_t& i, std::vector<double>& out) {
    while (i < text.size() && text[i] != '[') {
        ++i;
    }
    if (i >= text.size()) {
        return false;
    }
    ++i;
    while (i < text.size()) {
        while (i < text.size()
               && (text[i] == ' ' || text[i] == '\n' || text[i] == '\r' || text[i] == '\t'
                   || text[i] == ',')) {
            ++i;
        }
        if (i < text.size() && text[i] == ']') {
            ++i;
            return true;
        }
        if (i < text.size() && text[i] == '[') {
            if (!ParseJsonNumberArray(text, i, out)) {
                return false;
            }
            continue;
        }
        char* end = nullptr;
        const double value = std::strtod(text.c_str() + i, &end);
        if (end == text.c_str() + i) {
            return false;
        }
        out.push_back(value);
        i = static_cast<std::size_t>(end - text.c_str());
    }
    return false;
}

bool ExtractArray(const std::string& text, const char* key, std::size_t from, std::vector<double>& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    out.clear();
    return ParseJsonNumberArray(text, i, out);
}

bool ExtractNumber(const std::string& text, const char* key, std::size_t from, double& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str() + pos + needle.size(), &end);
    return end != text.c_str() + pos + needle.size() && std::isfinite(out);
}

bool ExtractQuoted(const std::string& text, const char* key, std::size_t from, std::string& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    while (i < text.size() && (text[i] == ' ' || text[i] == '\n' || text[i] == '\r' || text[i] == '\t')) {
        ++i;
    }
    if (i >= text.size() || text[i] != '"') {
        return false;
    }
    ++i;
    out.clear();
    while (i < text.size() && text[i] != '"') {
        if (text[i] == '\\' && i + 1 < text.size()) {
            out.push_back(text[i + 1]);
            i += 2;
            continue;
        }
        out.push_back(text[i++]);
    }
    return i < text.size();
}

std::vector<double> ExtractRepeatedNumbers(
    const std::string& text,
    const char* key,
    std::size_t from,
    std::size_t until,
    std::size_t count) {
    const std::string needle = std::string("\"") + key + "\":";
    std::vector<double> out;
    std::size_t pos = from;
    while (out.size() < count) {
        pos = text.find(needle, pos);
        if (pos == std::string::npos || pos >= until) {
            break;
        }
        pos += needle.size();
        char* end = nullptr;
        out.push_back(std::strtod(text.c_str() + pos, &end));
        pos = static_cast<std::size_t>(end - text.c_str());
    }
    return out;
}

std::string LoadText(const char* path) {
    std::ifstream in(path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

double VectorInfRel(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size() || a.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double num = 0.0;
    double den = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        num = std::max(num, std::abs(a[i] - b[i]));
        den = std::max(den, std::abs(b[i]));
    }
    return num / std::max(1.0e-16, den);
}

bool CheckUnsaturatedFrozenQ() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 120; ++step) {
        ProximalStepDiagnostics diagnostics{};
        (void)model.stepProximal(world, 1.0 / 480.0, settings, diagnostics);
    }
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        std::cerr << "p3 unsaturated: failed read\n";
        return false;
    }
    for (std::size_t i = 6; i < v.size(); ++i) {
        v[i] = (i % 2 == 0) ? 0.35 : -0.22;
    }
    const std::array<double, 18> inertias = model.servoNominalInertias();
    std::array<double, 18> errors{};
    errors.fill(0.02);
    PinocchioHexapodModel::ImplicitDampingOracleResult result{};
    const double dt = 1.0 / 600.0;
    if (!model.computeImplicitDampingOracle(q, v, errors, inertias, dt, result)
        || result.ldltRelativeError > 1.0e-8
        || !result.unsaturated
        || !result.envelopeHoldsAtVin
        || !result.envelopeHoldsAtVfree
        || result.vFree.size() != v.size()
        || result.explicitVfree.size() != v.size()) {
        std::cerr << "p3 unsaturated: oracle failed ldlt=" << result.ldltRelativeError
                  << " unsaturated=" << result.unsaturated << "\n";
        return false;
    }
    if (VectorInfRel(result.vFree, result.explicitVfree) < 1.0e-12) {
        std::cerr << "p3 unsaturated: implicit matched explicit ABA with D v != 0\n";
        return false;
    }
    std::uint32_t tibia = scene.legs[0].tibia;
    std::vector<double> jacobian;
    if (!model.computeLinkAngularJacobian(q, tibia, jacobian) || jacobian.size() != 3U * v.size()) {
        std::cerr << "p3 unsaturated: tibia Jacobian missing\n";
        return false;
    }
    PinocchioHexapodModel::ImplicitDampingOracleResult withJ{};
    if (!model.computeImplicitDampingOracle(
            q, v, errors, inertias, dt, withJ, 1.0, &jacobian, 3)
        || withJ.maxDelassusAbsDiff <= 1.0e-16) {
        std::cerr << "p3 unsaturated: G_H did not differ from G_M diff="
                  << withJ.maxDelassusAbsDiff << "\n";
        return false;
    }
    std::cout << "p3 unsaturated frozen-q ldlt_rel=" << result.ldltRelativeError
              << " aba_rel=" << result.explicitAbaRelativeError
              << " delassus_diff=" << withJ.maxDelassusAbsDiff << "\n";
    return true;
}

bool CheckSaturatedEnvelope() {
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 30; ++step) {
        ProximalStepDiagnostics diagnostics{};
        (void)model.stepProximal(world, 1.0 / 480.0, settings, diagnostics);
    }
    std::vector<double> q;
    std::vector<double> v;
    if (!model.readState(world, q, v)) {
        return false;
    }
    std::fill(v.begin(), v.end(), 0.0);
    const std::array<double, 18> inertias = model.servoNominalInertias();
    std::array<double, 18> errors{};
    errors.fill(50.0);
    PinocchioHexapodModel::ImplicitDampingOracleResult result{};
    if (!model.computeImplicitDampingOracle(q, v, errors, inertias, 1.0 / 600.0, result)
        || result.tauChosen.size() != v.size()) {
        std::cerr << "p3 saturated: oracle failed\n";
        return false;
    }
    if (result.unsaturated) {
        double minI = inertias[0];
        double maxTauP = 0.0;
        for (std::size_t i = 0; i < 18; ++i) {
            minI = std::min(minI, inertias[i]);
            maxTauP = std::max(maxTauP, std::abs(result.positionTorque[i]));
        }
        std::cerr << "p3 saturated: still unsaturated minI=" << minI << " maxTauP=" << maxTauP
                  << " envelope_vin=" << result.envelopeHoldsAtVin
                  << " envelope_vfree=" << result.envelopeHoldsAtVfree << "\n";
        return false;
    }
    if (!result.envelopeHoldsAtVin || !result.envelopeHoldsAtVfree) {
        std::cerr << "p3 saturated: clipped torque outside envelope vin="
                  << result.envelopeHoldsAtVin << " vfree=" << result.envelopeHoldsAtVfree << "\n";
        return false;
    }
    std::size_t clipped = 0;
    for (std::size_t i = 6; i < result.tauChosen.size(); ++i) {
        if (result.dampingDiag[i] != 0.0) {
            continue;
        }
        ++clipped;
        if (std::abs(result.tauChosen[i]) > result.availableAtVin[i] + 1.0e-9
            && std::abs(result.tauChosen[i]) > result.availableAtVfree[i] + 1.0e-9) {
            std::cerr << "p3 saturated: tauChosen outside envelopes i=" << i << " tau="
                      << result.tauChosen[i] << "\n";
            return false;
        }
    }
    if (clipped == 0) {
        std::cerr << "p3 saturated: no joint froze D\n";
        return false;
    }
    std::cout << "p3 saturated envelope holds at v_in and v_free, D frozen on clips\n";
    return true;
}

struct FixtureReplayReport {
    std::string winner;
    double speedIn = 0.0;
    double speedCapturedFree = 0.0;
    double speedExplicitAba = 0.0;
    double speedImplicit = 0.0;
    double capturedVsHRel = 0.0;
    double capturedVsExplicitRel = 0.0;
    bool unsaturated = true;
    bool envelopeVin = true;
    bool envelopeVfree = true;
    bool unlimitedBrake = false;
    bool legal = false;
    bool underCap = false;
    bool keepProduction = false;
    std::string classification;
};

bool ReplaySpeedLimitFixture(
    const char* path,
    const char* label,
    bool requireKeepUnderCap,
    FixtureReplayReport& report,
    void (*mutateErrors)(std::array<double, 18>&) = nullptr) {
    report = {};
    if (path == nullptr || path[0] == '\0') {
        std::cerr << label << ": compile-time path missing\n";
        return false;
    }
    const std::string text = LoadText(path);
    if (text.find("\"kind\":\"speed_limit\"") == std::string::npos) {
        std::cerr << label << ": failed to load " << path << "\n";
        return false;
    }
    const auto winnerPos = text.find("\"winner\":");
    if (winnerPos == std::string::npos || !ExtractQuoted(text, "frame", winnerPos, report.winner)
        || report.winner.empty()) {
        std::cerr << label << ": winner frame missing\n";
        return false;
    }
    const auto kin = text.find("\"kinematics\":");
    const auto history = text.find("\"accepted_history\":");
    if (kin == std::string::npos || history == std::string::npos || history <= kin) {
        std::cerr << label << ": missing kinematics\n";
        return false;
    }
    std::vector<double> q;
    std::vector<double> vIn;
    std::vector<double> vFreeCaptured;
    std::vector<double> angular;
    double dt = 0.0;
    double cap = 10.0;
    if (!ExtractArray(text, "q", kin, q) || q.size() != 25
        || !ExtractArray(text, "v_in", kin, vIn) || vIn.size() != 24
        || !ExtractArray(text, "v_free", kin, vFreeCaptured) || vFreeCaptured.size() != 24
        || !ExtractNumber(text, "dt", 0, dt)
        || !ExtractNumber(text, "max_angular_speed", 0, cap)) {
        std::cerr << label << ": failed q/v/dt parse\n";
        return false;
    }
    const auto wires = text.find("\"wires\":", kin);
    const auto links = text.find("\"links\":", kin);
    if (wires == std::string::npos || links == std::string::npos || links <= wires) {
        std::cerr << label << ": missing wires\n";
        return false;
    }
    const std::vector<double> errors = ExtractRepeatedNumbers(text, "error", wires, links, 18);
    const std::vector<double> inertias =
        ExtractRepeatedNumbers(text, "effective_inertia", wires, links, 18);
    if (errors.size() != 18 || inertias.size() != 18) {
        std::cerr << label << ": wire error/inertia count\n";
        return false;
    }
    std::array<double, 18> servoErrors{};
    std::array<double, 18> servoInertias{};
    for (std::size_t i = 0; i < 18; ++i) {
        servoErrors[i] = errors[i];
        servoInertias[i] = inertias[i];
    }
    if (mutateErrors != nullptr) {
        mutateErrors(servoErrors);
    }
    const std::string nameNeedle = std::string("\"name\":\"") + report.winner + "\"";
    const auto winnerName = text.find(nameNeedle, kin);
    double bodyIdValue = 0.0;
    if (winnerName == std::string::npos
        || !ExtractNumber(text, "body_id", winnerName, bodyIdValue)
        || !ExtractArray(text, "angular_jacobian", winnerName, angular)
        || angular.size() != 72) {
        std::cerr << label << ": winner Jacobian missing for " << report.winner << "\n";
        return false;
    }
    std::vector<double> jacobianColMajor(72);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 24; ++col) {
            jacobianColMajor[static_cast<std::size_t>(col * 3 + row)] =
                angular[static_cast<std::size_t>(row * 24 + col)];
        }
    }

    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    PinocchioHexapodModel::ImplicitDampingOracleResult result{};
    if (!model.computeImplicitDampingOracle(
            q,
            vIn,
            servoErrors,
            servoInertias,
            dt,
            result,
            1.0,
            &jacobianColMajor,
            3)
        || result.ldltRelativeError > 1.0e-8) {
        std::cerr << label << ": oracle reconstruct failed ldlt=" << result.ldltRelativeError
                  << "\n";
        return false;
    }
    if (result.maxDelassusAbsDiff <= 1.0e-16) {
        std::cerr << label << ": G_H == G_M on captured J\n";
        return false;
    }
    if (result.explicitVfree.size() != vIn.size() || result.vFree.size() != vIn.size()) {
        std::cerr << label << ": oracle velocity size\n";
        return false;
    }

    auto angularSpeed = [&](const std::vector<double>& velocity) {
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        for (int col = 0; col < 24; ++col) {
            wx += jacobianColMajor[static_cast<std::size_t>(col * 3 + 0)]
                * velocity[static_cast<std::size_t>(col)];
            wy += jacobianColMajor[static_cast<std::size_t>(col * 3 + 1)]
                * velocity[static_cast<std::size_t>(col)];
            wz += jacobianColMajor[static_cast<std::size_t>(col * 3 + 2)]
                * velocity[static_cast<std::size_t>(col)];
        }
        return std::sqrt(wx * wx + wy * wy + wz * wz);
    };
    report.speedIn = angularSpeed(vIn);
    report.speedCapturedFree = angularSpeed(vFreeCaptured);
    report.speedExplicitAba = angularSpeed(result.explicitVfree);
    report.speedImplicit = angularSpeed(result.vFree);
    report.capturedVsHRel = VectorInfRel(vFreeCaptured, result.vFree);
    report.capturedVsExplicitRel = VectorInfRel(vFreeCaptured, result.explicitVfree);
    report.unsaturated = result.unsaturated;
    report.envelopeVin = result.envelopeHoldsAtVin;
    report.envelopeVfree = result.envelopeHoldsAtVfree;
    report.unlimitedBrake = result.usedUnlimitedDampingBrake;
    Vec3 linear{};
    Vec3 angularWorld{};
    const auto winnerBody = static_cast<std::uint32_t>(bodyIdValue);
    if (!model.computeLinkWorldTwist(q, result.vFree, winnerBody, linear, angularWorld)) {
        std::cerr << label << ": twist reconstruct failed\n";
        return false;
    }
    const double twistSpeed = Length(angularWorld);
    if (std::abs(twistSpeed - report.speedImplicit) > 1.0e-8) {
        std::cerr << label << ": Jacobian speed " << report.speedImplicit << " vs twist "
                  << twistSpeed << "\n";
        return false;
    }

    report.legal =
        result.envelopeHoldsAtVin && result.envelopeHoldsAtVfree && !result.usedUnlimitedDampingBrake;
    report.underCap = report.speedImplicit < cap;
    report.keepProduction = report.legal && report.underCap;
    if (!report.legal && report.underCap) {
        std::cerr << label << ": under-cap only via envelope-violating damping\n";
        report.keepProduction = false;
        report.classification = "reject_d_as_brake";
    } else if (report.keepProduction) {
        report.classification = "keep_under_cap";
    } else if (!result.unsaturated && !report.underCap) {
        report.classification = "saturation_miss";
    } else if (report.capturedVsHRel < 1.0e-6 && !report.underCap && report.legal) {
        report.classification = "legal_miss";
    } else if (report.capturedVsExplicitRel < 1.0e-6 && report.capturedVsHRel >= 1.0e-6) {
        report.classification = "production_oracle_mismatch";
    } else if (report.legal && !report.underCap) {
        report.classification = "legal_miss";
    } else {
        report.classification = "unclassified";
    }

    if (requireKeepUnderCap && !report.keepProduction) {
        std::cerr << label << ": expected legal under-cap keep\n";
        return false;
    }

    std::cout << label << " winner=" << report.winner
              << " speed_in=" << report.speedIn
              << " speed_captured_free=" << report.speedCapturedFree
              << " speed_explicit_aba=" << report.speedExplicitAba
              << " speed_implicit_free=" << report.speedImplicit
              << " cap=" << cap
              << " captured_vs_H_rel=" << report.capturedVsHRel
              << " captured_vs_explicit_rel=" << report.capturedVsExplicitRel
              << " unsaturated=" << result.unsaturated
              << " envelope_vin=" << result.envelopeHoldsAtVin
              << " envelope_vfree=" << result.envelopeHoldsAtVfree
              << " unlimited_envelope=" << result.unlimitedEnvelopeHolds
              << " unlimited_brake=" << result.usedUnlimitedDampingBrake
              << " delassus_diff=" << result.maxDelassusAbsDiff
              << " work=" << result.actuatorWork
              << " keep_production=" << (report.keepProduction ? 1 : 0)
              << " classification=" << report.classification << "\n";
    return true;
}

bool CheckFixtureReplay(bool& keepProduction) {
    FixtureReplayReport report{};
    if (!ReplaySpeedLimitFixture(HEXAPOD_P3_ABA_OVER_FIXTURE, "p3 fixture", true, report)) {
        return false;
    }
    if (report.winner != "leg_0_tibia_body") {
        std::cerr << "p3 fixture: expected winner leg_0_tibia_body got " << report.winner << "\n";
        return false;
    }
    keepProduction = report.keepProduction;
    return true;
}

bool CheckSeqFixtureReplay(FixtureReplayReport& report) {
    if (!ReplaySpeedLimitFixture(
            HEXAPOD_P3_SEQ_ABA_OVER_FIXTURE, "p3 seq fixture", false, report)) {
        return false;
    }
    if (report.winner != "leg_2_tibia_body") {
        std::cerr << "p3 seq fixture: expected winner leg_2_tibia_body got " << report.winner
                  << "\n";
        return false;
    }
    if (report.classification == "reject_d_as_brake") {
        std::cerr << "p3 seq fixture: under-cap only via D-as-brake; reject production change\n";
        return false;
    }
    std::cout << "P3_SEQ_CLASSIFICATION=" << report.classification << "\n";
    return true;
}

void ZeroWire17(std::array<double, 18>& errors) {
    errors[17] = 0.0;
}

void ZeroWire16(std::array<double, 18>& errors) {
    errors[16] = 0.0;
}

void ZeroOthersKeepWire17(std::array<double, 18>& errors) {
    for (std::size_t i = 0; i < 18; ++i) {
        if (i != 17) {
            errors[i] = 0.0;
        }
    }
}

const char* SpeedClass(double speed, double cap) {
    return speed < cap ? "under_cap" : "still_over";
}

bool CheckNearCapFixtureReplay(FixtureReplayReport& report) {
    if (!ReplaySpeedLimitFixture(
            HEXAPOD_P3_NEAR_CAP_FIXTURE, "p3 near-cap fixture", false, report)) {
        return false;
    }
    if (report.winner != "leg_5_tibia_body") {
        std::cerr << "p3 near-cap fixture: expected winner leg_5_tibia_body got " << report.winner
                  << "\n";
        return false;
    }
    if (report.classification == "reject_d_as_brake") {
        std::cerr << "p3 near-cap fixture: under-cap only via D-as-brake\n";
        return false;
    }
    std::cout << "P3_NEAR_CAP_CLASSIFICATION=" << report.classification
              << " legal_under_cap=" << (report.keepProduction ? 1 : 0)
              << " legal_miss=" << ((!report.keepProduction && report.legal) ? 1 : 0) << "\n";
    return true;
}

bool CheckNearCapKnockouts() {
    FixtureReplayReport identity{};
    FixtureReplayReport zero17{};
    FixtureReplayReport zero16{};
    FixtureReplayReport only17{};
    if (!ReplaySpeedLimitFixture(
            HEXAPOD_P3_NEAR_CAP_FIXTURE, "near-cap knockout identity", false, identity)
        || !ReplaySpeedLimitFixture(
            HEXAPOD_P3_NEAR_CAP_FIXTURE, "near-cap knockout zero_wire17", false, zero17, ZeroWire17)
        || !ReplaySpeedLimitFixture(
            HEXAPOD_P3_NEAR_CAP_FIXTURE, "near-cap knockout zero_wire16", false, zero16, ZeroWire16)
        || !ReplaySpeedLimitFixture(
            HEXAPOD_P3_NEAR_CAP_FIXTURE,
            "near-cap knockout only_wire17",
            false,
            only17,
            ZeroOthersKeepWire17)) {
        return false;
    }
    constexpr double kCap = 10.0;
    const char* c17 = SpeedClass(zero17.speedExplicitAba, kCap);
    const char* c16 = SpeedClass(zero16.speedExplicitAba, kCap);
    const char* cOnly = SpeedClass(only17.speedExplicitAba, kCap);
    std::cout << "NEAR_CAP_KNOCKOUT identity_explicit=" << identity.speedExplicitAba
              << " identity_implicit=" << identity.speedImplicit
              << " zero_wire17_explicit=" << zero17.speedExplicitAba << " " << c17
              << " zero_wire16_explicit=" << zero16.speedExplicitAba << " " << c16
              << " only_wire17_explicit=" << only17.speedExplicitAba << " " << cOnly << "\n";
    if (!(zero17.speedExplicitAba < identity.speedExplicitAba)) {
        std::cerr << "near-cap knockout: zero wire 17 must lower explicit ABA speed\n";
        return false;
    }
    if (zero17.speedExplicitAba >= kCap) {
        std::cerr << "near-cap knockout: zero wire 17 still_over on explicit ABA\n";
        return false;
    }
    if (zero16.speedExplicitAba < kCap) {
        std::cerr << "near-cap knockout: zero wire 16 unexpectedly under_cap\n";
        return false;
    }
    return true;
}

double MotorEnvelopeAvailableHost(
    const double requested,
    const double velocity,
    const double stallTorque,
    const double noLoadSpeed) {
    double available = stallTorque;
    if (requested * velocity > 0.0) {
        available *= std::max(0.0, 1.0 - std::abs(velocity) / noLoadSpeed);
    }
    return available;
}

bool AuditFinalContactTorque(const char* path, const char* label) {
    if (path == nullptr || path[0] == '\0') {
        std::cerr << label << ": compile-time path missing\n";
        return false;
    }
    const std::string text = LoadText(path);
    if (text.find("\"kind\":\"speed_limit\"") == std::string::npos) {
        std::cerr << label << ": failed to load " << path << "\n";
        return false;
    }
    const auto kin = text.find("\"kinematics\":");
    const auto history = text.find("\"accepted_history\":");
    if (kin == std::string::npos || history == std::string::npos || history <= kin) {
        std::cerr << label << ": missing kinematics\n";
        return false;
    }
    std::vector<double> q;
    std::vector<double> vIn;
    std::vector<double> vAfter;
    double dt = 0.0;
    double noLoad = hexapod_dynamics::kServoNoLoadSpeedRadPerSec;
    if (!ExtractArray(text, "q", kin, q) || q.size() != 25
        || !ExtractArray(text, "v_in", kin, vIn) || vIn.size() != 24
        || !ExtractArray(text, "v_after", kin, vAfter) || vAfter.size() != 24
        || !ExtractNumber(text, "dt", 0, dt)) {
        std::cerr << label << ": failed q/v_after/dt parse\n";
        return false;
    }
    (void)ExtractNumber(text, "no_load_speed", 0, noLoad);
    const auto wires = text.find("\"wires\":", kin);
    const auto links = text.find("\"links\":", kin);
    if (wires == std::string::npos || links == std::string::npos || links <= wires) {
        std::cerr << label << ": missing wires\n";
        return false;
    }
    const std::vector<double> errors = ExtractRepeatedNumbers(text, "error", wires, links, 18);
    const std::vector<double> inertias =
        ExtractRepeatedNumbers(text, "effective_inertia", wires, links, 18);
    if (errors.size() != 18 || inertias.size() != 18) {
        std::cerr << label << ": wire error/inertia count\n";
        return false;
    }
    std::array<double, 18> servoErrors{};
    std::array<double, 18> servoInertias{};
    for (std::size_t i = 0; i < 18; ++i) {
        servoErrors[i] = errors[i];
        servoInertias[i] = inertias[i];
    }

    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    PinocchioHexapodModel::ImplicitDampingOracleResult result{};
    if (!model.computeImplicitDampingOracle(q, vIn, servoErrors, servoInertias, dt, result)
        || result.ldltRelativeError > 1.0e-8
        || result.tauChosen.size() != vAfter.size()
        || result.positionTorque.size() != vAfter.size()
        || result.dampingDiag.size() != vAfter.size()) {
        std::cerr << label << ": oracle reconstruct failed\n";
        return false;
    }

    const double stallTorque = hexapod_dynamics::kServoMaxTorqueNm;
    bool envelopeMiss = false;
    bool branchWouldChange = false;
    double workAfter = 0.0;
    int unsaturatedWires = 0;
    int frozenWires = 0;
    for (std::size_t wire = 0; wire < 18; ++wire) {
        const std::size_t vi = 6 + wire;
        const double damping = result.dampingDiag[vi];
        const double tauP = result.positionTorque[vi];
        const double tauFree = result.tauChosen[vi];
        const double velAfter = vAfter[vi];
        const bool frozen = std::abs(damping) <= 1.0e-18;
        if (frozen) {
            ++frozenWires;
        } else {
            ++unsaturatedWires;
        }
        const double tauAfter = frozen ? tauFree : (tauP - damping * velAfter);
        const double availableAfter =
            MotorEnvelopeAvailableHost(tauAfter, velAfter, stallTorque, noLoad);
        if (std::abs(tauAfter) > availableAfter + 1.0e-12) {
            envelopeMiss = true;
        }
        if (!frozen) {
            const double unsaturatedAfter = tauP - damping * velAfter;
            const double availableUnsaturated =
                MotorEnvelopeAvailableHost(unsaturatedAfter, velAfter, stallTorque, noLoad);
            if (std::abs(unsaturatedAfter) > availableUnsaturated + 1.0e-12) {
                branchWouldChange = true;
            }
        }
        workAfter += tauAfter * velAfter * dt;
    }
    const bool workInconsistent =
        (workAfter * result.actuatorWork) < 0.0
        && (std::abs(workAfter) + std::abs(result.actuatorWork)) > 1.0e-12;
    const char* classification = "ok";
    if (envelopeMiss) {
        classification = "envelope_miss_after_contact";
    } else if (branchWouldChange) {
        classification = "branch_would_change";
    } else if (workInconsistent) {
        classification = "work_inconsistent";
    }
    const bool named =
        std::string(classification) == "envelope_miss_after_contact"
        || std::string(classification) == "branch_would_change";
    std::cout << "FINAL_CONTACT_TORQUE dump=" << label
              << " class=" << classification
              << " envelope_miss=" << (envelopeMiss ? 1 : 0)
              << " branch_would_change=" << (branchWouldChange ? 1 : 0)
              << " work_inconsistent=" << (workInconsistent ? 1 : 0)
              << " named_miss=" << (named ? 1 : 0)
              << " unsaturated_wires=" << unsaturatedWires
              << " frozen_wires=" << frozenWires
              << " work_free=" << result.actuatorWork
              << " work_after=" << workAfter
              << " envelope_vfree=" << result.envelopeHoldsAtVfree
              << "\n";
    return true;
}

bool CheckFinalContactTorqueAudit() {
    if (!AuditFinalContactTorque(HEXAPOD_P3_NEAR_CAP_FIXTURE, "sl-abort-near-cap-v1")
        || !AuditFinalContactTorque(HEXAPOD_P3_SEQ_ABA_OVER_FIXTURE, "p3-seq-first-trip-buffer")) {
        return false;
    }
    return true;
}

int Run() {
    bool keepProduction = false;
    FixtureReplayReport seq{};
    if (!CheckUnsaturatedFrozenQ()) {
        return 1;
    }
    if (!CheckSaturatedEnvelope()) {
        return 1;
    }
    if (!CheckFixtureReplay(keepProduction)) {
        return 1;
    }
    if (!CheckSeqFixtureReplay(seq)) {
        return 1;
    }
    FixtureReplayReport nearCap{};
    if (!CheckNearCapFixtureReplay(nearCap)) {
        return 1;
    }
    if (!CheckNearCapKnockouts()) {
        return 1;
    }
    if (!CheckFinalContactTorqueAudit()) {
        return 1;
    }
    std::cout << "P3_KEEP_PRODUCTION=" << (keepProduction ? 1 : 0) << "\n";
    std::cout << "p3 implicit-damping oracle, envelope, fixture, sequential miss, near-cap, and G_H tests passed\n";
    return 0;
}

} // namespace

int main() {
    return Run();
}
