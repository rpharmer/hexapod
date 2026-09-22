// Restore a P5 turn-entry stand_cutpoint and dump it again; q/v/warm_start_count
// must match. Skip missing_fixture until P5-0 capture lands.

#include "minphys3d/demo/hexapod_scene.hpp"
#include "minphys3d/demo/pinocchio_hexapod.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef HEXAPOD_P5_TURN_ENTRY_CUTPOINT_ISOLATED
#define HEXAPOD_P5_TURN_ENTRY_CUTPOINT_ISOLATED ""
#endif
#ifndef HEXAPOD_P5_TURN_ENTRY_CUTPOINT_SEQUENTIAL
#define HEXAPOD_P5_TURN_ENTRY_CUTPOINT_SEQUENTIAL ""
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

bool loadFile(const char* path, std::string& out) {
    std::ifstream in(path, std::ios::in | std::ios::binary);
    if (!in) {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return !out.empty();
}

bool extractArray(const std::string& text, const char* key, std::vector<double>& out) {
    const std::string needle = std::string("\"") + key + "\":[";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    out.clear();
    const char* cursor = text.c_str() + pos + needle.size();
    while (*cursor != '\0' && *cursor != ']') {
        while (*cursor == ' ' || *cursor == ',') {
            ++cursor;
        }
        if (*cursor == ']') {
            break;
        }
        char* end = nullptr;
        const double value = std::strtod(cursor, &end);
        if (end == cursor) {
            return false;
        }
        out.push_back(value);
        cursor = end;
    }
    return !out.empty();
}

bool extractNumber(const std::string& text, const char* key, double& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    char* end = nullptr;
    out = std::strtod(text.c_str() + pos + needle.size(), &end);
    return end != text.c_str() + pos + needle.size() && std::isfinite(out);
}

double maxAbsDiff(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size() || a.empty()) {
        return 1.0e9;
    }
    double peak = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        peak = std::max(peak, std::abs(a[i] - b[i]));
    }
    return peak;
}

bool fileExists(const char* path) {
    if (path == nullptr || path[0] == '\0') {
        return false;
    }
    std::ifstream in(path);
    return in.good();
}

bool checkOne(const char* tag, const char* path) {
    if (!fileExists(path)) {
        std::cout << "P5_TURN_ENTRY_IDENTITY tag=" << tag
                  << " classification=missing_fixture path=" << (path ? path : "") << '\n';
        return true;
    }
    World world({0.0, -9.80665, 0.0});
    const HexapodSceneObjects scene = BuildHexapodScene(world);
    PinocchioHexapodModel model(world, scene, SceneJointOrder(scene));
    ProximalSolverSettings settings{};
    for (int step = 0; step < 30; ++step) {
        ProximalStepDiagnostics diagnostics{};
        (void)model.stepProximal(world, 1.0 / 480.0, settings, diagnostics);
    }
    if (!model.restoreDiagnosticCutpoint(world, path)) {
        std::cerr << "P5_TURN_ENTRY_IDENTITY tag=" << tag << " restore failed\n";
        return false;
    }
    const char* tmp = "/tmp/hexapod-p5-turn-entry-identity-roundtrip.json";
    std::remove(tmp);
    if (!model.dumpDiagnosticCutpoint(tmp)) {
        std::cerr << "P5_TURN_ENTRY_IDENTITY tag=" << tag << " redump failed\n";
        return false;
    }
    std::string original;
    std::string roundtrip;
    if (!loadFile(path, original) || !loadFile(tmp, roundtrip)) {
        std::cerr << "P5_TURN_ENTRY_IDENTITY tag=" << tag << " load failed\n";
        return false;
    }
    std::vector<double> q0;
    std::vector<double> v0;
    std::vector<double> q1;
    std::vector<double> v1;
    double warm0 = 0.0;
    double warm1 = 0.0;
    if (!extractArray(original, "q", q0) || !extractArray(original, "v", v0)
        || !extractArray(roundtrip, "q", q1) || !extractArray(roundtrip, "v", v1)
        || !extractNumber(original, "warm_start_count", warm0)
        || !extractNumber(roundtrip, "warm_start_count", warm1)) {
        std::cerr << "P5_TURN_ENTRY_IDENTITY tag=" << tag << " parse failed\n";
        return false;
    }
    const double dq = maxAbsDiff(q0, q1);
    const double dv = maxAbsDiff(v0, v1);
    const bool ok = dq < 1.0e-9 && dv < 1.0e-9 && std::abs(warm0 - warm1) < 0.5;
    std::cout << "P5_TURN_ENTRY_IDENTITY tag=" << tag
              << " classification=" << (ok ? "identity" : "mismatch")
              << " dq=" << dq << " dv=" << dv
              << " warm0=" << warm0 << " warm1=" << warm1
              << " path=" << path << '\n';
    return ok;
}

} // namespace

int main() {
    const bool isolated_ok = checkOne("isolated", HEXAPOD_P5_TURN_ENTRY_CUTPOINT_ISOLATED);
    const bool sequential_ok = checkOne("sequential", HEXAPOD_P5_TURN_ENTRY_CUTPOINT_SEQUENTIAL);
    return (isolated_ok && sequential_ok) ? EXIT_SUCCESS : EXIT_FAILURE;
}
