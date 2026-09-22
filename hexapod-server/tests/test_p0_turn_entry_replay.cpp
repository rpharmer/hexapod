// Parse turn-entry dumps: isolated vs sequential-pass / sequential-fail.
// Pass vs isolated → entry_pose|entry_stance|entry_match|unknown.
// Fail vs pass first-WALK → more_extreme|similar|milder.

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#ifndef HEXAPOD_P0_TURN_ENTRY_ISOLATED_FIXTURE
#define HEXAPOD_P0_TURN_ENTRY_ISOLATED_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FIXTURE
#define HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FAIL_FIXTURE
#define HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FAIL_FIXTURE ""
#endif

namespace {

constexpr int kNumLegs = 6;

struct Snapshot {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    int support{0};
    double stance_width_m{0.0};
    double foot_centroid_x{0.0};
    double foot_centroid_y{0.0};
    double body_to_centroid_x{0.0};
    double body_to_centroid_y{0.0};
    std::array<double, kNumLegs> foot_body_x{};
    std::array<double, kNumLegs> foot_body_y{};
    std::array<int, kNumLegs> fused_support{};
    bool valid{false};
};

struct Dump {
    double net{0.0};
    bool have_net{false};
    double held{0.0};
    double cmd_yaw{0.0};
    double start_x{0.0};
    double start_y{0.0};
    Snapshot stand_end{};
    Snapshot first_walk{};
};

struct Delta {
    double dxy{0.0};
    double dz{0.0};
    double dtilt{0.0};
    double dyaw{0.0};
    double dspeed{0.0};
    double dwz{0.0};
    int dsupport{0};
    double dwidth{0.0};
    double dcentroid_body{0.0};
    double foot_body_rmse{0.0};
};

struct Classified {
    std::string rule{"unknown"};
    Delta stand{};
    Delta first{};
};

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

bool extractObject(const std::string& text, const char* key, std::string& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    while (i < text.size() && (text[i] == ' ' || text[i] == '\n')) {
        ++i;
    }
    if (i >= text.size() || text[i] != '{') {
        return false;
    }
    int depth = 0;
    const std::size_t start = i;
    for (; i < text.size(); ++i) {
        if (text[i] == '{') {
            ++depth;
        } else if (text[i] == '}') {
            --depth;
            if (depth == 0) {
                out = text.substr(start, i - start + 1);
                return true;
            }
        }
    }
    return false;
}

bool parseSnapshot(const std::string& obj, Snapshot& snap) {
    (void)extractNumber(obj, "x", snap.x);
    (void)extractNumber(obj, "y", snap.y);
    (void)extractNumber(obj, "z", snap.z);
    (void)extractNumber(obj, "roll", snap.roll);
    (void)extractNumber(obj, "pitch", snap.pitch);
    (void)extractNumber(obj, "yaw", snap.yaw);
    (void)extractNumber(obj, "vx", snap.vx);
    (void)extractNumber(obj, "vy", snap.vy);
    (void)extractNumber(obj, "wz", snap.wz);
    double support = 0.0;
    if (extractNumber(obj, "support", support)) {
        snap.support = static_cast<int>(support);
    }
    (void)extractNumber(obj, "stance_width_m", snap.stance_width_m);
    (void)extractNumber(obj, "foot_centroid_x", snap.foot_centroid_x);
    (void)extractNumber(obj, "foot_centroid_y", snap.foot_centroid_y);
    (void)extractNumber(obj, "body_to_centroid_x", snap.body_to_centroid_x);
    (void)extractNumber(obj, "body_to_centroid_y", snap.body_to_centroid_y);

    const auto feet_pos = obj.find("\"feet\":[");
    if (feet_pos != std::string::npos) {
        std::size_t i = feet_pos + 8;
        int idx = 0;
        while (i < obj.size() && idx < kNumLegs) {
            while (i < obj.size() && (obj[i] == ' ' || obj[i] == '\n' || obj[i] == ',')) {
                ++i;
            }
            if (i < obj.size() && obj[i] == ']') {
                break;
            }
            if (i >= obj.size() || obj[i] != '{') {
                break;
            }
            const auto end = obj.find('}', i);
            if (end == std::string::npos) {
                break;
            }
            const std::string foot = obj.substr(i, end - i + 1);
            (void)extractNumber(foot, "bx", snap.foot_body_x[static_cast<std::size_t>(idx)]);
            (void)extractNumber(foot, "by", snap.foot_body_y[static_cast<std::size_t>(idx)]);
            double loaded = 0.0;
            if (extractNumber(foot, "support", loaded)) {
                snap.fused_support[static_cast<std::size_t>(idx)] = static_cast<int>(loaded);
            }
            ++idx;
            i = end + 1;
        }
    }
    snap.valid = true;
    return true;
}

bool loadDump(const char* path, Dump& dump, std::string& err) {
    if (path == nullptr || path[0] == '\0') {
        err = "missing_fixture";
        return false;
    }
    std::ifstream exists(path);
    if (!exists.good()) {
        err = "missing_fixture";
        return false;
    }
    exists.close();
    std::string text;
    if (!loadFile(path, text)) {
        err = "missing_fixture";
        return false;
    }
    (void)extractNumber(text, "held", dump.held);
    dump.have_net = extractNumber(text, "net_horizontal_distance_m", dump.net);
    (void)extractNumber(text, "cmd_yaw_radps", dump.cmd_yaw);
    (void)extractNumber(text, "start_x_m", dump.start_x);
    (void)extractNumber(text, "start_y_m", dump.start_y);
    std::string stand_obj;
    std::string first_obj;
    if (!extractObject(text, "stand_end", stand_obj) || !parseSnapshot(stand_obj, dump.stand_end)) {
        err = "unknown";
        return false;
    }
    if (!extractObject(text, "first_walk", first_obj) || !parseSnapshot(first_obj, dump.first_walk)) {
        err = "unknown";
        return false;
    }
    return dump.stand_end.valid && dump.first_walk.valid;
}

Delta compareSnapshots(const Snapshot& a, const Snapshot& b) {
    Delta d{};
    d.dxy = std::hypot(b.x - a.x, b.y - a.y);
    d.dz = b.z - a.z;
    const double tilt_a = std::hypot(a.roll, a.pitch);
    const double tilt_b = std::hypot(b.roll, b.pitch);
    d.dtilt = tilt_b - tilt_a;
    d.dyaw = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    d.dspeed = std::hypot(b.vx, b.vy) - std::hypot(a.vx, a.vy);
    d.dwz = b.wz - a.wz;
    d.dsupport = b.support - a.support;
    d.dwidth = b.stance_width_m - a.stance_width_m;
    d.dcentroid_body = std::hypot(b.body_to_centroid_x - a.body_to_centroid_x,
                                  b.body_to_centroid_y - a.body_to_centroid_y);
    double sse = 0.0;
    for (int i = 0; i < kNumLegs; ++i) {
        const double ex = b.foot_body_x[static_cast<std::size_t>(i)]
            - a.foot_body_x[static_cast<std::size_t>(i)];
        const double ey = b.foot_body_y[static_cast<std::size_t>(i)]
            - a.foot_body_y[static_cast<std::size_t>(i)];
        sse += ex * ex + ey * ey;
    }
    d.foot_body_rmse = std::sqrt(sse / static_cast<double>(kNumLegs));
    return d;
}

bool stanceDiffers(const Delta& d) {
    return std::abs(d.dsupport) >= 1
        || std::abs(d.dz) > 0.008
        || std::abs(d.dtilt) > 0.03
        || std::abs(d.dwidth) > 0.015;
}

bool poseDiffers(const Delta& d) {
    return d.dcentroid_body > 0.015 || d.foot_body_rmse > 0.015;
}

Classified classify(const Dump& isolated, const Dump& sequential) {
    Classified out{};
    out.stand = compareSnapshots(isolated.stand_end, sequential.stand_end);
    out.first = compareSnapshots(isolated.first_walk, sequential.first_walk);
    if (stanceDiffers(out.first)) {
        out.rule = "entry_stance";
    } else if (poseDiffers(out.first)) {
        out.rule = "entry_pose";
    } else {
        out.rule = "entry_match";
    }
    return out;
}

double snapshotTilt(const Snapshot& s) {
    return std::hypot(s.roll, s.pitch);
}

double snapshotCentroidMag(const Snapshot& s) {
    return std::hypot(s.body_to_centroid_x, s.body_to_centroid_y);
}

bool failMoreExtremeThanPass(const Snapshot& fail, const Snapshot& pass) {
    return fail.support < pass.support
        || (snapshotTilt(fail) - snapshotTilt(pass)) > 0.02
        || (snapshotCentroidMag(fail) - snapshotCentroidMag(pass)) > 0.015
        || (std::abs(fail.wz) - std::abs(pass.wz)) > 0.03;
}

bool failCloserToIsolated(const Snapshot& fail, const Snapshot& pass, const Snapshot& isolated) {
    const double fail_support = std::abs(fail.support - isolated.support);
    const double pass_support = std::abs(pass.support - isolated.support);
    const double fail_tilt = std::abs(snapshotTilt(fail) - snapshotTilt(isolated));
    const double pass_tilt = std::abs(snapshotTilt(pass) - snapshotTilt(isolated));
    const double fail_c = std::abs(snapshotCentroidMag(fail) - snapshotCentroidMag(isolated));
    const double pass_c = std::abs(snapshotCentroidMag(pass) - snapshotCentroidMag(isolated));
    const double fail_wz = std::abs(fail.wz - isolated.wz);
    const double pass_wz = std::abs(pass.wz - isolated.wz);
    return fail_support < pass_support && fail_tilt < pass_tilt && fail_c < pass_c && fail_wz < pass_wz;
}

std::string vsPassRule(const Dump& fail, const Dump& pass, const Dump& isolated) {
    const Snapshot& f = fail.first_walk;
    const Snapshot& p = pass.first_walk;
    const Snapshot& i = isolated.first_walk;
    if (failMoreExtremeThanPass(f, p)) {
        return "more_extreme";
    }
    if (failCloserToIsolated(f, p, i)) {
        return "milder";
    }
    return "similar";
}

void printDump(const char* tag, const char* path, const Dump& dump) {
    const Snapshot& s = dump.stand_end;
    const Snapshot& f = dump.first_walk;
    std::cout << "P0_TURN_ENTRY tag=" << tag
              << " path=" << path
              << " net=" << dump.net
              << " have_net=" << (dump.have_net ? "true" : "false")
              << " held=" << dump.held
              << " start_xy=" << dump.start_x << "," << dump.start_y
              << " stand_z=" << s.z
              << " stand_tilt=" << std::hypot(s.roll, s.pitch)
              << " stand_width=" << s.stance_width_m
              << " stand_support=" << s.support
              << " stand_centroid_body=" << s.body_to_centroid_x << "," << s.body_to_centroid_y
              << " first_z=" << f.z
              << " first_tilt=" << std::hypot(f.roll, f.pitch)
              << " first_width=" << f.stance_width_m
              << " first_support=" << f.support
              << " first_centroid_body=" << f.body_to_centroid_x << "," << f.body_to_centroid_y
              << " first_xy=" << f.x << "," << f.y
              << " first_yaw=" << f.yaw
              << " first_speed=" << std::hypot(f.vx, f.vy)
              << " first_wz=" << f.wz << '\n';
}

void printDelta(const char* phase, const Delta& d) {
    std::cout << "P0_TURN_ENTRY_DELTA phase=" << phase
              << " dxy=" << d.dxy
              << " dz=" << d.dz
              << " dtilt=" << d.dtilt
              << " dyaw=" << d.dyaw
              << " dspeed=" << d.dspeed
              << " dwz=" << d.dwz
              << " dsupport=" << d.dsupport
              << " dwidth=" << d.dwidth
              << " dcentroid_body=" << d.dcentroid_body
              << " foot_body_rmse=" << d.foot_body_rmse << '\n';
}

} // namespace

int main() {
    const char* isolated_path = HEXAPOD_P0_TURN_ENTRY_ISOLATED_FIXTURE;
    const char* sequential_path = HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FIXTURE;
    const char* fail_path = HEXAPOD_P0_TURN_ENTRY_SEQUENTIAL_FAIL_FIXTURE;
    Dump isolated{};
    Dump sequential{};
    Dump fail{};
    std::string isolated_err;
    std::string sequential_err;
    std::string fail_err;
    const bool have_isolated = loadDump(isolated_path, isolated, isolated_err);
    const bool have_sequential = loadDump(sequential_path, sequential, sequential_err);
    const bool have_fail = loadDump(fail_path, fail, fail_err);
    if (have_isolated) {
        printDump("isolated", isolated_path, isolated);
    } else {
        std::cout << "P0_TURN_ENTRY tag=isolated classification=" << isolated_err
                  << " path=" << (isolated_path ? isolated_path : "") << '\n';
    }
    if (have_sequential) {
        printDump("sequential", sequential_path, sequential);
    } else {
        std::cout << "P0_TURN_ENTRY tag=sequential classification=" << sequential_err
                  << " path=" << (sequential_path ? sequential_path : "") << '\n';
    }
    if (have_fail) {
        printDump("sequential_fail", fail_path, fail);
    } else {
        std::cout << "P0_TURN_ENTRY tag=sequential_fail classification=" << fail_err
                  << " path=" << (fail_path ? fail_path : "") << '\n';
    }
    if (have_isolated && have_sequential) {
        const Classified c = classify(isolated, sequential);
        printDelta("stand_end", c.stand);
        printDelta("first_walk", c.first);
        std::cout << "P0_TURN_ENTRY classification=" << c.rule << '\n';
    } else if (!have_isolated && isolated_err == "missing_fixture"
               && !have_sequential && sequential_err == "missing_fixture") {
        std::cout << "P0_TURN_ENTRY classification=missing_fixture\n";
    } else {
        std::cout << "P0_TURN_ENTRY classification=unknown\n";
    }
    if (have_isolated && have_fail) {
        const Classified fail_vs_iso = classify(isolated, fail);
        printDelta("fail_stand_end", fail_vs_iso.stand);
        printDelta("fail_first_walk", fail_vs_iso.first);
        std::cout << "P0_TURN_ENTRY_FAIL vs_isolated=" << fail_vs_iso.rule << '\n';
    } else {
        std::cout << "P0_TURN_ENTRY_FAIL vs_isolated=" << (have_fail ? "unknown" : "missing_fixture")
                  << '\n';
    }
    if (have_isolated && have_sequential && have_fail) {
        const Delta fail_vs_pass = compareSnapshots(sequential.first_walk, fail.first_walk);
        printDelta("fail_vs_pass_first_walk", fail_vs_pass);
        std::cout << "P0_TURN_ENTRY_FAIL vs_pass=" << vsPassRule(fail, sequential, isolated) << '\n';
    } else {
        std::cout << "P0_TURN_ENTRY_FAIL vs_pass=" << (have_fail ? "unknown" : "missing_fixture")
                  << '\n';
    }
    return EXIT_SUCCESS;
}
