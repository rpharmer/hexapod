// Parse turn-traj dumps: circle fit, CoR, body-frame halves → orbit|translation|entry|late|unknown.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef HEXAPOD_P0_TURN_TRAJ_ISOLATED_FIXTURE
#define HEXAPOD_P0_TURN_TRAJ_ISOLATED_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_TURN_TRAJ_SEQUENTIAL_FIXTURE
#define HEXAPOD_P0_TURN_TRAJ_SEQUENTIAL_FIXTURE ""
#endif

namespace {

struct Tick {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    int support{0};
};

struct Dump {
    double net{0.0};
    double yaw_delta{0.0};
    double r_equivalent{0.0};
    double path_per_rad{0.0};
    double held{0.0};
    std::vector<Tick> samples;
};

struct Classified {
    std::string rule{"unknown"};
    double fit_cx{0.0};
    double fit_cy{0.0};
    double fit_r{0.0};
    double fit_rmse{0.0};
    double rel_residual{0.0};
    double cor_mean_x{0.0};
    double cor_mean_y{0.0};
    double cor_std{0.0};
    int cor_n{0};
    double mean_body_vx{0.0};
    double mean_body_vy{0.0};
    double first_net{0.0};
    double second_net{0.0};
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

bool parseObjectNumber(const std::string& obj, const char* key, double& out) {
    return extractNumber(obj, key, out);
}

bool parseSamples(const std::string& text, std::vector<Tick>& out) {
    const auto start = text.find("\"samples\":[");
    if (start == std::string::npos) {
        return false;
    }
    std::size_t i = start + 11;
    while (i < text.size()) {
        while (i < text.size() && (text[i] == ' ' || text[i] == '\n' || text[i] == ',')) {
            ++i;
        }
        if (i < text.size() && text[i] == ']') {
            return true;
        }
        if (i >= text.size() || text[i] != '{') {
            return !out.empty();
        }
        const auto end = text.find('}', i);
        if (end == std::string::npos) {
            return false;
        }
        const std::string obj = text.substr(i, end - i + 1);
        Tick t{};
        (void)parseObjectNumber(obj, "x", t.x);
        (void)parseObjectNumber(obj, "y", t.y);
        (void)parseObjectNumber(obj, "yaw", t.yaw);
        (void)parseObjectNumber(obj, "vx", t.vx);
        (void)parseObjectNumber(obj, "vy", t.vy);
        (void)parseObjectNumber(obj, "wz", t.wz);
        double support = 0.0;
        if (parseObjectNumber(obj, "support", support)) {
            t.support = static_cast<int>(support);
        }
        out.push_back(t);
        i = end + 1;
    }
    return !out.empty();
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
    (void)extractNumber(text, "net_horizontal_distance_m", dump.net);
    (void)extractNumber(text, "yaw_delta_rad", dump.yaw_delta);
    (void)extractNumber(text, "r_equivalent_m", dump.r_equivalent);
    (void)extractNumber(text, "path_per_rad_m", dump.path_per_rad);
    (void)extractNumber(text, "held", dump.held);
    if (!parseSamples(text, dump.samples) || dump.samples.empty()) {
        err = "unknown";
        return false;
    }
    return true;
}

bool circleFit(const std::vector<Tick>& samples, double& cx, double& cy, double& r, double& rmse) {
    const double n = static_cast<double>(samples.size());
    if (n < 8.0) {
        return false;
    }
    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    double sx = 0.0;
    double sy = 0.0;
    double sxu = 0.0;
    double syu = 0.0;
    double su = 0.0;
    for (const Tick& t : samples) {
        const double u = t.x * t.x + t.y * t.y;
        sxx += t.x * t.x;
        sxy += t.x * t.y;
        syy += t.y * t.y;
        sx += t.x;
        sy += t.y;
        sxu += t.x * u;
        syu += t.y * u;
        su += u;
    }
    // Solve 3x3 via Cramer's rule: [sxx sxy sx; sxy syy sy; sx sy n] [D E F]^T = -[sxu syu su]
    const double a11 = sxx;
    const double a12 = sxy;
    const double a13 = sx;
    const double a21 = sxy;
    const double a22 = syy;
    const double a23 = sy;
    const double a31 = sx;
    const double a32 = sy;
    const double a33 = n;
    const double b1 = -sxu;
    const double b2 = -syu;
    const double b3 = -su;
    const double det = a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31);
    if (!std::isfinite(det) || std::abs(det) < 1e-18) {
        return false;
    }
    const double det_d = b1 * (a22 * a33 - a23 * a32) - a12 * (b2 * a33 - a23 * b3)
        + a13 * (b2 * a32 - a22 * b3);
    const double det_e = a11 * (b2 * a33 - a23 * b3) - b1 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * b3 - b2 * a31);
    const double det_f = a11 * (a22 * b3 - b2 * a32) - a12 * (a21 * b3 - b2 * a31)
        + b1 * (a21 * a32 - a22 * a31);
    const double D = det_d / det;
    const double E = det_e / det;
    const double F = det_f / det;
    cx = -0.5 * D;
    cy = -0.5 * E;
    const double rad2 = cx * cx + cy * cy - F;
    if (!(rad2 > 0.0) || !std::isfinite(rad2)) {
        return false;
    }
    r = std::sqrt(rad2);
    double sse = 0.0;
    for (const Tick& t : samples) {
        const double ri = std::hypot(t.x - cx, t.y - cy);
        const double e = ri - r;
        sse += e * e;
    }
    rmse = std::sqrt(sse / n);
    return std::isfinite(cx) && std::isfinite(cy) && std::isfinite(r) && std::isfinite(rmse);
}

void worldFromBody(const Tick& t, double& vwx, double& vwy) {
    const double c = std::cos(t.yaw);
    const double s = std::sin(t.yaw);
    vwx = t.vx * c - t.vy * s;
    vwy = t.vx * s + t.vy * c;
}

Classified classify(const Dump& dump) {
    Classified out{};
    double rmse = 0.0;
    const bool fitted = circleFit(dump.samples, out.fit_cx, out.fit_cy, out.fit_r, rmse);
    out.fit_rmse = rmse;
    const double net = std::max(dump.net, 1e-3);
    out.rel_residual = fitted ? (rmse / net) : 1e9;

    double body_vx_sum = 0.0;
    double body_vy_sum = 0.0;
    int body_n = 0;
    double cor_x_sum = 0.0;
    double cor_y_sum = 0.0;
    double cor_x2 = 0.0;
    double cor_y2 = 0.0;
    constexpr double kWzCut = 0.05;
    for (const Tick& t : dump.samples) {
        body_vx_sum += t.vx;
        body_vy_sum += t.vy;
        ++body_n;
        if (std::abs(t.wz) <= kWzCut) {
            continue;
        }
        double vwx = 0.0;
        double vwy = 0.0;
        worldFromBody(t, vwx, vwy);
        const double cx = t.x - vwy / t.wz;
        const double cy = t.y + vwx / t.wz;
        if (!std::isfinite(cx) || !std::isfinite(cy)) {
            continue;
        }
        cor_x_sum += cx;
        cor_y_sum += cy;
        cor_x2 += cx * cx;
        cor_y2 += cy * cy;
        ++out.cor_n;
    }
    if (body_n > 0) {
        out.mean_body_vx = body_vx_sum / static_cast<double>(body_n);
        out.mean_body_vy = body_vy_sum / static_cast<double>(body_n);
    }
    if (out.cor_n > 1) {
        const double inv = 1.0 / static_cast<double>(out.cor_n);
        out.cor_mean_x = cor_x_sum * inv;
        out.cor_mean_y = cor_y_sum * inv;
        const double varx = std::max(0.0, cor_x2 * inv - out.cor_mean_x * out.cor_mean_x);
        const double vary = std::max(0.0, cor_y2 * inv - out.cor_mean_y * out.cor_mean_y);
        out.cor_std = std::sqrt(varx + vary);
    }

    const std::size_t mid = dump.samples.size() / 2;
    if (!dump.samples.empty()) {
        const Tick& a = dump.samples.front();
        const Tick& m = dump.samples[mid];
        const Tick& b = dump.samples.back();
        out.first_net = std::hypot(m.x - a.x, m.y - a.y);
        out.second_net = std::hypot(b.x - m.x, b.y - m.y);
    }

    const double chord = std::max(dump.r_equivalent, 1e-3);
    const bool orbit_shape = fitted && out.rel_residual < 0.25 && out.fit_r > 0.05
        && std::abs(out.fit_r - dump.r_equivalent) / chord < 0.35;
    const double body_planar = std::hypot(out.mean_body_vx, out.mean_body_vy);
    const bool translation = (!orbit_shape)
        && (body_planar > 0.03 || out.cor_std > 0.08);
    if (orbit_shape) {
        out.rule = "orbit";
    } else if (translation) {
        out.rule = "translation";
    } else if (out.first_net > 1.4 * out.second_net && out.first_net > 0.05) {
        out.rule = "entry";
    } else if (out.second_net > 1.4 * out.first_net && out.second_net > 0.05) {
        out.rule = "late";
    } else {
        out.rule = "unknown";
    }
    return out;
}

void printOne(const char* tag, const char* path, const Dump& dump, const Classified& c) {
    std::cout << "P0_TURN_TRAJ tag=" << tag
              << " path=" << path
              << " classification=" << c.rule
              << " net=" << dump.net
              << " yaw=" << dump.yaw_delta
              << " chord_r=" << dump.r_equivalent
              << " fit_r=" << c.fit_r
              << " fit_rmse=" << c.fit_rmse
              << " rel_residual=" << c.rel_residual
              << " cor_std=" << c.cor_std
              << " cor_n=" << c.cor_n
              << " mean_body_vx=" << c.mean_body_vx
              << " mean_body_vy=" << c.mean_body_vy
              << " first_net=" << c.first_net
              << " second_net=" << c.second_net
              << " held=" << dump.held
              << " samples=" << dump.samples.size() << '\n';
}

} // namespace

int main() {
    const char* isolated_path = HEXAPOD_P0_TURN_TRAJ_ISOLATED_FIXTURE;
    const char* sequential_path = HEXAPOD_P0_TURN_TRAJ_SEQUENTIAL_FIXTURE;
    Dump isolated{};
    std::string isolated_err;
    if (loadDump(isolated_path, isolated, isolated_err)) {
        printOne("isolated", isolated_path, isolated, classify(isolated));
    } else {
        std::cout << "P0_TURN_TRAJ tag=isolated classification=" << isolated_err
                  << " path=" << (isolated_path ? isolated_path : "") << '\n';
    }
    Dump sequential{};
    std::string sequential_err;
    if (loadDump(sequential_path, sequential, sequential_err)) {
        printOne("sequential", sequential_path, sequential, classify(sequential));
    } else {
        std::cout << "P0_TURN_TRAJ tag=sequential classification=" << sequential_err
                  << " path=" << (sequential_path ? sequential_path : "") << '\n';
    }
    return EXIT_SUCCESS;
}
