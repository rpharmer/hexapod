// Parse the first-trip TIP_OVER dump: angle vs rate vs both vs unknown.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#ifndef HEXAPOD_P0_TIP_OVER_FIXTURE
#define HEXAPOD_P0_TIP_OVER_FIXTURE ""
#endif

namespace {

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

bool extractString(const std::string& text, const char* key, std::string& out) {
    const std::string needle = std::string("\"") + key + "\":\"";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    const std::size_t start = pos + needle.size();
    const auto end = text.find('"', start);
    if (end == std::string::npos) {
        return false;
    }
    out = text.substr(start, end - start);
    return true;
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

} // namespace

int main() {
    const char* path = HEXAPOD_P0_TIP_OVER_FIXTURE;
    if (path == nullptr || path[0] == '\0') {
        std::cout << "P0_TIP_OVER classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::ifstream exists(path);
    if (!exists.good()) {
        std::cout << "P0_TIP_OVER classification=missing_fixture path=" << path << '\n';
        return EXIT_SUCCESS;
    }
    exists.close();
    std::string text;
    if (!loadFile(path, text)) {
        std::cout << "P0_TIP_OVER classification=missing_fixture path=" << path << '\n';
        return EXIT_SUCCESS;
    }

    std::string rule;
    double roll = 0.0;
    double pitch = 0.0;
    double gyro = 0.0;
    double support = 0.0;
    double max_tilt = 0.0;
    double rapid_rate = 0.0;
    double rapid_contacts = 0.0;
    double loop = 0.0;
    (void)extractString(text, "rule", rule);
    (void)extractNumber(text, "roll_rad", roll);
    (void)extractNumber(text, "pitch_rad", pitch);
    (void)extractNumber(text, "gyro_hypot_radps", gyro);
    (void)extractNumber(text, "support_count", support);
    (void)extractNumber(text, "max_tilt_rad", max_tilt);
    (void)extractNumber(text, "rapid_body_rate_radps", rapid_rate);
    (void)extractNumber(text, "rapid_body_rate_max_contacts", rapid_contacts);
    (void)extractNumber(text, "loop", loop);

    const bool angle = std::abs(roll) > max_tilt || std::abs(pitch) > max_tilt;
    const bool rate = gyro > rapid_rate && support <= rapid_contacts;
    std::cout << "P0_TIP_OVER classification=" << (rule.empty() ? "unknown" : rule)
              << " loop=" << loop
              << " roll=" << roll
              << " pitch=" << pitch
              << " gyro_hypot=" << gyro
              << " support=" << support
              << " max_tilt=" << max_tilt
              << " rapid_rate=" << rapid_rate
              << " rapid_contacts=" << rapid_contacts
              << " angle_vs_max=" << (angle ? 1 : 0)
              << " rate_vs_thresh=" << (rate ? 1 : 0) << '\n';
    return EXIT_SUCCESS;
}
