// Replay dumped R2 planned/IK/slew feet vs frozen vin-landing cmd XY.
// Diagnose-only: no gait, governor, Kd, or plant change.

#include "body_controller.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef HEXAPOD_P3_SEQ_HISTORY_FIXTURE
#define HEXAPOD_P3_SEQ_HISTORY_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_COMMANDED_FOOT_FIXTURE
#define HEXAPOD_P0_COMMANDED_FOOT_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_NOMINAL_DEPARTURE_FIXTURE
#define HEXAPOD_P0_NOMINAL_DEPARTURE_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_DEPARTURE_THROUGH_HOLD_FIXTURE
#define HEXAPOD_P0_DEPARTURE_THROUGH_HOLD_FIXTURE ""
#endif

namespace {

constexpr std::size_t kR2Leg = 2;
constexpr std::size_t kR2CoxaWire = 6;
constexpr std::size_t kR2FemurWire = 7;
constexpr std::size_t kR2TibiaWire = 8;
constexpr double kCmdFarFromNominalM = 0.040;
constexpr double kMatchVinM = 0.040;

void skipWs(const std::string& text, std::size_t& i) {
    while (i < text.size() && (text[i] == ' ' || text[i] == '\n' || text[i] == '\r' || text[i] == '\t')) {
        ++i;
    }
}

bool parseJsonNumberArray(const std::string& text, std::size_t& i, std::vector<double>& out) {
    skipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    skipWs(text, i);
    if (i < text.size() && text[i] == ']') {
        ++i;
        return true;
    }
    while (i < text.size()) {
        skipWs(text, i);
        char* end = nullptr;
        const double value = std::strtod(text.c_str() + i, &end);
        if (end == text.c_str() + i) {
            return false;
        }
        out.push_back(value);
        i = static_cast<std::size_t>(end - text.c_str());
        skipWs(text, i);
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

bool extractArray(
    const std::string& text, const char* key, std::size_t from, std::size_t to, std::vector<double>& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    std::size_t i = pos + needle.size();
    return parseJsonNumberArray(text, i, out);
}

bool extractNumber(
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

bool extractBool(const std::string& text, const char* key, std::size_t from, std::size_t to, bool& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    std::size_t i = pos + needle.size();
    skipWs(text, i);
    if (text.compare(i, 4, "true") == 0) {
        out = true;
        return true;
    }
    if (text.compare(i, 5, "false") == 0) {
        out = false;
        return true;
    }
    return false;
}

bool extractString(const std::string& text, const char* key, std::size_t from, std::size_t to, std::string& out) {
    const std::string needle = std::string("\"") + key + "\":";
    const auto pos = text.find(needle, from);
    if (pos == std::string::npos || pos >= to) {
        return false;
    }
    std::size_t i = pos + needle.size();
    skipWs(text, i);
    if (i >= text.size() || text[i] != '"') {
        return false;
    }
    ++i;
    const std::size_t start = i;
    while (i < text.size() && text[i] != '"') {
        ++i;
    }
    if (i >= text.size()) {
        return false;
    }
    out = text.substr(start, i - start);
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

bool parseVec3(const std::string& text, const char* key, std::size_t from, std::size_t to, Vec3& out) {
    std::vector<double> values;
    if (!extractArray(text, key, from, to, values) || values.size() != 3) {
        return false;
    }
    out = Vec3{values[0], values[1], values[2]};
    return true;
}

double planarXy(const Vec3& a, const Vec3& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

struct HistorySample {
    std::array<double, 18> targets{};
};

bool parseAcceptedHistory(const std::string& text, std::vector<HistorySample>& out) {
    const std::string needle = "\"accepted_history\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    skipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    skipWs(text, i);
    while (i < text.size() && text[i] != ']') {
        skipWs(text, i);
        if (i >= text.size() || text[i] != '{') {
            return false;
        }
        const std::size_t object_start = i;
        int depth = 0;
        std::size_t object_end = i;
        for (; object_end < text.size(); ++object_end) {
            if (text[object_end] == '{') {
                ++depth;
            } else if (text[object_end] == '}') {
                --depth;
                if (depth == 0) {
                    ++object_end;
                    break;
                }
            }
        }
        HistorySample sample;
        std::vector<double> targets;
        if (!extractArray(text, "targets", object_start, object_end, targets) || targets.size() != 18) {
            return false;
        }
        std::copy(targets.begin(), targets.end(), sample.targets.begin());
        out.push_back(sample);
        i = object_end;
        skipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

Vec3 commandedFootFromTargets(const std::array<double, 18>& targets) {
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const ServoCalibration& cal = geo.legGeometry[kR2Leg].servo;
    const LegState servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
        cal, static_cast<int>(kR2Leg), static_cast<float>(targets[kR2CoxaWire]),
        static_cast<float>(targets[kR2FemurWire]), static_cast<float>(targets[kR2TibiaWire]));
    LegFK fk{};
    const FootTarget foot = fk.footInBodyFrame(servo, geo.legGeometry[kR2Leg]);
    return Vec3{foot.pos_body_m.x, foot.pos_body_m.y, foot.pos_body_m.z};
}

struct CommandedFootSample {
    Vec3 planned{};
    Vec3 pre_slew_fk{};
    Vec3 post_slew_fk{};
    Vec3 nominal{};
    double phase{0.0};
    double loop{0.0};
    bool in_stance{false};
    bool hold_stance{false};
    bool slew_hit{false};
    bool ik_reach{false};
    bool departed{false};
    bool latched{false};
    bool bus_ok{true};
    std::string source;
    std::string trigger_stage;
};

bool parseCommandedFootDump(const std::string& text, std::vector<CommandedFootSample>& out) {
    const std::string needle = "\"samples\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return false;
    }
    std::size_t i = pos + needle.size();
    skipWs(text, i);
    if (i >= text.size() || text[i] != '[') {
        return false;
    }
    ++i;
    out.clear();
    skipWs(text, i);
    while (i < text.size() && text[i] != ']') {
        skipWs(text, i);
        if (i >= text.size() || text[i] != '{') {
            return false;
        }
        const std::size_t object_start = i;
        int depth = 0;
        std::size_t object_end = i;
        for (; object_end < text.size(); ++object_end) {
            if (text[object_end] == '{') {
                ++depth;
            } else if (text[object_end] == '}') {
                --depth;
                if (depth == 0) {
                    ++object_end;
                    break;
                }
            }
        }
        CommandedFootSample sample;
        if (!parseVec3(text, "planned", object_start, object_end, sample.planned)
            || !parseVec3(text, "pre_slew_fk", object_start, object_end, sample.pre_slew_fk)
            || !parseVec3(text, "post_slew_fk", object_start, object_end, sample.post_slew_fk)) {
            return false;
        }
        extractBool(text, "in_stance", object_start, object_end, sample.in_stance);
        extractBool(text, "hold_stance", object_start, object_end, sample.hold_stance);
        extractBool(text, "slew_hit", object_start, object_end, sample.slew_hit);
        extractBool(text, "ik_reach", object_start, object_end, sample.ik_reach);
        extractBool(text, "departed", object_start, object_end, sample.departed);
        extractBool(text, "latched", object_start, object_end, sample.latched);
        extractBool(text, "bus_ok", object_start, object_end, sample.bus_ok);
        extractString(text, "source", object_start, object_end, sample.source);
        extractString(text, "trigger_stage", object_start, object_end, sample.trigger_stage);
        (void)parseVec3(text, "nominal", object_start, object_end, sample.nominal);
        (void)extractNumber(text, "phase", object_start, object_end, sample.phase);
        (void)extractNumber(text, "loop", object_start, object_end, sample.loop);
        out.push_back(sample);
        i = object_end;
        skipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

struct NearestFoot {
    std::size_t index{0};
    double dxy{1.0e9};
    std::string source;
};

NearestFoot nearestToVin(const std::vector<CommandedFootSample>& samples,
                         const Vec3& vin_cmd,
                         Vec3 CommandedFootSample::*field) {
    NearestFoot best;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const double d = planarXy(samples[i].*field, vin_cmd);
        if (d < best.dxy) {
            best.dxy = d;
            best.index = i;
            best.source = samples[i].source;
        }
    }
    return best;
}

} // namespace

int main() {
    const char* history_path = HEXAPOD_P3_SEQ_HISTORY_FIXTURE;
    std::string history_text;
    std::vector<HistorySample> history;
    if (history_path == nullptr || history_path[0] == '\0' || !loadFile(history_path, history_text)
        || !parseAcceptedHistory(history_text, history) || history.size() < 2) {
        std::cerr << "p0-commanded-foot: failed to load history "
                  << (history_path != nullptr ? history_path : "(null)") << '\n';
        return EXIT_FAILURE;
    }

    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const Vec3 vin_cmd = commandedFootFromTargets(history.back().targets);
    std::size_t medial_count = 0;
    double min_y = 1.0e9;
    double max_y = -1.0e9;
    for (std::size_t i = 0; i < history.size(); ++i) {
        const Vec3 cmd = commandedFootFromTargets(history[i].targets);
        const double body_height_m = std::max(0.04, -cmd.z);
        const Vec3 nominal = computeNominalStance(geo, body_height_m)[kR2Leg];
        const double d_nominal = planarXy(cmd, nominal);
        const double d_vin = planarXy(cmd, vin_cmd);
        min_y = std::min(min_y, cmd.y);
        max_y = std::max(max_y, cmd.y);
        if (d_nominal > kCmdFarFromNominalM) {
            ++medial_count;
        }
        std::cout << "P0_COMMANDED_FOOT history_i=" << i << " cmd=(" << cmd.x << "," << cmd.y << "," << cmd.z
                  << ") d_nominal=" << d_nominal << " d_vin=" << d_vin << '\n';
    }
    const Vec3 first_cmd = commandedFootFromTargets(history.front().targets);
    const double first_d_nominal = planarXy(
        first_cmd, computeNominalStance(geo, std::max(0.04, -first_cmd.z))[kR2Leg]);
    const char* history_class = "never_medial_in_ring";
    if (first_d_nominal > kCmdFarFromNominalM && medial_count == history.size()) {
        history_class = "held_medial_from_ring_start";
    } else if (first_d_nominal <= kCmdFarFromNominalM && medial_count > 0) {
        history_class = "entered_medial_in_ring";
    }
    std::cout << "P0_COMMANDED_FOOT history classification=" << history_class
              << " samples=" << history.size() << " medial_count=" << medial_count
              << " cmd_y=[" << min_y << ":" << max_y << "]"
              << " vin_cmd=(" << vin_cmd.x << "," << vin_cmd.y << "," << vin_cmd.z << ")\n";

    const char* frozen_path = HEXAPOD_P0_COMMANDED_FOOT_FIXTURE;
    if (frozen_path == nullptr || frozen_path[0] == '\0') {
        std::cout << "P0_COMMANDED_FOOT frozen classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::string frozen_text;
    std::vector<CommandedFootSample> frozen;
    if (!loadFile(frozen_path, frozen_text) || !parseCommandedFootDump(frozen_text, frozen)) {
        std::cout << "P0_COMMANDED_FOOT frozen classification=missing_fixture path=" << frozen_path << '\n';
        return EXIT_SUCCESS;
    }

    const NearestFoot planned = nearestToVin(frozen, vin_cmd, &CommandedFootSample::planned);
    const NearestFoot pre_slew = nearestToVin(frozen, vin_cmd, &CommandedFootSample::pre_slew_fk);
    const NearestFoot post_slew = nearestToVin(frozen, vin_cmd, &CommandedFootSample::post_slew_fk);
    const char* frozen_class = "command_misses_medial_cmd";
    if (planned.dxy < kMatchVinM) {
        frozen_class = "planned_matches_medial_cmd";
    } else if (pre_slew.dxy < kMatchVinM) {
        frozen_class = "ik_matches_medial_cmd";
    } else if (post_slew.dxy < kMatchVinM) {
        frozen_class = "slew_holds_medial_cmd";
    }
    std::cout << "P0_COMMANDED_FOOT frozen classification=" << frozen_class
              << " samples=" << frozen.size()
              << " planned_i=" << planned.index << " planned_dxy=" << planned.dxy
              << " planned_source=" << planned.source
              << " pre_slew_i=" << pre_slew.index << " pre_slew_dxy=" << pre_slew.dxy
              << " post_slew_i=" << post_slew.index << " post_slew_dxy=" << post_slew.dxy
              << " post_slew_source=" << post_slew.source << '\n';

    const char* departure_path = HEXAPOD_P0_NOMINAL_DEPARTURE_FIXTURE;
    if (departure_path == nullptr || departure_path[0] == '\0') {
        std::cout << "P0_NOMINAL_DEPARTURE frozen classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::string departure_text;
    std::vector<CommandedFootSample> departure;
    if (!loadFile(departure_path, departure_text) || !parseCommandedFootDump(departure_text, departure)) {
        std::cout << "P0_NOMINAL_DEPARTURE frozen classification=missing_fixture path=" << departure_path
                  << '\n';
        return EXIT_SUCCESS;
    }
    std::size_t latch_i = departure.size();
    for (std::size_t i = 0; i < departure.size(); ++i) {
        if (departure[i].latched) {
            latch_i = i;
            break;
        }
    }
    if (latch_i >= departure.size()) {
        std::cout << "P0_NOMINAL_DEPARTURE frozen classification=no_departure samples=" << departure.size()
                  << '\n';
        return EXIT_SUCCESS;
    }
    const CommandedFootSample& latch = departure[latch_i];
    const char* stage_class = "no_departure";
    if (latch.trigger_stage == "planned") {
        stage_class = "planned_departs_first";
    } else if (latch.trigger_stage == "pre_slew") {
        stage_class = "ik_departs_first";
    } else if (latch.trigger_stage == "post_slew") {
        stage_class = "slew_departs_first";
    }
    const double latch_dxy = planarXy(latch.planned, vin_cmd);
    const double latch_post_dxy = planarXy(latch.post_slew_fk, vin_cmd);
    const char* vin_class =
        (std::min(latch_dxy, latch_post_dxy) < kMatchVinM) ? "departure_matches_medial_cmd"
                                                           : "departure_misses_medial_cmd";
    std::cout << "P0_NOMINAL_DEPARTURE frozen classification=" << stage_class
              << " vin_class=" << vin_class
              << " latch_i=" << latch_i
              << " source=" << latch.source
              << " stage=" << latch.trigger_stage
              << " planned=(" << latch.planned.x << "," << latch.planned.y << "," << latch.planned.z << ")"
              << " post_slew=(" << latch.post_slew_fk.x << "," << latch.post_slew_fk.y << ","
              << latch.post_slew_fk.z << ")"
              << " latch_dxy=" << latch_dxy
              << " latch_post_dxy=" << latch_post_dxy
              << " samples=" << departure.size() << '\n';

    const char* through_path = HEXAPOD_P0_DEPARTURE_THROUGH_HOLD_FIXTURE;
    if (through_path == nullptr || through_path[0] == '\0') {
        std::cout << "P0_CONTROLLER_ORIGIN classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::string through_text;
    std::vector<CommandedFootSample> through;
    if (!loadFile(through_path, through_text) || !parseCommandedFootDump(through_text, through)) {
        std::cout << "P0_CONTROLLER_ORIGIN classification=missing_fixture path=" << through_path << '\n';
        return EXIT_SUCCESS;
    }
    std::size_t through_latch = through.size();
    for (std::size_t i = 0; i < through.size(); ++i) {
        if (through[i].latched) {
            through_latch = i;
            break;
        }
    }
    std::size_t vin_i = through.size();
    double vin_planned_dxy = 1.0e9;
    double vin_post_dxy = 1.0e9;
    std::size_t min_y_i = through_latch < through.size() ? through_latch : 0;
    double min_cmd_y = 1.0e9;
    for (std::size_t i = (through_latch < through.size() ? through_latch : 0); i < through.size(); ++i) {
        const double py = through[i].planned.y;
        const double sy = through[i].post_slew_fk.y;
        if (std::min(py, sy) < min_cmd_y) {
            min_cmd_y = std::min(py, sy);
            min_y_i = i;
        }
        const double dp = planarXy(through[i].planned, vin_cmd);
        const double ds = planarXy(through[i].post_slew_fk, vin_cmd);
        if (vin_i >= through.size() && std::min(dp, ds) < kMatchVinM) {
            vin_i = i;
            vin_planned_dxy = dp;
            vin_post_dxy = ds;
        }
    }
    const char* origin = "never_reaches_vin";
    std::size_t origin_i = min_y_i;
    if (vin_i < through.size()) {
        origin_i = vin_i;
        const bool planned_hit = vin_planned_dxy < kMatchVinM;
        const bool post_hit = vin_post_dxy < kMatchVinM;
        const std::string& src = through[vin_i].source;
        if (post_hit && !planned_hit) {
            origin = "slew_tucks_to_vin";
        } else if (src == "hold") {
            origin = "hold_tucks_to_vin";
        } else if (src == "recovery_swing") {
            origin = "recovery_tucks_to_vin";
        } else if (src == "stance") {
            origin = "stance_tucks_to_vin";
        } else {
            origin = "planned_tucks_to_vin";
        }
    }
    const CommandedFootSample& origin_s = through[origin_i];
    std::cout << "P0_CONTROLLER_ORIGIN classification=" << origin
              << " origin_i=" << origin_i
              << " loop=" << origin_s.loop
              << " source=" << origin_s.source
              << " phase=" << origin_s.phase
              << " planned=(" << origin_s.planned.x << "," << origin_s.planned.y << ","
              << origin_s.planned.z << ")"
              << " post_slew=(" << origin_s.post_slew_fk.x << "," << origin_s.post_slew_fk.y << ","
              << origin_s.post_slew_fk.z << ")"
              << " planned_dxy=" << planarXy(origin_s.planned, vin_cmd)
              << " post_slew_dxy=" << planarXy(origin_s.post_slew_fk, vin_cmd)
              << " min_y=" << min_cmd_y
              << " samples=" << through.size()
              << " latch_i=" << through_latch << '\n';
    return EXIT_SUCCESS;
}
