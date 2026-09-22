// Replay dumped R2 planSwingFoot inputs. Diagnose-only: no gait production change.

#include "body_controller.hpp"
#include "foot_planners.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "motion_intent_utils.hpp"
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
#include <unistd.h>
#include <vector>

#ifndef HEXAPOD_P3_SEQ_HISTORY_FIXTURE
#define HEXAPOD_P3_SEQ_HISTORY_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_SWING_PLANNER_FIXTURE
#define HEXAPOD_P0_SWING_PLANNER_FIXTURE ""
#endif

namespace {

constexpr std::size_t kR2Leg = 2;
constexpr std::size_t kR2CoxaWire = 6;
constexpr std::size_t kR2FemurWire = 7;
constexpr std::size_t kR2TibiaWire = 8;
constexpr double kReplayMatchM = 1.0e-6;
constexpr double kCmdFarFromNominalM = 0.040;

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

struct PlannerSample {
    SwingFootInputs sw{};
    BodyTwist twist{};
    RobotState est{};
    Vec3 planned_pre_rot{};
    Vec3 target_clamped{};
    double phase{0.0};
};

bool parsePlannerDump(const std::string& text, std::vector<PlannerSample>& out) {
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
        PlannerSample sample;
        if (!extractNumber(text, "phase", object_start, object_end, sample.phase)
            || !extractNumber(text, "tau01", object_start, object_end, sample.sw.tau01)
            || !extractNumber(text, "swing_span", object_start, object_end, sample.sw.swing_span)
            || !extractNumber(text, "f_hz", object_start, object_end, sample.sw.f_hz)
            || !extractNumber(text, "step_length_m", object_start, object_end, sample.sw.step_length_m)
            || !extractNumber(text, "swing_height_m", object_start, object_end, sample.sw.swing_height_m)
            || !extractNumber(text, "stance_lookahead_s", object_start, object_end, sample.sw.stance_lookahead_s)
            || !extractNumber(
                text, "static_stability_margin_m", object_start, object_end, sample.sw.static_stability_margin_m)
            || !extractNumber(text, "swing_time_ease_01", object_start, object_end, sample.sw.swing_time_ease_01)
            || !parseVec3(text, "anchor", object_start, object_end, sample.sw.anchor)
            || !parseVec3(text, "stance_end", object_start, object_end, sample.sw.stance_end)
            || !parseVec3(text, "v_liftoff_body", object_start, object_end, sample.sw.v_liftoff_body)
            || !parseVec3(text, "twist_linear_mps", object_start, object_end, sample.twist.linear_mps)
            || !parseVec3(text, "twist_angular_radps", object_start, object_end, sample.twist.angular_radps)
            || !parseVec3(text, "planned_pre_rot", object_start, object_end, sample.planned_pre_rot)
            || !parseVec3(text, "target_clamped", object_start, object_end, sample.target_clamped)) {
            return false;
        }
        (void)extractNumber(
            text, "cmd_accel_body_x_mps2", object_start, object_end, sample.sw.cmd_accel_body_x_mps2);
        (void)extractNumber(
            text, "cmd_accel_body_y_mps2", object_start, object_end, sample.sw.cmd_accel_body_y_mps2);
        extractBool(text, "est_valid", object_start, object_end, sample.est.valid);
        extractBool(text, "est_has_body_twist", object_start, object_end, sample.est.has_body_twist_state);
        Vec3 est_lin{};
        Vec3 est_ang{};
        if (parseVec3(text, "est_linear_mps", object_start, object_end, est_lin)
            && parseVec3(text, "est_angular_radps", object_start, object_end, est_ang)) {
            sample.est.body_twist_state.body_trans_mps = est_lin;
            sample.est.body_twist_state.twist_vel_radps = est_ang;
        }
        out.push_back(sample);
        i = object_end;
        skipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

double planarXy(const Vec3& a, const Vec3& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool replayIdentity(const std::vector<PlannerSample>& samples, const char* label) {
    double peak = 0.0;
    std::size_t peak_i = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        Vec3 pos{};
        Vec3 vel{};
        planSwingFoot(samples[i].est, samples[i].twist, samples[i].sw, pos, vel);
        const double err = std::hypot(pos.x - samples[i].planned_pre_rot.x,
                                      pos.y - samples[i].planned_pre_rot.y);
        if (err > peak) {
            peak = err;
            peak_i = i;
        }
        if (err > kReplayMatchM) {
            std::cerr << "p0-swing-plan: replay identity failed label=" << label << " i=" << i
                      << " xy_err=" << err << '\n';
            return false;
        }
    }
    std::cout << "P0_SWING_PLAN classification=replay_identity label=" << label
              << " samples=" << samples.size() << " peak_i=" << peak_i << " peak_xy_err=" << peak
              << '\n';
    return true;
}

Vec3 frozenVinLandingCmdFoot() {
    const char* path = HEXAPOD_P3_SEQ_HISTORY_FIXTURE;
    std::string text;
    if (path == nullptr || path[0] == '\0' || !loadFile(path, text)) {
        return Vec3{};
    }
    const std::string needle = "\"accepted_history\":";
    const auto pos = text.find(needle);
    if (pos == std::string::npos) {
        return Vec3{};
    }
    const auto targets_pos = text.rfind("\"targets\":");
    if (targets_pos == std::string::npos) {
        return Vec3{};
    }
    std::vector<double> values;
    std::size_t i = targets_pos + 10;
    if (!parseJsonNumberArray(text, i, values) || values.size() != 18) {
        return Vec3{};
    }
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const ServoCalibration& cal = geo.legGeometry[kR2Leg].servo;
    const LegState servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
        cal, static_cast<int>(kR2Leg), static_cast<float>(values[kR2CoxaWire]),
        static_cast<float>(values[kR2FemurWire]), static_cast<float>(values[kR2TibiaWire]));
    LegFK fk{};
    const FootTarget foot = fk.footInBodyFrame(servo, geo.legGeometry[kR2Leg]);
    return Vec3{foot.pos_body_m.x, foot.pos_body_m.y, foot.pos_body_m.z};
}

} // namespace

int main() {
    char dump_path[] = "/tmp/hexapod-swing-planner-unit.json";
    ::unlink(dump_path);
    ::setenv("HEXAPOD_SWING_PLANNER_DUMP_PATH", dump_path, 1);

    BodyController controller{};
    RobotState est{};
    SafetyState safety{};
    safety.inhibit_motion = false;
    MotionIntent walk_intent{};
    walk_intent.requested_mode = RobotMode::WALK;
    walk_intent.cmd_vx_mps = LinearRateMps{0.12};
    walk_intent.twist.body_trans_m.z = 0.14;
    walk_intent.timestamp_us = now_us();
    GaitState gait{};
    gait.duty_factor = 0.5;
    gait.stride_phase_rate_hz = FrequencyHz{1.0};
    gait.step_length_m = 0.06;
    gait.swing_height_m = 0.03;
    gait.phase[kR2Leg] = 0.75;
    gait.in_stance[kR2Leg] = false;
    const BodyTwist twist = rawLocomotionTwistFromIntent(walk_intent, planarMotionCommand(walk_intent));
    const LegTargets targets = controller.update(est, walk_intent, gait, safety, twist);
    (void)targets;
    ::unsetenv("HEXAPOD_SWING_PLANNER_DUMP_PATH");

    std::string dump_text;
    std::vector<PlannerSample> samples;
    if (!loadFile(dump_path, dump_text) || !parsePlannerDump(dump_text, samples)) {
        std::cerr << "p0-swing-plan: in-process dump missing path=" << dump_path << '\n';
        return EXIT_FAILURE;
    }
    if (!replayIdentity(samples, "in_process")) {
        return EXIT_FAILURE;
    }

    const Vec3 vin_cmd = frozenVinLandingCmdFoot();
    const Vec3 unit_clamped = samples.front().target_clamped;
    const double unit_vs_vin = planarXy(unit_clamped, vin_cmd);
    std::cout << "P0_SWING_PLAN unit_clamped=(" << unit_clamped.x << "," << unit_clamped.y << ","
              << unit_clamped.z << ") vin_cmd=(" << vin_cmd.x << "," << vin_cmd.y << "," << vin_cmd.z
              << ") dxy=" << unit_vs_vin << '\n';

    const char* frozen_path = HEXAPOD_P0_SWING_PLANNER_FIXTURE;
    if (frozen_path != nullptr && frozen_path[0] != '\0') {
        std::string frozen_text;
        std::vector<PlannerSample> frozen;
        if (loadFile(frozen_path, frozen_text) && parsePlannerDump(frozen_text, frozen)
            && replayIdentity(frozen, "fixture")) {
            double nearest = 1.0e9;
            std::size_t nearest_i = 0;
            double min_y = 1.0e9;
            double max_y = -1.0e9;
            for (std::size_t i = 0; i < frozen.size(); ++i) {
                const double d = planarXy(frozen[i].target_clamped, vin_cmd);
                if (d < nearest) {
                    nearest = d;
                    nearest_i = i;
                }
                min_y = std::min(min_y, frozen[i].target_clamped.y);
                max_y = std::max(max_y, frozen[i].target_clamped.y);
            }
            const bool matches_medial = nearest < kCmdFarFromNominalM;
            std::cout << "P0_SWING_PLAN frozen classification="
                      << (matches_medial ? "planner_matches_medial_cmd" : "planner_misses_medial_cmd")
                      << " nearest_i=" << nearest_i << " dxy_to_vin_cmd=" << nearest
                      << " clamped_y=[" << min_y << ":" << max_y << "]"
                      << " samples=" << frozen.size() << '\n';
        } else {
            std::cout << "P0_SWING_PLAN frozen classification=missing_fixture path="
                      << frozen_path << '\n';
        }
    } else {
        std::cout << "P0_SWING_PLAN frozen classification=missing_fixture\n";
    }

    return EXIT_SUCCESS;
}
