// Replay dumped R2 swing Cartesian decomp: mm budget + offline counterfactuals.
// Diagnose-only: no Kd, governor, or P5 change.

#include "body_controller.hpp"
#include "foot_planners.hpp"
#include "foot_reachability.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"
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
#ifndef HEXAPOD_P0_SWING_DECOMP_FIXTURE
#define HEXAPOD_P0_SWING_DECOMP_FIXTURE ""
#endif
#ifndef HEXAPOD_P0_SWING_DECOMP_V3_FIXTURE
#define HEXAPOD_P0_SWING_DECOMP_V3_FIXTURE ""
#endif

namespace {

constexpr std::size_t kR2Leg = 2;
constexpr std::size_t kR2CoxaWire = 6;
constexpr std::size_t kR2FemurWire = 7;
constexpr std::size_t kR2TibiaWire = 8;
constexpr double kReplayMatchM = 1.0e-6;
constexpr double kMatchVinM = 0.040;
constexpr double kMedialDyM = 0.040;
constexpr double kVinTuckYM = 0.15;
constexpr double kWinnerFrac = 0.60;
constexpr double kFootReachInsetM = 0.004;
constexpr double kFloorInsideM = 0.040;
constexpr double kFloorClipYM = 0.119;
constexpr double kFloorBindM = 0.001;
constexpr double kV2VinDxyM = 0.023;
constexpr double kHeldFarTibiaWire = -0.85;

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

double planarXy(const Vec3& a, const Vec3& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

struct DecompSample {
    Vec3 planned{};
    Vec3 nominal{};
    Vec3 vin_cmd{};
    double phase{0.0};
    double loop{0.0};
    bool latched{false};
    bool decomp_valid{false};
    R2SwingDecompSnapshot decomp{};
};

bool parseDecompDump(const std::string& text, std::vector<DecompSample>& out) {
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
        DecompSample sample;
        if (!parseVec3(text, "planned", object_start, object_end, sample.planned)) {
            return false;
        }
        (void)parseVec3(text, "nominal", object_start, object_end, sample.nominal);
        (void)extractNumber(text, "phase", object_start, object_end, sample.phase);
        (void)extractNumber(text, "loop", object_start, object_end, sample.loop);
        extractBool(text, "latched", object_start, object_end, sample.latched);
        extractBool(text, "decomp_valid", object_start, object_end, sample.decomp_valid);
        if (sample.decomp_valid) {
            sample.decomp.valid = true;
            if (!extractNumber(text, "tau01", object_start, object_end, sample.decomp.tau01)
                || !extractNumber(text, "swing_span", object_start, object_end, sample.decomp.swing_span)
                || !extractNumber(text, "f_hz", object_start, object_end, sample.decomp.f_hz)
                || !extractNumber(text, "step_length_m", object_start, object_end, sample.decomp.step_length_m)
                || !extractNumber(text, "swing_height_m", object_start, object_end, sample.decomp.swing_height_m)
                || !extractNumber(
                    text, "stance_lookahead_s", object_start, object_end, sample.decomp.stance_lookahead_s)
                || !extractNumber(text,
                                  "static_stability_margin_m",
                                  object_start,
                                  object_end,
                                  sample.decomp.static_stability_margin_m)
                || !extractNumber(
                    text, "swing_time_ease_01", object_start, object_end, sample.decomp.swing_time_ease_01)
                || !parseVec3(text, "anchor", object_start, object_end, sample.decomp.anchor)
                || !parseVec3(text, "stance_end", object_start, object_end, sample.decomp.stance_end)
                || !parseVec3(text, "v_liftoff_body", object_start, object_end, sample.decomp.v_liftoff_body)
                || !parseVec3(text, "twist_linear_mps", object_start, object_end, sample.decomp.kinematic_twist.linear_mps)
                || !parseVec3(
                    text, "twist_angular_radps", object_start, object_end, sample.decomp.kinematic_twist.angular_radps)
                || !parseVec3(text, "planned_pre_rot", object_start, object_end, sample.decomp.planned_pre_rot)
                || !parseVec3(text, "after_terrain", object_start, object_end, sample.decomp.after_terrain)
                || !parseVec3(text, "origin_rot", object_start, object_end, sample.decomp.origin_rot)
                || !parseVec3(text, "coxa_rot", object_start, object_end, sample.decomp.coxa_rot)
                || !parseVec3(text, "target_clamped", object_start, object_end, sample.decomp.target_clamped)
                || !parseVec3(text, "foothold_nominal", object_start, object_end, sample.decomp.foothold_nominal)
                || !parseVec3(text, "capture_body", object_start, object_end, sample.decomp.capture_body)
                || !parseVec3(text, "foothold_final", object_start, object_end, sample.decomp.foothold_final)
                || !parseVec3(text, "terrain_xy_delta", object_start, object_end, sample.decomp.terrain_xy_delta)) {
                return false;
            }
            (void)parseVec3(text, "coxa", object_start, object_end, sample.decomp.coxa);
            (void)extractNumber(text, "capture_limit_m", object_start, object_end, sample.decomp.capture_limit_m);
            (void)extractNumber(text, "clamp_dxy", object_start, object_end, sample.decomp.clamp_dxy);
            (void)extractNumber(text, "roll_rad", object_start, object_end, sample.decomp.roll_rad);
            (void)extractNumber(text, "pitch_rad", object_start, object_end, sample.decomp.pitch_rad);
            (void)extractNumber(text, "yaw_rad", object_start, object_end, sample.decomp.yaw_rad);
            extractBool(text, "est_valid", object_start, object_end, sample.decomp.est_valid);
            extractBool(text, "est_has_body_twist", object_start, object_end, sample.decomp.est_has_body_twist);
            Vec3 est_lin{};
            Vec3 est_ang{};
            if (parseVec3(text, "est_linear_mps", object_start, object_end, est_lin)
                && parseVec3(text, "est_angular_radps", object_start, object_end, est_ang)) {
                sample.decomp.est_linear_mps = est_lin;
                sample.decomp.est_angular_radps = est_ang;
            }
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

Vec3 commandedFootFromHistoryTargets(const std::array<double, 18>& targets) {
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const ServoCalibration& cal = geo.legGeometry[kR2Leg].servo;
    const LegState servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
        cal,
        static_cast<int>(kR2Leg),
        static_cast<float>(targets[kR2CoxaWire]),
        static_cast<float>(targets[kR2FemurWire]),
        static_cast<float>(targets[kR2TibiaWire]));
    LegFK fk{};
    const FootTarget foot = fk.footInBodyFrame(servo, geo.legGeometry[kR2Leg]);
    return Vec3{foot.pos_body_m.x, foot.pos_body_m.y, foot.pos_body_m.z};
}

bool parseVinLanding(const std::string& text, Vec3& foot, double& tibia_wire) {
    const std::string needle = "\"accepted_history\":";
    if (text.find(needle) == std::string::npos) {
        return false;
    }
    const auto targets_pos = text.rfind("\"targets\":");
    if (targets_pos == std::string::npos) {
        return false;
    }
    std::vector<double> values;
    std::size_t i = targets_pos + 10;
    if (!parseJsonNumberArray(text, i, values) || values.size() != 18) {
        return false;
    }
    std::array<double, 18> targets{};
    std::copy(values.begin(), values.end(), targets.begin());
    foot = commandedFootFromHistoryTargets(targets);
    tibia_wire = values[kR2TibiaWire];
    return true;
}

SwingFootInputs swingInputsFromDecomp(const R2SwingDecompSnapshot& d) {
    SwingFootInputs sw{};
    sw.anchor = d.anchor;
    sw.stance_end = d.stance_end;
    sw.v_liftoff_body = d.v_liftoff_body;
    sw.tau01 = d.tau01;
    sw.swing_span = d.swing_span;
    sw.f_hz = d.f_hz;
    sw.step_length_m = d.step_length_m;
    sw.swing_height_m = d.swing_height_m;
    sw.cmd_accel_body_x_mps2 = d.cmd_accel_body_x_mps2;
    sw.cmd_accel_body_y_mps2 = d.cmd_accel_body_y_mps2;
    sw.stance_lookahead_s = d.stance_lookahead_s;
    sw.static_stability_margin_m = d.static_stability_margin_m;
    sw.swing_time_ease_01 = d.swing_time_ease_01;
    return sw;
}

RobotState estFromDecomp(const R2SwingDecompSnapshot& d) {
    RobotState est{};
    est.valid = d.est_valid;
    est.has_body_twist_state = d.est_has_body_twist;
    if (d.est_has_body_twist) {
        est.body_twist_state.body_trans_mps = d.est_linear_mps;
        est.body_twist_state.twist_vel_radps = d.est_angular_radps;
    }
    return est;
}

Mat3 bodyRotationFromDecomp(const R2SwingDecompSnapshot& d) {
    return (Mat3::rotZ(d.yaw_rad) * Mat3::rotY(d.pitch_rad) * Mat3::rotX(d.roll_rad)).transpose();
}

Vec3 clampSwing(const Vec3& p) {
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    return foot_reachability::clampFootPositionBody(geo.legGeometry[kR2Leg], p, kFootReachInsetM);
}

double ikTibiaWire(const Vec3& foot_body) {
    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const ServoCalibration& cal = geo.legGeometry[kR2Leg].servo;
    LegIK ik{geo};
    RobotState est{};
    est.timestamp_us = now_us();
    SafetyState safety{};
    safety.inhibit_motion = false;
    LegTargets targets{};
    targets.feet[kR2Leg].pos_body_m = foot_body;
    const JointTargets joints = ik.solve(est, targets, safety);
    float c = 0.0f;
    float f = 0.0f;
    float t = 0.0f;
    physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(
        cal, static_cast<int>(kR2Leg), joints.leg_states[kR2Leg], c, f, t);
    return static_cast<double>(t);
}

void printCounterfactual(const char* name, const Vec3& planned, const Vec3& vin, double dumped_tibia) {
    const double tibia = ikTibiaWire(planned);
    std::cout << "P0_SWING_DECOMP counterfactual=" << name
              << " planned=(" << planned.x << "," << planned.y << "," << planned.z << ")"
              << " dxy_to_vin=" << planarXy(planned, vin)
              << " ik_tibia_wire=" << tibia
              << " dumped_tibia_wire=" << dumped_tibia << '\n';
}

bool replayIdentity(const std::vector<DecompSample>& samples) {
    double peak = 0.0;
    std::size_t peak_i = 0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (!samples[i].decomp_valid) {
            continue;
        }
        ++n;
        Vec3 pos{};
        Vec3 vel{};
        planSwingFoot(estFromDecomp(samples[i].decomp),
                      samples[i].decomp.kinematic_twist,
                      swingInputsFromDecomp(samples[i].decomp),
                      pos,
                      vel);
        const double err = std::hypot(pos.x - samples[i].decomp.planned_pre_rot.x,
                                      pos.y - samples[i].decomp.planned_pre_rot.y);
        if (err > peak) {
            peak = err;
            peak_i = i;
        }
        if (err > kReplayMatchM) {
            std::cerr << "p0-swing-decomp: replay identity failed i=" << i << " xy_err=" << err << '\n';
            return false;
        }
    }
    std::cout << "P0_SWING_DECOMP classification=replay_identity samples=" << n << " peak_i=" << peak_i
              << " peak_xy_err=" << peak << '\n';
    return n > 0;
}

bool isVinTuck(const DecompSample& sample, const Vec3& vin_cmd) {
    return sample.decomp_valid
        && (sample.planned.y <= kVinTuckYM || planarXy(sample.planned, vin_cmd) < kMatchVinM);
}

double sampleFloorY(const R2SwingDecompSnapshot& d) {
    return std::max(0.0, std::abs(d.anchor.y) - kFloorInsideM);
}

bool sampleOnFloor(const DecompSample& sample) {
    if (!sample.decomp_valid) {
        return false;
    }
    const double floor_y = sampleFloorY(sample.decomp);
    const double planned_abs = std::abs(sample.planned.y);
    const double untilted_abs = std::abs(sample.decomp.after_terrain.y);
    const double pre_abs = std::abs(sample.decomp.planned_pre_rot.y);
    return std::abs(planned_abs - floor_y) <= kFloorBindM
        || std::abs(untilted_abs - floor_y) <= kFloorBindM
        || std::abs(pre_abs - floor_y) <= kFloorBindM;
}

const char* classifyFloor(bool floor_bound, bool has_vin_tuck, double min_planned_y, double vin_dxy) {
    if (floor_bound) {
        return "floor_working";
    }
    if (has_vin_tuck && std::abs(min_planned_y - kVinTuckYM) <= 0.01
        && std::abs(vin_dxy - kV2VinDxyM) <= 0.010) {
        return "floor_noop";
    }
    if (min_planned_y >= kFloorClipYM) {
        return "floor_working";
    }
    return "mixed";
}

bool scoreV3Dump(const std::vector<DecompSample>& frozen, const Vec3& vin_cmd, double vin_tibia) {
    if (!replayIdentity(frozen)) {
        return false;
    }
    double min_planned_y = 1.0e9;
    std::size_t min_i = frozen.size();
    std::size_t floor_bound_n = 0;
    std::size_t vin_tuck_count = 0;
    std::size_t sample_i = frozen.size();
    double best_dxy = 1.0e9;
    for (std::size_t i = 0; i < frozen.size(); ++i) {
        if (!frozen[i].decomp_valid) {
            continue;
        }
        if (frozen[i].planned.y < min_planned_y) {
            min_planned_y = frozen[i].planned.y;
            min_i = i;
        }
        if (sampleOnFloor(frozen[i])) {
            ++floor_bound_n;
        }
        if (!isVinTuck(frozen[i], vin_cmd)) {
            continue;
        }
        ++vin_tuck_count;
        const double d = planarXy(frozen[i].planned, vin_cmd);
        if (d < best_dxy) {
            best_dxy = d;
            sample_i = i;
        }
    }
    const bool floor_bound = floor_bound_n > 0;
    const bool has_vin_tuck = sample_i < frozen.size();
    const double class_y = has_vin_tuck ? frozen[sample_i].planned.y : min_planned_y;
    const double class_dxy = has_vin_tuck ? best_dxy
                                          : (min_i < frozen.size() ? planarXy(frozen[min_i].planned, vin_cmd) : 1.0e9);
    const char* floor_class = classifyFloor(floor_bound, has_vin_tuck, class_y, class_dxy);

    if (!has_vin_tuck) {
        const DecompSample& s = frozen[min_i];
        const double floor_y = s.decomp_valid ? sampleFloorY(s.decomp) : 0.0;
        std::cout << "P0_SWING_DECOMP v3 classification=not_vin_tuck"
                  << " floor_class=" << floor_class
                  << " min_planned_y=" << min_planned_y
                  << " min_i=" << min_i
                  << " loop=" << (min_i < frozen.size() ? s.loop : 0.0)
                  << " floor_y=" << floor_y
                  << " floor_bound=" << (floor_bound ? 1 : 0)
                  << " floor_bound_n=" << floor_bound_n
                  << " dxy_to_vin=" << class_dxy
                  << " v2_dxy=" << kV2VinDxyM
                  << " samples=" << frozen.size()
                  << " vin_tuck_count=" << vin_tuck_count << '\n';
        if (min_i < frozen.size()) {
            printCounterfactual("dumped", s.planned, vin_cmd, vin_tibia);
        }
        return true;
    }

    const DecompSample& s = frozen[sample_i];
    const R2SwingDecompSnapshot& d = s.decomp;
    const Vec3 untilted = d.after_terrain;
    const double medial_dy = s.nominal.y - s.planned.y;
    const double from_anchor_dy = d.anchor.y - s.planned.y;
    const double bezier_dy = d.anchor.y - untilted.y;
    const double capture_dy = -d.capture_body.y;
    const double origin_dy = untilted.y - d.origin_rot.y;
    const double origin_vs_coxa_dy = d.coxa_rot.y - d.origin_rot.y;
    const double clamp_dy = d.origin_rot.y - s.planned.y;
    const double floor_y = sampleFloorY(d);
    const double tibia = ikTibiaWire(s.planned);
    std::cout << "P0_SWING_DECOMP v3 mm_budget i=" << sample_i
              << " loop=" << s.loop
              << " planned=(" << s.planned.x << "," << s.planned.y << "," << s.planned.z << ")"
              << " min_planned_y=" << min_planned_y
              << " floor_y=" << floor_y
              << " floor_bound=" << (floor_bound ? 1 : 0)
              << " dxy_to_vin=" << planarXy(s.planned, vin_cmd)
              << " v2_dxy=" << kV2VinDxyM
              << " medial_dy_mm=" << 1000.0 * medial_dy
              << " from_anchor_dy_mm=" << 1000.0 * from_anchor_dy
              << " bezier_vs_anchor_dy_mm=" << 1000.0 * bezier_dy
              << " capture_dy_mm=" << 1000.0 * capture_dy
              << " origin_dy_mm=" << 1000.0 * origin_dy
              << " origin_vs_coxa_dy_mm=" << 1000.0 * origin_vs_coxa_dy
              << " clamp_dy_mm=" << 1000.0 * clamp_dy
              << " ik_tibia_wire=" << tibia
              << " held_far_tibia_wire=" << kHeldFarTibiaWire
              << " floor_class=" << floor_class << '\n';

    const Vec3 skip_r = clampSwing(untilted);
    const Vec3 coxa_r = clampSwing(d.coxa_rot);
    Vec3 zero_cap_untilted{};
    Vec3 zero_cap_vel{};
    SwingFootInputs sw_zero = swingInputsFromDecomp(d);
    sw_zero.static_stability_margin_m = 1.0;
    RobotState est_zero{};
    planSwingFoot(est_zero, d.kinematic_twist, sw_zero, zero_cap_untilted, zero_cap_vel);
    const Mat3 R = bodyRotationFromDecomp(d);
    const Vec3 zero_cap = clampSwing(R * zero_cap_untilted);
    printCounterfactual("skip_R", skip_r, vin_cmd, vin_tibia);
    printCounterfactual("coxa_R", coxa_r, vin_cmd, vin_tibia);
    printCounterfactual("zero_capture", zero_cap, vin_cmd, vin_tibia);
    printCounterfactual("dumped", s.planned, vin_cmd, vin_tibia);
    std::cout << "P0_SWING_DECOMP v3 classification=" << floor_class
              << " vin_dxy=" << best_dxy
              << " vin_tuck_count=" << vin_tuck_count
              << " samples=" << frozen.size() << '\n';
    return true;
}

} // namespace

int main() {
    char dump_path[] = "/tmp/hexapod-swing-decomp-unit.bin";
    (void)dump_path;
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
    gait.phase[kR2Leg] = 0.82;
    gait.in_stance[kR2Leg] = false;
    const BodyTwist twist = rawLocomotionTwistFromIntent(walk_intent, planarMotionCommand(walk_intent));
    const LegTargets targets = controller.update(est, walk_intent, gait, safety, twist);
    const R2SwingDecompSnapshot unit = controller.lastR2SwingDecomp();
    if (!unit.valid) {
        std::cerr << "p0-swing-decomp: in-process R2 swing decomp missing\n";
        return EXIT_FAILURE;
    }
    Vec3 replay_pos{};
    Vec3 replay_vel{};
    planSwingFoot(estFromDecomp(unit), unit.kinematic_twist, swingInputsFromDecomp(unit), replay_pos, replay_vel);
    const double unit_err =
        std::hypot(replay_pos.x - unit.planned_pre_rot.x, replay_pos.y - unit.planned_pre_rot.y);
    if (unit_err > kReplayMatchM) {
        std::cerr << "p0-swing-decomp: in-process identity failed xy_err=" << unit_err << '\n';
        return EXIT_FAILURE;
    }
    const Mat3 unit_R = bodyRotationFromDecomp(unit);
    const Vec3 unit_origin = unit_R * unit.after_terrain;
    const Vec3 unit_coxa = unit.coxa + (unit_R * (unit.after_terrain - unit.coxa));
    if (planarXy(unit_origin, unit.origin_rot) > kReplayMatchM
        || planarXy(unit_coxa, unit.coxa_rot) > kReplayMatchM) {
        std::cerr << "p0-swing-decomp: in-process origin/coxa reconstruction missed\n";
        return EXIT_FAILURE;
    }
    (void)targets;
    std::cout << "P0_SWING_DECOMP in_process valid=1 planned_y=" << unit.target_clamped.y
              << " untilted_y=" << unit.after_terrain.y
              << " origin_rot_y=" << unit.origin_rot.y
              << " coxa_rot_y=" << unit.coxa_rot.y
              << " clamp_dxy=" << unit.clamp_dxy << '\n';

    const char* history_path = HEXAPOD_P3_SEQ_HISTORY_FIXTURE;
    std::string history_text;
    Vec3 vin_cmd{};
    double vin_tibia = 0.0;
    if (history_path == nullptr || history_path[0] == '\0' || !loadFile(history_path, history_text)
        || !parseVinLanding(history_text, vin_cmd, vin_tibia)) {
        std::cerr << "p0-swing-decomp: failed to load history "
                  << (history_path != nullptr ? history_path : "(null)") << '\n';
        return EXIT_FAILURE;
    }

    const char* frozen_path = HEXAPOD_P0_SWING_DECOMP_FIXTURE;
    if (frozen_path == nullptr || frozen_path[0] == '\0') {
        std::cout << "P0_SWING_DECOMP frozen classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::ifstream exists(frozen_path);
    if (!exists.good()) {
        std::cout << "P0_SWING_DECOMP frozen classification=missing_fixture path=" << frozen_path << '\n';
        return EXIT_SUCCESS;
    }
    exists.close();
    std::string frozen_text;
    std::vector<DecompSample> frozen;
    if (!loadFile(frozen_path, frozen_text) || !parseDecompDump(frozen_text, frozen)) {
        std::cout << "P0_SWING_DECOMP frozen classification=missing_fixture path=" << frozen_path << '\n';
        return EXIT_SUCCESS;
    }
    if (!replayIdentity(frozen)) {
        return EXIT_FAILURE;
    }

    std::size_t latch_i = frozen.size();
    for (std::size_t i = 0; i < frozen.size(); ++i) {
        if (frozen[i].latched) {
            latch_i = i;
            break;
        }
    }
    std::size_t sample_i = frozen.size();
    double best_dxy = 1.0e9;
    double min_vin_y = 1.0e9;
    std::size_t vin_tuck_count = 0;
    const auto is_vin_tuck = [&](const DecompSample& sample) {
        return sample.decomp_valid
            && (sample.planned.y <= kVinTuckYM || planarXy(sample.planned, vin_cmd) < kMatchVinM);
    };
    for (std::size_t i = 0; i < frozen.size(); ++i) {
        if (!is_vin_tuck(frozen[i])) {
            continue;
        }
        ++vin_tuck_count;
        const double d = planarXy(frozen[i].planned, vin_cmd);
        if (d < best_dxy || (d == best_dxy && frozen[i].planned.y < min_vin_y)) {
            best_dxy = d;
            min_vin_y = frozen[i].planned.y;
            sample_i = i;
        }
    }
    if (latch_i < frozen.size() && is_vin_tuck(frozen[latch_i])
        && planarXy(frozen[latch_i].planned, vin_cmd) <= best_dxy + 1.0e-9) {
        sample_i = latch_i;
        best_dxy = planarXy(frozen[latch_i].planned, vin_cmd);
    }
    if (sample_i >= frozen.size()) {
        std::cout << "P0_SWING_DECOMP frozen classification=not_vin_tuck samples=" << frozen.size()
                  << " latch_i=" << latch_i
                  << " vin_tuck_count=" << vin_tuck_count << '\n';
        return EXIT_SUCCESS;
    }

    const DecompSample& s = frozen[sample_i];
    const R2SwingDecompSnapshot& d = s.decomp;
    const Vec3 untilted = d.after_terrain;
    const double medial_dy = s.nominal.y - s.planned.y;
    const double anchor_vs_nominal_dy = s.nominal.y - d.anchor.y;
    const double from_anchor_dy = d.anchor.y - s.planned.y;
    const double bezier_dy = d.anchor.y - untilted.y;
    const double capture_dy = -d.capture_body.y;
    const double origin_dy = untilted.y - d.origin_rot.y;
    const double origin_vs_coxa_dy = d.coxa_rot.y - d.origin_rot.y;
    const double clamp_dy = d.origin_rot.y - s.planned.y;
    const double terrain_dy = -d.terrain_xy_delta.y;
    const double denom = std::max(std::abs(from_anchor_dy), 1.0e-6);
    const double untilted_frac = bezier_dy / denom;
    const double origin_frac = origin_dy / denom;
    const double clamp_frac = clamp_dy / denom;
    const char* winner = "mixed";
    if (untilted.y <= kVinTuckYM && untilted_frac >= kWinnerFrac) {
        winner = "untilted";
    } else if (untilted_frac >= kWinnerFrac) {
        winner = "untilted";
    } else if (origin_frac >= kWinnerFrac) {
        winner = "origin_rot";
    } else if (clamp_frac >= kWinnerFrac) {
        winner = "clamp";
    }
    std::cout << "P0_SWING_DECOMP mm_budget i=" << sample_i
              << " loop=" << s.loop
              << " phase=" << s.phase
              << " tau01=" << d.tau01
              << " planned=(" << s.planned.x << "," << s.planned.y << "," << s.planned.z << ")"
              << " untilted=(" << untilted.x << "," << untilted.y << "," << untilted.z << ")"
              << " origin_rot=(" << d.origin_rot.x << "," << d.origin_rot.y << "," << d.origin_rot.z << ")"
              << " coxa_rot=(" << d.coxa_rot.x << "," << d.coxa_rot.y << "," << d.coxa_rot.z << ")"
              << " anchor=(" << d.anchor.x << "," << d.anchor.y << "," << d.anchor.z << ")"
              << " d_nominal=" << planarXy(s.planned, s.nominal)
              << " dxy_to_vin=" << planarXy(s.planned, vin_cmd)
              << " medial_dy_mm=" << 1000.0 * medial_dy
              << " from_anchor_dy_mm=" << 1000.0 * from_anchor_dy
              << " anchor_vs_nominal_dy_mm=" << 1000.0 * anchor_vs_nominal_dy
              << " bezier_vs_anchor_dy_mm=" << 1000.0 * bezier_dy
              << " capture_dy_mm=" << 1000.0 * capture_dy
              << " capture_limit_mm=" << 1000.0 * d.capture_limit_m
              << " origin_dy_mm=" << 1000.0 * origin_dy
              << " origin_vs_coxa_dy_mm=" << 1000.0 * origin_vs_coxa_dy
              << " clamp_dy_mm=" << 1000.0 * clamp_dy
              << " clamp_dxy_mm=" << 1000.0 * d.clamp_dxy
              << " terrain_dy_mm=" << 1000.0 * terrain_dy
              << " roll_rad=" << d.roll_rad
              << " pitch_rad=" << d.pitch_rad
              << " winner=" << winner << '\n';

    if (std::string(winner) == "origin_rot") {
        const bool origin_medial = (d.anchor.y - d.origin_rot.y) > kMedialDyM;
        const bool coxa_medial = (d.anchor.y - d.coxa_rot.y) > kMedialDyM;
        if (!(origin_medial && !coxa_medial)) {
            std::cerr << "p0-swing-decomp: origin-rot winner expected coxa-rot outside 40 mm medial of anchor\n";
            return EXIT_FAILURE;
        }
    }

    const Vec3 skip_r = clampSwing(untilted);
    const Vec3 coxa_r = clampSwing(d.coxa_rot);
    Vec3 zero_cap_untilted{};
    Vec3 zero_cap_vel{};
    SwingFootInputs sw_zero = swingInputsFromDecomp(d);
    sw_zero.static_stability_margin_m = 1.0;
    RobotState est_zero{};
    planSwingFoot(est_zero, d.kinematic_twist, sw_zero, zero_cap_untilted, zero_cap_vel);
    const Mat3 R = bodyRotationFromDecomp(d);
    const Vec3 zero_cap_origin = R * zero_cap_untilted;
    const Vec3 zero_cap = clampSwing(zero_cap_origin);
    printCounterfactual("skip_R", skip_r, vin_cmd, vin_tibia);
    printCounterfactual("coxa_R", coxa_r, vin_cmd, vin_tibia);
    printCounterfactual("zero_capture", zero_cap, vin_cmd, vin_tibia);
    printCounterfactual("dumped", s.planned, vin_cmd, vin_tibia);

    std::cout << "P0_SWING_DECOMP frozen classification=" << winner
              << " vin_dxy=" << best_dxy
              << " vin_tuck_count=" << vin_tuck_count
              << " samples=" << frozen.size()
              << " latch_i=" << latch_i << '\n';

    const char* v3_path = HEXAPOD_P0_SWING_DECOMP_V3_FIXTURE;
    if (v3_path == nullptr || v3_path[0] == '\0') {
        std::cout << "P0_SWING_DECOMP v3 classification=missing_fixture\n";
        return EXIT_SUCCESS;
    }
    std::ifstream v3_exists(v3_path);
    if (!v3_exists.good()) {
        std::cout << "P0_SWING_DECOMP v3 classification=missing_fixture path=" << v3_path << '\n';
        return EXIT_SUCCESS;
    }
    v3_exists.close();
    std::string v3_text;
    std::vector<DecompSample> v3;
    if (!loadFile(v3_path, v3_text) || !parseDecompDump(v3_text, v3)) {
        std::cout << "P0_SWING_DECOMP v3 classification=missing_fixture path=" << v3_path << '\n';
        return EXIT_SUCCESS;
    }
    if (!scoreV3Dump(v3, vin_cmd, vin_tibia)) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
