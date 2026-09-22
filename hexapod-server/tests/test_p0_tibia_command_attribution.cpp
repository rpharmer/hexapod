// Offline gait/IK attribution of the frozen p3-seq held R2 tibia command.
// Diagnose-only: no gait, governor, Kd, or plant change.

#include "body_controller.hpp"
#include "control_pipeline.hpp"
#include "foot_reachability.hpp"
#include "geometry_config.hpp"
#include "leg_fk.hpp"
#include "leg_ik.hpp"
#include "physics_sim_joint_wire_mapping.hpp"
#include "physics_sim_protocol.hpp"
#include "types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifndef HEXAPOD_P3_SEQ_HISTORY_FIXTURE
#define HEXAPOD_P3_SEQ_HISTORY_FIXTURE ""
#endif

namespace {

constexpr std::size_t kR2Leg = 2;
constexpr std::size_t kR2CoxaWire = 6;
constexpr std::size_t kR2FemurWire = 7;
constexpr std::size_t kR2TibiaWire = 8;
constexpr double kCartesianFarM = 0.015;
constexpr double kIkTibiaMatchRad = 1.0e-3;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kCmdFarFromNominalM = 0.040;
constexpr double kLiveNearNominalM = 0.030;
constexpr double kClampMovedXyM = 0.005;

Vec3 toVec3(const PositionM3& p) {
    return Vec3{p.x, p.y, p.z};
}

double planarXy(const Vec3& a, const Vec3& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

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
        if (i < text.size() && text[i] == '[') {
            std::vector<double> nested;
            if (!parseJsonNumberArray(text, i, nested)) {
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

struct HistorySample {
    std::uint8_t load_bearing_mask = 0;
    std::array<double, 18> targets{};
    std::array<double, 18> errors{};
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
        double mask_value = 0.0;
        if (extractNumber(text, "load_bearing_mask", object_start, object_end, mask_value)) {
            sample.load_bearing_mask = static_cast<std::uint8_t>(mask_value);
        }
        std::vector<double> targets;
        std::vector<double> errors;
        if (!extractArray(text, "targets", object_start, object_end, targets) || targets.size() != 18) {
            return false;
        }
        std::copy(targets.begin(), targets.end(), sample.targets.begin());
        if (!extractArray(text, "errors", object_start, object_end, errors) || errors.size() != 18) {
            return false;
        }
        std::copy(errors.begin(), errors.end(), sample.errors.begin());
        out.push_back(sample);
        i = object_end;
        skipWs(text, i);
        if (i < text.size() && text[i] == ',') {
            ++i;
        }
    }
    return !out.empty();
}

double liveWire(double target, double error) {
    return std::remainder(target - error, kTwoPi);
}

double mechanicalTibiaFromWire(double wire) {
    return static_cast<double>(physics_sim::kWireZeroTibiaMechanicalRad) + wire;
}

const char* classifyVinLanding(bool loaded, bool cartesian_far, bool tibia_error_large) {
    if (loaded) {
        return "stance_loaded_command";
    }
    if (cartesian_far) {
        return "cartesian_command_far";
    }
    if (tibia_error_large) {
        return "held_joint_setpoint";
    }
    return "held_joint_setpoint";
}

const char* classifySwingXy(
    bool loaded,
    bool cmd_in_annulus,
    bool live_in_annulus,
    double clamp_dxy,
    double d_nominal_cmd,
    double d_nominal_live) {
    if (!cmd_in_annulus || clamp_dxy >= kClampMovedXyM) {
        return "reach_bound_command";
    }
    if (d_nominal_live > kLiveNearNominalM) {
        return "live_also_far";
    }
    if (!loaded && d_nominal_cmd > kCmdFarFromNominalM && live_in_annulus) {
        return "medial_cmd_near_nominal_live";
    }
    return "live_also_far";
}

} // namespace

int main() {
    const char* path = HEXAPOD_P3_SEQ_HISTORY_FIXTURE;
    std::string text;
    std::vector<HistorySample> samples;
    if (path == nullptr || path[0] == '\0' || !loadFile(path, text) || !parseAcceptedHistory(text, samples)
        || samples.size() < 2) {
        std::cerr << "p0-tibia-ik: failed to load " << (path != nullptr ? path : "(null)") << '\n';
        return EXIT_FAILURE;
    }

    const HexapodGeometry& geo = geometry_config::buildDefaultHexapodGeometry();
    const ServoCalibration& cal = geo.legGeometry[kR2Leg].servo;
    const LegGeometry& leg_geo = geo.legGeometry[kR2Leg];
    LegFK fk{};
    LegIK ik{geo};

    ControlPipeline pipeline;
    RobotState estimated{};
    estimated.timestamp_us = now_us();
    MotionIntent stand_intent{};
    stand_intent.requested_mode = RobotMode::STAND;
    stand_intent.timestamp_us = now_us();
    SafetyState safety{};
    safety.inhibit_motion = false;
    const PipelineStepResult stand = pipeline.runStep(estimated, stand_intent, safety, true, 1);
    float stand_c = 0.0f;
    float stand_f = 0.0f;
    float stand_t = 0.0f;
    physics_sim_joint_wire_mapping::simWireTargetsFromServoLeg(
        cal, static_cast<int>(kR2Leg), stand.joint_targets.leg_states[kR2Leg], stand_c, stand_f, stand_t);
    const FootTarget stand_foot =
        fk.footInBodyFrame(stand.joint_targets.leg_states[kR2Leg], leg_geo);
    std::cout << "P0_TIBIA_IK stand_ref wire8=" << stand_t
              << " mech8=" << mechanicalTibiaFromWire(static_cast<double>(stand_t))
              << " foot_z=" << stand_foot.pos_body_m.z << '\n';

    bool ik_inconsistent = false;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const HistorySample& sample = samples[i];
        const double cmd_c = sample.targets[kR2CoxaWire];
        const double cmd_f = sample.targets[kR2FemurWire];
        const double cmd_t = sample.targets[kR2TibiaWire];
        const double live_c = liveWire(sample.targets[kR2CoxaWire], sample.errors[kR2CoxaWire]);
        const double live_f = liveWire(sample.targets[kR2FemurWire], sample.errors[kR2FemurWire]);
        const double live_t = liveWire(sample.targets[kR2TibiaWire], sample.errors[kR2TibiaWire]);
        const double d_target8 = (i == 0) ? 0.0 : cmd_t - samples[i - 1].targets[kR2TibiaWire];

        const LegState cmd_servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
            cal, static_cast<int>(kR2Leg), static_cast<float>(cmd_c), static_cast<float>(cmd_f),
            static_cast<float>(cmd_t));
        const LegState live_servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
            cal, static_cast<int>(kR2Leg), static_cast<float>(live_c), static_cast<float>(live_f),
            static_cast<float>(live_t));
        const FootTarget cmd_foot = fk.footInBodyFrame(cmd_servo, leg_geo);
        const FootTarget live_foot = fk.footInBodyFrame(live_servo, leg_geo);
        const double dxy = std::hypot(cmd_foot.pos_body_m.x - live_foot.pos_body_m.x,
                                      cmd_foot.pos_body_m.y - live_foot.pos_body_m.y);
        const double dz = cmd_foot.pos_body_m.z - live_foot.pos_body_m.z;

        LegTargets ik_targets{};
        ik_targets.feet[kR2Leg] = cmd_foot;
        const JointTargets recovered = ik.solve(estimated, ik_targets, safety);
        const bool reach_clamped = ik.lastReachClampHit()[kR2Leg];
        const double recovered_tibia = recovered.leg_states[kR2Leg].joint_state[TIBIA].pos_rad.value;
        const double commanded_tibia = cmd_servo.joint_state[TIBIA].pos_rad.value;
        if (std::abs(shortestAngleDeltaRad(commanded_tibia, recovered_tibia)) > kIkTibiaMatchRad) {
            std::cerr << "p0-tibia-ik: IK(FK(commanded)) missed tibia sample=" << i
                      << " cmd=" << commanded_tibia << " recovered=" << recovered_tibia << '\n';
            ik_inconsistent = true;
        }

        const bool loaded = (sample.load_bearing_mask & static_cast<std::uint8_t>(1u << kR2Leg)) != 0;
        std::cout << "P0_TIBIA_IK sample=" << i
                  << " wire8_cmd=" << cmd_t
                  << " wire8_live=" << live_t
                  << " mech8_cmd=" << mechanicalTibiaFromWire(cmd_t)
                  << " mech8_live=" << mechanicalTibiaFromWire(live_t)
                  << " error8=" << sample.errors[kR2TibiaWire]
                  << " d_target8=" << d_target8
                  << " foot_cmd=(" << cmd_foot.pos_body_m.x << "," << cmd_foot.pos_body_m.y << ","
                  << cmd_foot.pos_body_m.z << ")"
                  << " foot_live=(" << live_foot.pos_body_m.x << "," << live_foot.pos_body_m.y << ","
                  << live_foot.pos_body_m.z << ")"
                  << " dxy=" << dxy
                  << " dz=" << dz
                  << " mask_bit2=" << (loaded ? 1 : 0)
                  << " reach_clamped=" << (reach_clamped ? 1 : 0) << '\n';

        const Vec3 cmd_body = toVec3(cmd_foot.pos_body_m);
        const Vec3 live_body = toVec3(live_foot.pos_body_m);
        const double body_height_m = std::max(0.04, -cmd_foot.pos_body_m.z);
        const Vec3 nominal = computeNominalStance(geo, body_height_m)[kR2Leg];
        const Vec3 hip = toVec3(leg_geo.bodyCoxaOffset);
        const double r_hip_cmd = planarXy(cmd_body, hip);
        const double r_hip_live = planarXy(live_body, hip);
        const double d_nominal_cmd = planarXy(cmd_body, nominal);
        const double d_nominal_live = planarXy(live_body, nominal);
        const double femur_d_cmd = foot_reachability::femurPlaneDistanceM(leg_geo, cmd_body);
        const double femur_d_live = foot_reachability::femurPlaneDistanceM(leg_geo, live_body);
        const bool cmd_in_annulus = foot_reachability::footInReachAnnulus(leg_geo, cmd_body);
        const bool live_in_annulus = foot_reachability::footInReachAnnulus(leg_geo, live_body);
        const Vec3 clamped_cmd = foot_reachability::clampFootPositionBody(leg_geo, cmd_body);
        const double clamp_dxy = planarXy(clamped_cmd, cmd_body);
        std::cout << "P0_SWING_XY sample=" << i
                  << " nominal=(" << nominal.x << "," << nominal.y << "," << nominal.z << ")"
                  << " d_nominal_cmd=" << d_nominal_cmd
                  << " d_nominal_live=" << d_nominal_live
                  << " r_hip_cmd=" << r_hip_cmd
                  << " r_hip_live=" << r_hip_live
                  << " femur_d_cmd=" << femur_d_cmd
                  << " femur_d_live=" << femur_d_live
                  << " cmd_in_annulus=" << (cmd_in_annulus ? 1 : 0)
                  << " live_in_annulus=" << (live_in_annulus ? 1 : 0)
                  << " clamp_dxy=" << clamp_dxy
                  << " dxy=" << dxy
                  << " mask_bit2=" << (loaded ? 1 : 0) << '\n';
    }

    if (ik_inconsistent) {
        std::cout << "P0_TIBIA_IK classification=ik_inconsistent samples=" << samples.size() << '\n';
        return EXIT_FAILURE;
    }

    const HistorySample& vin = samples.back();
    const double cmd_c = vin.targets[kR2CoxaWire];
    const double cmd_f = vin.targets[kR2FemurWire];
    const double cmd_t = vin.targets[kR2TibiaWire];
    const double live_c = liveWire(vin.targets[kR2CoxaWire], vin.errors[kR2CoxaWire]);
    const double live_f = liveWire(vin.targets[kR2FemurWire], vin.errors[kR2FemurWire]);
    const double live_t = liveWire(vin.targets[kR2TibiaWire], vin.errors[kR2TibiaWire]);
    const LegState cmd_servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
        cal, static_cast<int>(kR2Leg), static_cast<float>(cmd_c), static_cast<float>(cmd_f),
        static_cast<float>(cmd_t));
    const LegState live_servo = physics_sim_joint_wire_mapping::servoLegFromSimWireAngles(
        cal, static_cast<int>(kR2Leg), static_cast<float>(live_c), static_cast<float>(live_f),
        static_cast<float>(live_t));
    const FootTarget cmd_foot = fk.footInBodyFrame(cmd_servo, leg_geo);
    const FootTarget live_foot = fk.footInBodyFrame(live_servo, leg_geo);
    const double dxy = std::hypot(cmd_foot.pos_body_m.x - live_foot.pos_body_m.x,
                                  cmd_foot.pos_body_m.y - live_foot.pos_body_m.y);
    const double dz = cmd_foot.pos_body_m.z - live_foot.pos_body_m.z;
    const bool loaded = (vin.load_bearing_mask & static_cast<std::uint8_t>(1u << kR2Leg)) != 0;
    const bool cartesian_far = dxy >= kCartesianFarM || std::abs(dz) >= kCartesianFarM;
    const bool tibia_error_large = std::abs(vin.errors[kR2TibiaWire]) >= 0.20;
    const char* classification = classifyVinLanding(loaded, cartesian_far, tibia_error_large);
    std::cout << "P0_TIBIA_IK classification=" << classification
              << " vin_sample=" << (samples.size() - 1)
              << " dxy=" << dxy
              << " dz=" << dz
              << " error8=" << vin.errors[kR2TibiaWire]
              << " mask_bit2=" << (loaded ? 1 : 0)
              << " samples=" << samples.size() << '\n';

    const Vec3 vin_cmd = toVec3(cmd_foot.pos_body_m);
    const Vec3 vin_live = toVec3(live_foot.pos_body_m);
    const double vin_height = std::max(0.04, -cmd_foot.pos_body_m.z);
    const Vec3 vin_nominal = computeNominalStance(geo, vin_height)[kR2Leg];
    const double d_nominal_cmd = planarXy(vin_cmd, vin_nominal);
    const double d_nominal_live = planarXy(vin_live, vin_nominal);
    const bool cmd_in_annulus = foot_reachability::footInReachAnnulus(leg_geo, vin_cmd);
    const bool live_in_annulus = foot_reachability::footInReachAnnulus(leg_geo, vin_live);
    const Vec3 clamped_cmd = foot_reachability::clampFootPositionBody(leg_geo, vin_cmd);
    const double clamp_dxy = planarXy(clamped_cmd, vin_cmd);
    const char* xy_class =
        classifySwingXy(loaded, cmd_in_annulus, live_in_annulus, clamp_dxy, d_nominal_cmd, d_nominal_live);
    std::cout << "P0_SWING_XY classification=" << xy_class
              << " vin_sample=" << (samples.size() - 1)
              << " d_nominal_cmd=" << d_nominal_cmd
              << " d_nominal_live=" << d_nominal_live
              << " cmd_in_annulus=" << (cmd_in_annulus ? 1 : 0)
              << " live_in_annulus=" << (live_in_annulus ? 1 : 0)
              << " clamp_dxy=" << clamp_dxy
              << " dxy=" << dxy
              << " mask_bit2=" << (loaded ? 1 : 0) << '\n';
    return EXIT_SUCCESS;
}
