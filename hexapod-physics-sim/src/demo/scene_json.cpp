#include "demo/scene_json.hpp"

#include "demo/demo_run_control.hpp"
#include "demo/terrain_patch.hpp"

#include "minphys3d/core/body.hpp"
#include "minphys3d/math/quat.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>

namespace minphys3d::demo {

using minphys3d::Body;
using minphys3d::CompoundChild;
using minphys3d::IsCompoundChildShapeSupported;
using minphys3d::Contact;
using minphys3d::Length;
using minphys3d::Manifold;
using minphys3d::Normalize;
using minphys3d::Quat;
using minphys3d::ServoJoint;
using minphys3d::ShapeType;
using minphys3d::Vec3;
using minphys3d::World;

namespace {

using JsonObject = std::vector<std::pair<std::string, struct JsonValue>>;

struct JsonValue {
    enum Type { Null, Bool, Number, String, Array, Object } type = Null;
    bool b = false;
    double number = 0.0;
    std::string str;
    std::vector<JsonValue> array;
    JsonObject object;
};

class JsonParser {
public:
    explicit JsonParser(std::string_view text) : s_(text) {}

    bool parse(JsonValue& out, std::string& err) {
        if (!parseValue(out, err)) {
            return false;
        }
        ws();
        if (i_ != s_.size()) {
            err = "trailing junk after JSON root";
            return false;
        }
        return true;
    }

private:
    std::string_view s_;
    std::size_t i_ = 0;

    void ws() {
        while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) {
            ++i_;
        }
    }

    bool matchLiteral(std::string_view lit) {
        if (i_ + lit.size() > s_.size()) {
            return false;
        }
        for (std::size_t k = 0; k < lit.size(); ++k) {
            if (s_[i_ + k] != lit[k]) {
                return false;
            }
        }
        i_ += lit.size();
        return true;
    }

    bool parseValue(JsonValue& out, std::string& err) {
        ws();
        if (i_ >= s_.size()) {
            err = "unexpected end of input";
            return false;
        }
        const char c = s_[i_];
        if (c == '{') {
            out.type = JsonValue::Object;
            return parseObject(out.object, err);
        }
        if (c == '[') {
            out.type = JsonValue::Array;
            return parseArray(out.array, err);
        }
        if (c == '"') {
            out.type = JsonValue::String;
            return parseString(out.str, err);
        }
        if (c == '-' || (c >= '0' && c <= '9')) {
            out.type = JsonValue::Number;
            return parseNumber(out.number, err);
        }
        if (matchLiteral("null")) {
            out.type = JsonValue::Null;
            return true;
        }
        if (matchLiteral("true")) {
            out.type = JsonValue::Bool;
            out.b = true;
            return true;
        }
        if (matchLiteral("false")) {
            out.type = JsonValue::Bool;
            out.b = false;
            return true;
        }
        err = "invalid JSON value start";
        return false;
    }

    bool parseNumber(double& out, std::string& err) {
        const std::size_t start = i_;
        if (i_ < s_.size() && s_[i_] == '-') {
            ++i_;
        }
        while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) {
            ++i_;
        }
        if (i_ < s_.size() && s_[i_] == '.') {
            ++i_;
            while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) {
                ++i_;
            }
        }
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            ++i_;
            if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) {
                ++i_;
            }
            while (i_ < s_.size() && std::isdigit(static_cast<unsigned char>(s_[i_]))) {
                ++i_;
            }
        }
        if (i_ == start) {
            err = "malformed number";
            return false;
        }
        try {
            out = std::stod(std::string(s_.substr(start, i_ - start)));
        } catch (...) {
            err = "number parse failed";
            return false;
        }
        return true;
    }

    bool parseString(std::string& out, std::string& err) {
        if (i_ >= s_.size() || s_[i_] != '"') {
            err = "expected '\"' for string";
            return false;
        }
        ++i_;
        out.clear();
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') {
                return true;
            }
            if (c == '\\') {
                if (i_ >= s_.size()) {
                    err = "unterminated escape in string";
                    return false;
                }
                const char e = s_[i_++];
                switch (e) {
                    case '"':
                        out.push_back('"');
                        break;
                    case '\\':
                        out.push_back('\\');
                        break;
                    case '/':
                        out.push_back('/');
                        break;
                    case 'b':
                        out.push_back('\b');
                        break;
                    case 'f':
                        out.push_back('\f');
                        break;
                    case 'n':
                        out.push_back('\n');
                        break;
                    case 'r':
                        out.push_back('\r');
                        break;
                    case 't':
                        out.push_back('\t');
                        break;
                    default:
                        err = "unsupported string escape";
                        return false;
                }
            } else {
                out.push_back(c);
            }
        }
        err = "unterminated string";
        return false;
    }

    bool parseArray(std::vector<JsonValue>& out, std::string& err) {
        if (i_ >= s_.size() || s_[i_] != '[') {
            err = "expected '['";
            return false;
        }
        ++i_;
        ws();
        out.clear();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        for (;;) {
            JsonValue el;
            if (!parseValue(el, err)) {
                return false;
            }
            out.push_back(std::move(el));
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                ws();
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                ++i_;
                return true;
            }
            err = "expected ',' or ']' in array";
            return false;
        }
    }

    bool parseObject(JsonObject& out, std::string& err) {
        if (i_ >= s_.size() || s_[i_] != '{') {
            err = "expected '{'";
            return false;
        }
        ++i_;
        ws();
        out.clear();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        for (;;) {
            ws();
            std::string key;
            if (!parseString(key, err)) {
                return false;
            }
            ws();
            if (i_ >= s_.size() || s_[i_] != ':') {
                err = "expected ':' after object key";
                return false;
            }
            ++i_;
            JsonValue val;
            if (!parseValue(val, err)) {
                return false;
            }
            out.push_back({std::move(key), std::move(val)});
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                ++i_;
                return true;
            }
            err = "expected ',' or '}' in object";
            return false;
        }
    }
};

std::string StripLineComments(std::string_view in) {
    std::string out;
    out.reserve(in.size());
    bool in_string = false;
    bool escape = false;
    for (std::size_t j = 0; j < in.size(); ++j) {
        const char c = in[j];
        if (escape) {
            out.push_back(c);
            escape = false;
            continue;
        }
        if (in_string) {
            if (c == '\\') {
                escape = true;
                out.push_back(c);
            } else if (c == '"') {
                in_string = false;
                out.push_back(c);
            } else {
                out.push_back(c);
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            out.push_back(c);
            continue;
        }
        if (c == '/' && j + 1 < in.size() && in[j + 1] == '/') {
            while (++j < in.size() && in[j] != '\n') {
            }
            if (j < in.size()) {
                out.push_back('\n');
            }
            continue;
        }
        out.push_back(c);
    }
    return out;
}

const JsonValue* FindMember(const JsonObject& o, const std::string& key) {
    for (const auto& p : o) {
        if (p.first == key) {
            return &p.second;
        }
    }
    return nullptr;
}

bool GetNumber(const JsonObject& o, const std::string& key, double& out, std::string& err, bool required) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        if (required) {
            err = "missing number field: " + key;
            return false;
        }
        return false;
    }
    if (v->type != JsonValue::Number) {
        err = "field not a number: " + key;
        return false;
    }
    out = v->number;
    return true;
}

bool GetString(const JsonObject& o, const std::string& key, std::string& out, std::string& err, bool required) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        if (required) {
            err = "missing string field: " + key;
            return false;
        }
        return false;
    }
    if (v->type != JsonValue::String) {
        err = "field not a string: " + key;
        return false;
    }
    out = v->str;
    return true;
}

bool GetVec3(const JsonObject& o, const std::string& key, Vec3& out, std::string& err, const Vec3& defaultValue) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        out = defaultValue;
        return true;
    }
    if (v->type != JsonValue::Array || v->array.size() != 3) {
        err = "expected length-3 array for " + key;
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        if (v->array[static_cast<std::size_t>(k)].type != JsonValue::Number) {
            err = "non-numeric component in " + key;
            return false;
        }
    }
    out.x = static_cast<float>(v->array[0].number);
    out.y = static_cast<float>(v->array[1].number);
    out.z = static_cast<float>(v->array[2].number);
    return true;
}

bool GetBool(const JsonObject& o, const std::string& key, bool& out, bool defaultValue, std::string& err) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        out = defaultValue;
        return true;
    }
    if (v->type != JsonValue::Bool) {
        err = "field not a boolean: " + key;
        return false;
    }
    out = v->b;
    return true;
}

bool RequireVec3(const JsonObject& o, const std::string& key, Vec3& out, std::string& err) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        err = "missing vec3 field: " + key;
        return false;
    }
    if (v->type != JsonValue::Array || v->array.size() != 3) {
        err = "expected length-3 array for " + key;
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        if (v->array[static_cast<std::size_t>(k)].type != JsonValue::Number) {
            err = "non-numeric component in " + key;
            return false;
        }
    }
    out.x = static_cast<float>(v->array[0].number);
    out.y = static_cast<float>(v->array[1].number);
    out.z = static_cast<float>(v->array[2].number);
    return true;
}

bool GetQuat(const JsonObject& o, const std::string& key, Quat& out, std::string& err) {
    const JsonValue* v = FindMember(o, key);
    if (!v) {
        out = Quat{1.0f, 0.0f, 0.0f, 0.0f};
        return true;
    }
    if (v->type != JsonValue::Array || v->array.size() != 4) {
        err = "expected length-4 array [w,x,y,z] for " + key;
        return false;
    }
    for (int k = 0; k < 4; ++k) {
        if (v->array[static_cast<std::size_t>(k)].type != JsonValue::Number) {
            err = "non-numeric component in " + key;
            return false;
        }
    }
    out.w = static_cast<float>(v->array[0].number);
    out.x = static_cast<float>(v->array[1].number);
    out.y = static_cast<float>(v->array[2].number);
    out.z = static_cast<float>(v->array[3].number);
    out = Normalize(out);
    return true;
}

bool GetBodyIndex(const JsonObject& o, const std::string& key, std::uint32_t bodyCount, std::uint32_t& out, std::string& err) {
    const JsonValue* v = FindMember(o, key);
    if (!v || v->type != JsonValue::Number) {
        err = "missing or non-numeric body index: " + key;
        return false;
    }
    const double d = v->number;
    if (d < 0.0 || d != std::floor(d)) {
        err = "body index must be a non-negative integer: " + key;
        return false;
    }
    const auto idx = static_cast<std::uint32_t>(d);
    if (idx >= bodyCount) {
        err = "body index " + key + " out of range (body count " + std::to_string(bodyCount) + ")";
        return false;
    }
    out = idx;
    return true;
}

void ToLowerAscii(std::string& s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
}

bool ParseJointObject(const JsonObject& o, World& world, std::string& err, std::uint32_t body_index_offset = 0) {
    std::string typeStr;
    if (!GetString(o, "type", typeStr, err, true)) {
        return false;
    }
    ToLowerAscii(typeStr);

    const std::uint32_t bodyCount = world.GetBodyCount();
    const std::uint32_t appendedBodyCount =
        bodyCount >= body_index_offset ? (bodyCount - body_index_offset) : 0;
    std::uint32_t bodyA = 0;
    std::uint32_t bodyB = 0;
    if (!GetBodyIndex(o, "body_a", appendedBodyCount, bodyA, err)) {
        return false;
    }
    if (!GetBodyIndex(o, "body_b", appendedBodyCount, bodyB, err)) {
        return false;
    }
    bodyA += body_index_offset;
    bodyB += body_index_offset;

    if (typeStr == "distance") {
        Vec3 anchorA{};
        Vec3 anchorB{};
        if (!RequireVec3(o, "anchor_a", anchorA, err)) {
            return false;
        }
        if (!RequireVec3(o, "anchor_b", anchorB, err)) {
            return false;
        }
        double stiffness = 1.0;
        (void)GetNumber(o, "stiffness", stiffness, err, false);
        double damping = 0.1;
        (void)GetNumber(o, "damping", damping, err, false);
        world.CreateDistanceJoint(
            bodyA,
            bodyB,
            anchorA,
            anchorB,
            static_cast<float>(stiffness),
            static_cast<float>(damping));
        return true;
    }

    if (typeStr == "hinge") {
        Vec3 anchor{};
        if (!RequireVec3(o, "anchor", anchor, err)) {
            return false;
        }
        Vec3 axis{0.0f, 1.0f, 0.0f};
        if (!GetVec3(o, "axis", axis, err, axis)) {
            return false;
        }
        axis = Normalize(axis);
        bool enableLimits = false;
        if (!GetBool(o, "enable_limits", enableLimits, false, err)) {
            return false;
        }
        double lowerAngle = 0.0;
        (void)GetNumber(o, "lower_angle", lowerAngle, err, false);
        double upperAngle = 0.0;
        (void)GetNumber(o, "upper_angle", upperAngle, err, false);
        bool enableMotor = false;
        if (!GetBool(o, "enable_motor", enableMotor, false, err)) {
            return false;
        }
        double motorSpeed = 0.0;
        (void)GetNumber(o, "motor_speed", motorSpeed, err, false);
        double maxMotorTorque = 0.0;
        (void)GetNumber(o, "max_motor_torque", maxMotorTorque, err, false);
        world.CreateHingeJoint(
            bodyA,
            bodyB,
            anchor,
            axis,
            enableLimits,
            static_cast<float>(lowerAngle),
            static_cast<float>(upperAngle),
            enableMotor,
            static_cast<float>(motorSpeed),
            static_cast<float>(maxMotorTorque));
        return true;
    }

    if (typeStr == "ball_socket" || typeStr == "ballsocket") {
        Vec3 anchor{};
        if (!RequireVec3(o, "anchor", anchor, err)) {
            return false;
        }
        world.CreateBallSocketJoint(bodyA, bodyB, anchor);
        return true;
    }

    if (typeStr == "fixed") {
        Vec3 anchor{};
        if (!RequireVec3(o, "anchor", anchor, err)) {
            return false;
        }
        world.CreateFixedJoint(bodyA, bodyB, anchor);
        return true;
    }

    if (typeStr == "prismatic") {
        Vec3 anchor{};
        if (!RequireVec3(o, "anchor", anchor, err)) {
            return false;
        }
        Vec3 axis{1.0f, 0.0f, 0.0f};
        if (!GetVec3(o, "axis", axis, err, axis)) {
            return false;
        }
        axis = Normalize(axis);
        bool enableLimits = false;
        if (!GetBool(o, "enable_limits", enableLimits, false, err)) {
            return false;
        }
        double lowerT = 0.0;
        (void)GetNumber(o, "lower_translation", lowerT, err, false);
        double upperT = 0.0;
        (void)GetNumber(o, "upper_translation", upperT, err, false);
        bool enableMotor = false;
        if (!GetBool(o, "enable_motor", enableMotor, false, err)) {
            return false;
        }
        double motorSpeed = 0.0;
        (void)GetNumber(o, "motor_speed", motorSpeed, err, false);
        double maxMotorForce = 0.0;
        (void)GetNumber(o, "max_motor_force", maxMotorForce, err, false);
        world.CreatePrismaticJoint(
            bodyA,
            bodyB,
            anchor,
            axis,
            enableLimits,
            static_cast<float>(lowerT),
            static_cast<float>(upperT),
            enableMotor,
            static_cast<float>(motorSpeed),
            static_cast<float>(maxMotorForce));
        return true;
    }

    if (typeStr == "servo") {
        Vec3 anchor{};
        if (!RequireVec3(o, "anchor", anchor, err)) {
            return false;
        }
        Vec3 axis{0.0f, 1.0f, 0.0f};
        if (!GetVec3(o, "axis", axis, err, axis)) {
            return false;
        }
        axis = Normalize(axis);
        double targetAngle = 0.0;
        (void)GetNumber(o, "target_angle", targetAngle, err, false);
        double maxTorque = 1.0;
        (void)GetNumber(o, "max_servo_torque", maxTorque, err, false);
        double posGain = 40.0;
        (void)GetNumber(o, "position_gain", posGain, err, false);
        double dampGain = 1.0;
        (void)GetNumber(o, "damping_gain", dampGain, err, false);
        double intGain = 0.0;
        (void)GetNumber(o, "integral_gain", intGain, err, false);
        double intClamp = 0.5;
        (void)GetNumber(o, "integral_clamp", intClamp, err, false);
        double posSmooth = 0.0;
        (void)GetNumber(o, "position_error_smoothing", posSmooth, err, false);
        double maxCorrAngle = 0.5;
        (void)GetNumber(o, "max_correction_angle", maxCorrAngle, err, false);
        double angleStab = 1.0;
        (void)GetNumber(o, "angle_stabilization", angleStab, err, false);
        world.CreateServoJoint(
            bodyA,
            bodyB,
            anchor,
            axis,
            static_cast<float>(targetAngle),
            static_cast<float>(maxTorque),
            static_cast<float>(posGain),
            static_cast<float>(dampGain),
            static_cast<float>(intGain),
            static_cast<float>(intClamp),
            static_cast<float>(posSmooth),
            static_cast<float>(angleStab),
            static_cast<float>(maxCorrAngle));
        return true;
    }

    err = "unknown joint type: " + typeStr;
    return false;
}

bool ParseShapeType(const std::string& name, ShapeType& out, std::string& err) {
    if (name == "sphere") {
        out = ShapeType::Sphere;
        return true;
    }
    if (name == "box") {
        out = ShapeType::Box;
        return true;
    }
    if (name == "plane") {
        out = ShapeType::Plane;
        return true;
    }
    if (name == "capsule") {
        out = ShapeType::Capsule;
        return true;
    }
    if (name == "cylinder") {
        out = ShapeType::Cylinder;
        return true;
    }
    if (name == "half_cylinder" || name == "halfcylinder") {
        out = ShapeType::HalfCylinder;
        return true;
    }
    if (name == "compound") {
        out = ShapeType::Compound;
        return true;
    }
    err = "unknown shape: " + name;
    return false;
}

bool ParseCompoundChildren(const JsonValue& arrVal, std::vector<CompoundChild>& out, std::string& err) {
    if (arrVal.type != JsonValue::Array) {
        err = "compound children must be a JSON array";
        return false;
    }
    out.clear();
    for (const JsonValue& ch : arrVal.array) {
        if (ch.type != JsonValue::Object) {
            err = "compound child must be object";
            return false;
        }
        std::string shapeName;
        if (!GetString(ch.object, "shape", shapeName, err, true)) {
            return false;
        }
        CompoundChild child{};
        if (!ParseShapeType(shapeName, child.shape, err)) {
            return false;
        }
        if (!IsCompoundChildShapeSupported(child.shape)) {
            err = "unsupported compound child shape: " + shapeName;
            return false;
        }
        if (!GetVec3(ch.object, "local_position", child.localPosition, err, {})) {
            return false;
        }
        Quat lr;
        if (!GetQuat(ch.object, "local_rotation", lr, err)) {
            return false;
        }
        child.localOrientation = lr;
        double r = 0.5;
        (void)GetNumber(ch.object, "radius", r, err, false);
        child.radius = static_cast<float>(r);
        double hh = 0.5;
        (void)GetNumber(ch.object, "half_height", hh, err, false);
        child.halfHeight = static_cast<float>(hh);
        if (!GetVec3(ch.object, "half_extents", child.halfExtents, err, {0.5f, 0.5f, 0.5f})) {
            return false;
        }
        out.push_back(child);
    }
    return true;
}

bool ParseTerrainPatchObject(const JsonObject& o, TerrainPatchConfig& config, TerrainPatchSeed& seed, std::string& err) {
    seed.has_config = true;
    double rows = static_cast<double>(config.rows);
    double cols = static_cast<double>(config.cols);
    double cell_size = static_cast<double>(config.cell_size_m);
    double base_margin = static_cast<double>(config.base_margin_m);
    double min_thickness = static_cast<double>(config.min_cell_thickness_m);
    double influence_sigma = static_cast<double>(config.influence_sigma_m);
    double plane_conf = static_cast<double>(config.plane_confidence);
    double half_life = static_cast<double>(config.confidence_half_life_s);
    double base_blend = static_cast<double>(config.base_update_blend);
    double decay_boost = static_cast<double>(config.decay_update_boost);
    double use_sample_binning = config.use_sample_binning ? 1.0 : 0.0;
    double sample_bin_size = static_cast<double>(config.sample_bin_size_m);
    double use_conservative_collision = config.use_conservative_collision ? 1.0 : 0.0;
    double scroll_world_fixed = config.scroll_world_fixed ? 1.0 : 0.0;
    double lidar_fusion_enable = config.lidar_fusion_enable ? 1.0 : 0.0;
    double lidar_stride = static_cast<double>(config.lidar_sample_stride);
    double lidar_weight = static_cast<double>(config.lidar_sample_weight);

    (void)GetNumber(o, "rows", rows, err, false);
    (void)GetNumber(o, "cols", cols, err, false);
    (void)GetNumber(o, "cell_size_m", cell_size, err, false);
    (void)GetNumber(o, "base_margin_m", base_margin, err, false);
    (void)GetNumber(o, "min_cell_thickness_m", min_thickness, err, false);
    (void)GetNumber(o, "influence_sigma_m", influence_sigma, err, false);
    (void)GetNumber(o, "plane_confidence", plane_conf, err, false);
    (void)GetNumber(o, "confidence_half_life_s", half_life, err, false);
    (void)GetNumber(o, "base_update_blend", base_blend, err, false);
    (void)GetNumber(o, "decay_update_boost", decay_boost, err, false);
    (void)GetNumber(o, "use_sample_binning", use_sample_binning, err, false);
    (void)GetNumber(o, "sample_bin_size_m", sample_bin_size, err, false);
    (void)GetNumber(o, "use_conservative_collision", use_conservative_collision, err, false);
    (void)GetNumber(o, "scroll_world_fixed", scroll_world_fixed, err, false);
    (void)GetNumber(o, "lidar_fusion_enable", lidar_fusion_enable, err, false);
    (void)GetNumber(o, "lidar_sample_stride", lidar_stride, err, false);
    (void)GetNumber(o, "lidar_sample_weight", lidar_weight, err, false);

    if (!GetVec3(o, "center", seed.center, err, seed.center)) {
        return false;
    }
    seed.has_center = FindMember(o, "center") != nullptr;
    if (!GetVec3(o, "plane_normal", seed.plane_normal, err, seed.plane_normal)) {
        return false;
    }
    seed.has_plane_normal = FindMember(o, "plane_normal") != nullptr;

    double plane_height = static_cast<double>(seed.plane_height_m);
    const JsonValue* plane_height_val = FindMember(o, "plane_height_m");
    if (plane_height_val != nullptr) {
        if (plane_height_val->type != JsonValue::Number) {
            err = "field not a number: plane_height_m";
            return false;
        }
        plane_height = plane_height_val->number;
        seed.has_plane_height = true;
    } else {
        const JsonValue* plane_offset_val = FindMember(o, "plane_offset");
        if (plane_offset_val != nullptr) {
            if (plane_offset_val->type != JsonValue::Number) {
                err = "field not a number: plane_offset";
                return false;
            }
            plane_height = plane_offset_val->number;
            seed.has_plane_height = true;
        }
    }
    seed.plane_height_m = static_cast<float>(plane_height);

    auto validate_positive = [&](double value, const char* name) -> bool {
        if (!(value > 0.0) || !std::isfinite(value)) {
            err = std::string(name) + " must be positive";
            return false;
        }
        return true;
    };
    auto validate_range = [&](double value, double lo, double hi, const char* name) -> bool {
        if (!std::isfinite(value) || value < lo || value > hi) {
            err = std::string(name) + " out of range";
            return false;
        }
        return true;
    };

    if (!validate_range(rows, 3.0, 128.0, "terrain_patch.rows") ||
        !validate_range(cols, 3.0, 128.0, "terrain_patch.cols") ||
        !validate_positive(cell_size, "terrain_patch.cell_size_m") ||
        !validate_positive(base_margin, "terrain_patch.base_margin_m") ||
        !validate_positive(min_thickness, "terrain_patch.min_cell_thickness_m") ||
        !validate_positive(influence_sigma, "terrain_patch.influence_sigma_m") ||
        !validate_range(plane_conf, 0.0, 1.0, "terrain_patch.plane_confidence") ||
        !validate_positive(half_life, "terrain_patch.confidence_half_life_s") ||
        !validate_range(base_blend, 0.0, 1.0, "terrain_patch.base_update_blend") ||
        !validate_range(decay_boost, 0.0, 1.0, "terrain_patch.decay_update_boost") ||
        !validate_range(sample_bin_size, 0.01, 10.0, "terrain_patch.sample_bin_size_m") ||
        !validate_range(lidar_stride, 1.0, 32.0, "terrain_patch.lidar_sample_stride") ||
        !validate_range(lidar_weight, 0.0, 1.0, "terrain_patch.lidar_sample_weight") ||
        !validate_range(use_sample_binning, 0.0, 1.0, "terrain_patch.use_sample_binning") ||
        !validate_range(use_conservative_collision, 0.0, 1.0, "terrain_patch.use_conservative_collision") ||
        !validate_range(scroll_world_fixed, 0.0, 1.0, "terrain_patch.scroll_world_fixed") ||
        !validate_range(lidar_fusion_enable, 0.0, 1.0, "terrain_patch.lidar_fusion_enable")) {
        return false;
    }

    config.rows = static_cast<int>(rows);
    config.cols = static_cast<int>(cols);
    config.cell_size_m = static_cast<float>(cell_size);
    config.base_margin_m = static_cast<float>(base_margin);
    config.min_cell_thickness_m = static_cast<float>(min_thickness);
    config.influence_sigma_m = static_cast<float>(influence_sigma);
    config.plane_confidence = static_cast<float>(plane_conf);
    config.confidence_half_life_s = static_cast<float>(half_life);
    config.base_update_blend = static_cast<float>(base_blend);
    config.decay_update_boost = static_cast<float>(decay_boost);
    config.use_sample_binning = use_sample_binning != 0.0;
    config.sample_bin_size_m = static_cast<float>(sample_bin_size);
    config.use_conservative_collision = use_conservative_collision != 0.0;
    config.scroll_world_fixed = scroll_world_fixed != 0.0;
    config.lidar_fusion_enable = lidar_fusion_enable != 0.0;
    config.lidar_sample_stride =
        std::clamp(static_cast<int>(std::lround(lidar_stride)), 1, 32);
    config.lidar_sample_weight = static_cast<float>(lidar_weight);
    if (seed.has_plane_normal) {
        seed.plane_normal = Normalize(seed.plane_normal);
    }
    return true;
}

bool ParseBodyObject(const JsonObject& o, Body& body, std::string& err) {
    std::string shapeName;
    if (!GetString(o, "shape", shapeName, err, true)) {
        return false;
    }
    if (!ParseShapeType(shapeName, body.shape, err)) {
        return false;
    }

    bool isStatic = false;
    const JsonValue* st = FindMember(o, "static");
    if (st && st->type == JsonValue::Bool) {
        isStatic = st->b;
    }
    body.isStatic = isStatic;

    double mass = 1.0;
    (void)GetNumber(o, "mass", mass, err, false);
    body.mass = static_cast<float>(mass);

    if (!GetVec3(o, "position", body.position, err, {})) {
        return false;
    }
    if (!GetVec3(o, "velocity", body.velocity, err, {})) {
        return false;
    }
    if (!GetVec3(o, "angular_velocity", body.angularVelocity, err, {})) {
        return false;
    }
    if (!GetQuat(o, "orientation", body.orientation, err)) {
        return false;
    }

    double rs = 0.6;
    (void)GetNumber(o, "static_friction", rs, err, false);
    body.staticFriction = static_cast<float>(rs);
    double rd = 0.4;
    (void)GetNumber(o, "dynamic_friction", rd, err, false);
    body.dynamicFriction = static_cast<float>(rd);
    double rest = 0.2;
    (void)GetNumber(o, "restitution", rest, err, false);
    body.restitution = static_cast<float>(rest);
    double ld = 0.0;
    (void)GetNumber(o, "linear_damping", ld, err, false);
    body.linearDamping = static_cast<float>(ld);
    double ad = 0.0;
    (void)GetNumber(o, "angular_damping", ad, err, false);
    body.angularDamping = static_cast<float>(ad);
    double cg = static_cast<double>(body.collisionGroup);
    (void)GetNumber(o, "collision_group", cg, err, false);
    body.collisionGroup = static_cast<std::uint32_t>(cg);
    double cm = static_cast<double>(body.collisionMask);
    (void)GetNumber(o, "collision_mask", cm, err, false);
    body.collisionMask = static_cast<std::uint32_t>(cm);

    if (body.shape == ShapeType::Sphere) {
        double rad = 0.5;
        (void)GetNumber(o, "radius", rad, err, false);
        body.radius = static_cast<float>(rad);
    } else if (body.shape == ShapeType::Box) {
        if (!GetVec3(o, "half_extents", body.halfExtents, err, {0.5f, 0.5f, 0.5f})) {
            return false;
        }
    } else if (body.shape == ShapeType::Plane) {
        if (!GetVec3(o, "plane_normal", body.planeNormal, err, {0.0f, 1.0f, 0.0f})) {
            return false;
        }
        body.planeNormal = Normalize(body.planeNormal);
        double po = 0.0;
        (void)GetNumber(o, "plane_offset", po, err, false);
        body.planeOffset = static_cast<float>(po);
    } else if (
        body.shape == ShapeType::Capsule || body.shape == ShapeType::Cylinder
        || body.shape == ShapeType::HalfCylinder) {
        double rad = 0.25;
        (void)GetNumber(o, "radius", rad, err, false);
        body.radius = static_cast<float>(rad);
        double hh = 0.5;
        (void)GetNumber(o, "half_height", hh, err, false);
        body.halfHeight = static_cast<float>(hh);
    } else if (body.shape == ShapeType::Compound) {
        const JsonValue* ch = FindMember(o, "children");
        if (!ch) {
            err = "compound body requires \"children\" array";
            return false;
        }
        if (!ParseCompoundChildren(*ch, body.compoundChildren, err)) {
            return false;
        }
        if (!GetVec3(o, "half_extents", body.halfExtents, err, {0.5f, 0.5f, 0.5f})) {
            return false;
        }
    }

    return true;
}

std::filesystem::path BuildDebugLogPath() {
    std::filesystem::path log_dir = "logs";
    const std::filesystem::path cwd_name = std::filesystem::current_path().filename();
    if (cwd_name != "hexapod-physics-sim") {
        log_dir = std::filesystem::path("hexapod-physics-sim") / log_dir;
    }
    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return "hexapod-physics-sim.log";
    }
    return log_dir / "latest.log";
}

class RunLog {
public:
    explicit RunLog(std::filesystem::path path) : path_(std::move(path)) {
        file_ = std::fopen(path_.string().c_str(), "w");
    }

    ~RunLog() {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    RunLog(const RunLog&) = delete;
    RunLog& operator=(const RunLog&) = delete;

    std::FILE* stream() const { return file_; }
    bool available() const { return file_ != nullptr; }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    std::FILE* file_ = nullptr;
};

const char* SinkLabel(SinkKind k) {
    switch (k) {
        case SinkKind::Dummy:
            return "dummy";
        case SinkKind::Udp:
            return "udp";
    }
    return "?";
}

/// Resolves bundled example paths when cwd is repo root, `build/`, or `hexapod-physics-sim/`.
std::filesystem::path ResolveSceneFilePath(const std::string& pathIn) {
    namespace fs = std::filesystem;
    std::string path = pathIn;
    constexpr std::string_view kLegacyExamples = "demo/scenes/examples/";
    if (path.size() >= kLegacyExamples.size()
        && std::string_view(path).substr(0, kLegacyExamples.size()) == kLegacyExamples) {
        path = "assets/scenes/examples/" + path.substr(kLegacyExamples.size());
    }

    const fs::path p(path);
    std::error_code ec;
    if (fs::is_regular_file(p, ec)) {
        return fs::weakly_canonical(p, ec);
    }
    const fs::path candidates[] = {
        fs::path("hexapod-physics-sim") / path,
        fs::path("..") / path,
        fs::path("..") / "hexapod-physics-sim" / path,
    };
    for (const fs::path& c : candidates) {
        ec.clear();
        if (fs::is_regular_file(c, ec)) {
            return fs::weakly_canonical(c, ec);
        }
    }
    return p;
}

} // namespace

namespace {

float WrapAngleRad(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

void LogJsonSceneFrameDiagnostics(std::FILE* out, int frame, const World& world) {
    if (out == nullptr) {
        return;
    }
    const std::vector<Manifold>& manifolds = world.DebugManifolds();
    std::size_t contact_count = 0;
    float max_pen = 0.0f;
    for (const Manifold& m : manifolds) {
        for (const Contact& c : m.contacts) {
            ++contact_count;
            max_pen = std::max(max_pen, c.penetration);
        }
    }
    float max_dyn_w = 0.0f;
    float max_dyn_v = 0.0f;
    for (std::uint32_t i = 0; i < world.GetBodyCount(); ++i) {
        const Body& b = world.GetBody(i);
        if (b.invMass <= 0.0f || b.isSleeping) {
            continue;
        }
        max_dyn_w = std::max(max_dyn_w, Length(b.angularVelocity));
        max_dyn_v = std::max(max_dyn_v, Length(b.velocity));
    }
    std::fprintf(out,
                 "frame=%d manifolds=%zu contacts=%zu max_pen=%.6f max_dyn_|w|=%.6f max_dyn_|v|=%.6f",
                 frame,
                 manifolds.size(),
                 contact_count,
                 max_pen,
                 max_dyn_w,
                 max_dyn_v);
    const std::uint32_t servo_count = world.GetServoJointCount();
    for (std::uint32_t sj = 0; sj < servo_count; ++sj) {
        const ServoJoint& joint = world.GetServoJoint(sj);
        const float angle = world.GetServoJointAngle(sj);
        const float err = WrapAngleRad(angle - joint.targetAngle);
        std::fprintf(out, " servo%u_err=%.6f servo%u_imp=%.6f", sj, err, sj, joint.servoImpulseSum);
    }
    std::fprintf(out, "\n");
}

} // namespace

bool LoadWorldFromMinphysSceneJson(
    const std::string& json_text,
    World& world_out,
    int& solver_iterations_out,
    std::string& error_out,
    int* joints_loaded_out,
    TerrainPatchConfig* terrain_patch_config_out,
    TerrainPatchSeed* terrain_patch_seed_out) {
    if (joints_loaded_out != nullptr) {
        *joints_loaded_out = 0;
    }
    if (world_out.GetBodyCount() != 0) {
        error_out = "world must be empty before loading a JSON scene";
        return false;
    }
    solver_iterations_out = 16;

    return AppendWorldFromMinphysSceneJson(
        json_text,
        world_out,
        solver_iterations_out,
        error_out,
        joints_loaded_out,
        terrain_patch_config_out,
        terrain_patch_seed_out);
}

bool AppendWorldFromMinphysSceneJson(
    const std::string& json_text,
    World& world_out,
    int& solver_iterations_in_out,
    std::string& error_out,
    int* joints_loaded_out,
    TerrainPatchConfig* terrain_patch_config_out,
    TerrainPatchSeed* terrain_patch_seed_out) {
    if (joints_loaded_out != nullptr) {
        *joints_loaded_out = 0;
    }

    const std::uint32_t body_index_offset = world_out.GetBodyCount();

    const std::string stripped = StripLineComments(json_text);
    JsonParser parser(stripped);
    JsonValue root;
    if (!parser.parse(root, error_out)) {
        return false;
    }
    if (root.type != JsonValue::Object) {
        error_out = "root must be a JSON object";
        return false;
    }

    double schema = 0.0;
    if (GetNumber(root.object, "schema_version", schema, error_out, false) && schema > 2.01) {
        error_out = "unsupported schema_version (supported: 1, 2)";
        return false;
    }

    double si = static_cast<double>(solver_iterations_in_out > 0 ? solver_iterations_in_out : 16);
    if (GetNumber(root.object, "solver_iterations", si, error_out, false)) {
        if (si < 1.0 || si > 128.0) {
            error_out = "solver_iterations out of range [1,128]";
            return false;
        }
        solver_iterations_in_out = static_cast<int>(si);
    }

    if (terrain_patch_config_out != nullptr || terrain_patch_seed_out != nullptr) {
        const JsonValue* terrain_patch_val = FindMember(root.object, "terrain_patch");
        if (terrain_patch_val != nullptr) {
            if (terrain_patch_val->type != JsonValue::Object) {
                error_out = "\"terrain_patch\" must be a JSON object";
                return false;
            }
            TerrainPatchConfig config = terrain_patch_config_out != nullptr ? *terrain_patch_config_out : TerrainPatchConfig{};
            TerrainPatchSeed seed{};
            if (!ParseTerrainPatchObject(terrain_patch_val->object, config, seed, error_out)) {
                return false;
            }
            if (terrain_patch_config_out != nullptr) {
                *terrain_patch_config_out = config;
            }
            if (terrain_patch_seed_out != nullptr) {
                *terrain_patch_seed_out = seed;
            }
        }
    }

    const JsonValue* bodiesVal = FindMember(root.object, "bodies");
    if (!bodiesVal || bodiesVal->type != JsonValue::Array) {
        error_out = "missing \"bodies\" array";
        return false;
    }

    for (const JsonValue& b : bodiesVal->array) {
        if (b.type != JsonValue::Object) {
            error_out = "each body must be a JSON object";
            return false;
        }
        Body body{};
        if (!ParseBodyObject(b.object, body, error_out)) {
            return false;
        }
        world_out.CreateBody(body);
    }

    const JsonValue* jointsVal = FindMember(root.object, "joints");
    int jointsLoaded = 0;
    if (jointsVal != nullptr) {
        if (jointsVal->type != JsonValue::Array) {
            error_out = "\"joints\" must be a JSON array";
            return false;
        }
        for (const JsonValue& j : jointsVal->array) {
            if (j.type != JsonValue::Object) {
                error_out = "each joint must be a JSON object";
                return false;
            }
            if (!ParseJointObject(j.object, world_out, error_out, body_index_offset)) {
                return false;
            }
            ++jointsLoaded;
        }
    }
    if (joints_loaded_out != nullptr) {
        *joints_loaded_out = jointsLoaded;
    }

    return true;
}

bool AppendWorldFromMinphysSceneJsonFile(
    const std::string& scene_path,
    World& world_out,
    int& solver_iterations_in_out,
    std::string& error_out,
    int* joints_loaded_out,
    TerrainPatchConfig* terrain_patch_config_out,
    TerrainPatchSeed* terrain_patch_seed_out) {
    const std::filesystem::path resolved = ResolveSceneFilePath(scene_path);
    std::ifstream in(resolved, std::ios::in | std::ios::binary);
    if (!in) {
        error_out = "cannot open file: " + scene_path;
        if (resolved != std::filesystem::path(scene_path)) {
            error_out += " (resolved: " + resolved.string() + ")";
        }
        return false;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();
    return AppendWorldFromMinphysSceneJson(
        buffer.str(),
        world_out,
        solver_iterations_in_out,
        error_out,
        joints_loaded_out,
        terrain_patch_config_out,
        terrain_patch_seed_out);
}

int RunPhysicsDemoFromJsonFile(
    const std::string& scene_path,
    SinkKind sink_kind,
    int frame_count,
    bool realtime_playback,
    Vec3 gravity,
    const std::string& udp_host,
    int udp_port,
    DemoRunControl* run_control) {
    if (udp_port < 1 || udp_port > 65535) {
        std::cerr << "[scene_json] invalid udp port\n";
        return 1;
    }

    const std::filesystem::path resolved = ResolveSceneFilePath(scene_path);
    std::ifstream in(resolved, std::ios::in | std::ios::binary);
    if (!in) {
        std::cerr << "[scene_json] cannot open file: " << scene_path;
        if (resolved != std::filesystem::path(scene_path)) {
            std::cerr << " (resolved: " << resolved.string() << ")";
        }
        std::cerr << "\n";
        return 1;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    const std::string json = buffer.str();

    World world(gravity);
    int solver_iterations = 16;
    int joints_loaded = 0;
    std::string err;
    if (!LoadWorldFromMinphysSceneJson(json, world, solver_iterations, err, &joints_loaded)) {
        std::cerr << "[scene_json] parse/load failed: " << err << "\n";
        return 1;
    }

    RunLog run_log(BuildDebugLogPath());
#ifndef NDEBUG
    world.SetDebugLogStream(run_log.stream());
    world.SetBlockSolveDebugLogging(false);
#endif

    const std::uint32_t body_count = world.GetBodyCount();
    std::cout << "[hexapod-physics-sim] starting JSON scene file=" << resolved.string()
              << " bodies=" << body_count << " joints=" << joints_loaded << " sink=" << SinkLabel(sink_kind)
              << " udp=" << udp_host << ":" << udp_port << " realtime_playback=" << (realtime_playback ? "true" : "false")
              << " gravity=(" << gravity.x << "," << gravity.y << "," << gravity.z << ")"
              << " solver_iterations=" << solver_iterations;
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    } else {
        std::cout << " log=stderr (failed to open log file)";
    }
    std::cout << "\n";

    std::unique_ptr<FrameSink> sink = MakeFrameSink(sink_kind, udp_host, udp_port);

    constexpr float dt = 1.0f / 60.0f;
    const DemoSteadyClock::time_point t0 = DemoSteadyClock::now();
    for (int frame = 0; frame < frame_count; ++frame) {
        world.Step(dt, solver_iterations);

        sink->begin_frame(frame, static_cast<float>(frame + 1) * dt);
        for (std::uint32_t id = 0; id < body_count; ++id) {
            sink->emit_body(id, world.GetBody(id));
        }
        sink->end_frame();

        CooperativeYield(run_control);
        if (CooperativeCancelled(run_control)) {
            std::cout << "[hexapod-physics-sim] JSON scene cancelled mid-run\n";
            return 0;
        }

        if (run_log.available()) {
            LogJsonSceneFrameDiagnostics(run_log.stream(), frame, world);
        }
        if (const char* diag = std::getenv("MINPHYS_JSON_SCENE_DIAG")) {
            if (diag[0] != '\0' && diag[0] != '0') {
                LogJsonSceneFrameDiagnostics(stderr, frame, world);
            }
        }

        PaceRealtimeOuterFrame(PaceRealtimeEnabled(realtime_playback, run_control), t0, frame, dt);
    }

    std::cout << "[hexapod-physics-sim] completed JSON scene frames=" << frame_count << " sink=" << SinkLabel(sink_kind);
    if (run_log.available()) {
        std::cout << " log=" << run_log.path().string();
    }
    std::cout << "\n";

    return 0;
}

} // namespace minphys3d::demo
