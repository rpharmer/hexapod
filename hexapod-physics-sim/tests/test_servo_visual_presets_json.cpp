#include "demo/scene_json.hpp"

#include "minphys3d/core/body.hpp"
#include "minphys3d/core/world.hpp"
#include "minphys3d/joints/types.hpp"
#include "minphys3d/math/vec3.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>

namespace {

using minphys3d::Body;
using minphys3d::Length;
using minphys3d::Real;
using minphys3d::ServoJoint;
using minphys3d::Vec3;
using minphys3d::World;

Real WrapAngleRad(Real a) {
    return std::atan2(std::sin(a), std::cos(a));
}

/// Match `ResolveSceneFilePath` in scene_json.cpp so tests find assets from build/ or repo root.
std::filesystem::path ResolveBundledScenePath(const std::string& pathIn) {
    namespace fs = std::filesystem;
    const fs::path p(pathIn);
    std::error_code ec;
    if (fs::is_regular_file(p, ec)) {
        return fs::weakly_canonical(p, ec);
    }
    const fs::path candidates[] = {
        fs::path("hexapod-physics-sim") / pathIn,
        fs::path("..") / pathIn,
        fs::path("..") / "hexapod-physics-sim" / pathIn,
    };
    for (const fs::path& c : candidates) {
        ec.clear();
        if (fs::is_regular_file(c, ec)) {
            return fs::weakly_canonical(c, ec);
        }
    }
    return p;
}

struct ServoSceneCase {
    const char* relative_path;
    const char* label;
    minphys3d::Vec3 gravity;
    int steps;
    Real max_linear_speed;
    Real max_angular_speed;
    Real max_servo_angle_error;
    Real max_body_position_radius;
};

bool BodyStateFinite(const Body& b) {
    const Vec3& p = b.position;
    const Vec3& v = b.velocity;
    const Vec3& w = b.angularVelocity;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(v.x) && std::isfinite(v.y)
        && std::isfinite(v.z) && std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
}

bool RunCase(const ServoSceneCase& c, bool verbose) {
    const std::filesystem::path resolved = ResolveBundledScenePath(c.relative_path);
    std::ifstream in(resolved, std::ios::in | std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "[servo_json] FAIL missing file label=%s path=%s\n", c.label, c.relative_path);
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    World world(c.gravity);
    int solver_iterations = 16;
    std::string err;
    if (!minphys3d::demo::LoadWorldFromMinphysSceneJson(buffer.str(), world, solver_iterations, err, nullptr)) {
        std::fprintf(stderr, "[servo_json] FAIL load label=%s err=%s\n", c.label, err.c_str());
        return false;
    }

    constexpr Real kDt = 1.0 / 60.0;
    Real peak_v = 0.0;
    Real peak_w = 0.0;
    Real peak_err = 0.0;
    Real peak_pos_r = 0.0;

    for (int step = 0; step < c.steps; ++step) {
        world.Step(kDt, solver_iterations);
        for (std::uint32_t i = 0; i < world.GetBodyCount(); ++i) {
            const Body& b = world.GetBody(i);
            if (!BodyStateFinite(b)) {
                std::fprintf(stderr,
                             "[servo_json] FAIL non-finite body label=%s step=%d id=%u\n",
                             c.label,
                             step,
                             static_cast<unsigned>(i));
                return false;
            }
            peak_pos_r = std::max(peak_pos_r, Length(b.position));
            if (b.invMass > 0.0 && !b.isSleeping) {
                peak_v = std::max(peak_v, Length(b.velocity));
                peak_w = std::max(peak_w, Length(b.angularVelocity));
            }
        }
        for (std::uint32_t sj = 0; sj < world.GetServoJointCount(); ++sj) {
            const ServoJoint& j = world.GetServoJoint(sj);
            const Real ang = world.GetServoJointAngle(sj);
            peak_err = std::max(peak_err, std::abs(WrapAngleRad(ang - j.targetAngle)));
        }
        if (verbose && (step % 120 == 0 || step == c.steps - 1)) {
            std::fprintf(stderr,
                         "[servo_json] label=%s step=%d peak_|v|=%.4f peak_|w|=%.4f peak_err=%.4f R=%.2f\n",
                         c.label,
                         step,
                         peak_v,
                         peak_w,
                         peak_err,
                         peak_pos_r);
        }
    }

    bool ok = peak_v <= c.max_linear_speed && peak_w <= c.max_angular_speed && peak_err <= c.max_servo_angle_error
        && peak_pos_r <= c.max_body_position_radius;
    if (!ok) {
        std::fprintf(stderr,
                     "[servo_json] FAIL bounds label=%s peak_|v|=%.4f cap_v=%.4f peak_|w|=%.4f cap_w=%.4f peak_err=%.4f "
                     "cap_err=%.4f R=%.2f cap_R=%.2f\n",
                     c.label,
                     peak_v,
                     c.max_linear_speed,
                     peak_w,
                     c.max_angular_speed,
                     peak_err,
                     c.max_servo_angle_error,
                     peak_pos_r,
                     c.max_body_position_radius);
    }
    return ok;
}

} // namespace

int main() {
    const ServoSceneCase kCases[] = {
        {"assets/scenes/visual/vis_servo_arm.json",
            "vis_servo_arm",
            {0.0, -9.81, 0.0},
            720,
            0.12,
            1.2,
            0.38,
            6.0},
        {"assets/scenes/visual/vis_servo_chain.json",
            "vis_servo_chain",
            {0.0, -9.81, 0.0},
            900,
            18.0,
            14.0,
            1.15,
            8.0},
        {"assets/scenes/visual/vis_servo_chain.json",
            "vis_servo_chain_zero_g",
            {0.0, 0.0, 0.0},
            600,
            6.0,
            5.0,
            1.2,
            8.0},
        {"assets/scenes/visual/vis_servo_soft_track.json",
            "vis_servo_soft_track",
            {0.0, -9.81, 0.0},
            900,
            0.15,
            1.5,
            0.55,
            6.0},
        {"assets/scenes/visual/vis_servo_stiff_hold.json",
            "vis_servo_stiff_hold",
            {0.0, -9.81, 0.0},
            720,
            0.2,
            2.0,
            0.45,
            6.0},
        {"assets/scenes/visual/vis_servo_integral_bar.json",
            "vis_servo_integral_bar",
            {0.0, -9.81, 0.0},
            1440,
            4.0,
            8.0,
            0.65,
            6.0},
        {"assets/scenes/visual/vis_servo_y_swing.json",
            "vis_servo_y_swing",
            {0.0, -9.81, 0.0},
            900,
            3.5,
            6.0,
            0.95,
            6.0},
        {"assets/scenes/visual/vis_servo_torsion_stack.json",
            "vis_servo_torsion_stack",
            {0.0, -9.81, 0.0},
            960,
            12.0,
            10.0,
            0.85,
            6.0},
    };

    const char* v = std::getenv("MINPHYS_SERVO_JSON_TEST_VERBOSE");
    const bool verbose = v != nullptr && v[0] != '\0' && v[0] != '0';

    bool all_ok = true;
    for (const ServoSceneCase& c : kCases) {
        if (!RunCase(c, verbose)) {
            all_ok = false;
        }
    }
    return all_ok ? 0 : 1;
}
