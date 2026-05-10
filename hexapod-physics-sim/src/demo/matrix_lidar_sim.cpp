#include "demo/matrix_lidar_sim.hpp"

#include "matrix_lidar_geometry.hpp"
#include "minphys3d/core/body.hpp"
#include "minphys3d/math/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace minphys3d::demo {
namespace {

constexpr Real kMinRangeM = static_cast<float>(matrix_lidar_geom::kMinRangeMatrix64x8Mm) / 1000.0;
constexpr Real kMaxRangeM = static_cast<float>(matrix_lidar_geom::kMaxRangeMatrix64x8Mm) / 1000.0;
constexpr int kMmResolution = 14;

// Build a bitset (1 byte per body) marking the bodies that LiDAR rays must skip.
// Includes robot links (everything in scene.body_ids except the ground plane,
// which is a valid LiDAR target) and the terrain heightfield attachment (the
// patch raycast handles terrain separately). The bitset is built once per scan
// and shared across all 512 rays — replaces the previous O(robot_link_count)
// IsRobotBody linear scan that fired per ray per body.
std::vector<std::uint8_t> BuildLidarSkipBodies(const World& world,
                                               const HexapodSceneObjects& scene) {
    const std::uint32_t body_count = world.GetBodyCount();
    std::vector<std::uint8_t> skip(body_count, 0);
    for (const std::uint32_t bid : scene.body_ids) {
        if (bid == scene.plane) {
            continue;
        }
        if (bid < body_count) {
            skip[bid] = 1;
        }
    }
    for (std::uint32_t i = 0; i < body_count; ++i) {
        if (world.IsTerrainAttachmentBody(i)) {
            skip[i] = 1;
        }
    }
    return skip;
}

bool RayPlaneHit(const Vec3& o,
                 const Vec3& d,
                 const Vec3& plane_point,
                 const Vec3& plane_normal_unit,
                 Real& t_out) {
    const Real denom = Dot(d, plane_normal_unit);
    if (std::abs(denom) < 1.0e-8) {
        return false;
    }
    const Real t = Dot(plane_point - o, plane_normal_unit) / denom;
    if (!std::isfinite(t) || t <= 1.0e-4) {
        return false;
    }
    t_out = t;
    return true;
}

bool RaySphereHit(const Vec3& o, const Vec3& d, const Vec3& center, Real radius, Real& t_out) {
    if (radius <= 1.0e-6) {
        return false;
    }
    const Vec3 oc = o - center;
    const Real b = Dot(oc, d);
    const Real c = Dot(oc, oc) - radius * radius;
    const Real disc = b * b - c;
    if (disc < 0.0) {
        return false;
    }
    const Real s = std::sqrt(disc);
    Real t = -b - s;
    if (t <= 1.0e-4) {
        t = -b + s;
    }
    if (!std::isfinite(t) || t <= 1.0e-4) {
        return false;
    }
    t_out = t;
    return true;
}

bool RayObbHit(const Vec3& o,
               const Vec3& d,
               const Vec3& center,
               const Quat& orientation,
               const Vec3& half_extents,
               Real& t_out) {
    const Quat q_inv = Conjugate(Normalize(orientation));
    const Vec3 o_l = Rotate(q_inv, o - center);
    const Vec3 d_l = Rotate(q_inv, d);

    Real t_min = -std::numeric_limits<float>::infinity();
    Real t_max = std::numeric_limits<float>::infinity();

    const Real ox = o_l.x;
    const Real oy = o_l.y;
    const Real oz = o_l.z;
    const Real dx = d_l.x;
    const Real dy = d_l.y;
    const Real dz = d_l.z;
    const Real hx = half_extents.x;
    const Real hy = half_extents.y;
    const Real hz = half_extents.z;

    auto slab = [&](Real origin, Real dir, Real he, Real& in_min, Real& in_max) -> bool {
        if (std::abs(dir) < 1.0e-8) {
            if (origin < -he || origin > he) {
                return false;
            }
            return true;
        }
        Real t1 = (-he - origin) / dir;
        Real t2 = (he - origin) / dir;
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        in_min = std::max(in_min, t1);
        in_max = std::min(in_max, t2);
        return in_min <= in_max;
    };

    if (!slab(ox, dx, hx, t_min, t_max)) {
        return false;
    }
    if (!slab(oy, dy, hy, t_min, t_max)) {
        return false;
    }
    if (!slab(oz, dz, hz, t_min, t_max)) {
        return false;
    }

    Real t_hit = t_min;
    if (t_hit <= 1.0e-4) {
        t_hit = t_max;
    }
    if (!std::isfinite(t_hit) || t_hit <= 1.0e-4) {
        return false;
    }
    t_out = t_hit;
    return true;
}

bool RayAabbHit(const Vec3& o, const Vec3& d, const AABB& box, Real& t_out) {
    Real t_min = -std::numeric_limits<float>::infinity();
    Real t_max = std::numeric_limits<float>::infinity();

    auto axis = [&](Real o_a, Real d_a, Real b_min, Real b_max) -> bool {
        if (std::abs(d_a) < 1.0e-8) {
            if (o_a < b_min || o_a > b_max) {
                return false;
            }
            return true;
        }
        Real inv = 1.0 / d_a;
        Real t1 = (b_min - o_a) * inv;
        Real t2 = (b_max - o_a) * inv;
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        t_min = std::max(t_min, t1);
        t_max = std::min(t_max, t2);
        return t_min <= t_max;
    };

    if (!axis(o.x, d.x, box.min.x, box.max.x)) {
        return false;
    }
    if (!axis(o.y, d.y, box.min.y, box.max.y)) {
        return false;
    }
    if (!axis(o.z, d.z, box.min.z, box.max.z)) {
        return false;
    }

    Real t_hit = t_min;
    if (t_hit <= 1.0e-4) {
        t_hit = t_max;
    }
    if (!std::isfinite(t_hit) || t_hit <= 1.0e-4) {
        return false;
    }
    t_out = t_hit;
    return true;
}

// `skip_bodies` marks the body IDs that should not contribute to the ray (robot
// links, terrain heightfield attachment). It must cover at least world.GetBodyCount()
// entries; bodies whose ID is out of range are not skipped. Passed in by the
// caller so the bitset is built once per scan and reused across all 512 rays
// instead of doing an O(robot_link_count) linear scan per ray per body.
//
// Unlike the previous implementation we walk the broadphase BVH via
// `World::QueryRayCandidates` and only test bodies whose fat-AABB is intersected
// by the ray — turning what was O(N bodies) per ray into O(log N + hits).
Real CastRay(const World& world,
              const HexapodSceneObjects& scene,
              const TerrainPatch& terrain_patch,
              const std::vector<std::uint8_t>& skip_bodies,
              const Vec3& origin,
              const Vec3& dir_unit) {
    (void)scene;
    Real best = kMaxRangeM + 1.0;

    Real terrain_hit = -1.0;
    if (terrain_patch.RaycastWorld(origin, dir_unit, terrain_hit)) {
        best = terrain_hit;
    }

    world.QueryRayCandidates(origin, dir_unit, kMaxRangeM, [&](std::uint32_t id) {
        if (id < skip_bodies.size() && skip_bodies[id] != 0) {
            return;
        }
        const Body& b = world.GetBody(id);
        Real t_hit = std::numeric_limits<float>::infinity();

        if (b.shape == ShapeType::Plane) {
            Vec3 n{};
            if (!TryNormalize(b.planeNormal, n)) {
                return;
            }
            if (!RayPlaneHit(origin, dir_unit, b.position, n, t_hit)) {
                return;
            }
        } else if (b.shape == ShapeType::Sphere) {
            if (!RaySphereHit(origin, dir_unit, b.position, b.radius, t_hit)) {
                return;
            }
        } else if (b.shape == ShapeType::Box) {
            const Vec3 c = BodyWorldShapeOrigin(b);
            if (!RayObbHit(origin, dir_unit, c, b.orientation, b.halfExtents, t_hit)) {
                return;
            }
        } else {
            const AABB aabb = b.ComputeAABB();
            if (!RayAabbHit(origin, dir_unit, aabb, t_hit)) {
                return;
            }
        }

        if (t_hit < best) {
            best = t_hit;
        }
    });

    if (best > kMaxRangeM || !std::isfinite(best)) {
        return -1.0;
    }
    return best;
}

Vec3 ChassisForward(const Quat& q_body) {
    // Body local −Z is toward the front leg row (see `scenes.cpp` mount offsets).
    return Normalize(Rotate(q_body, Vec3{0.0, 0.0, -1.0}));
}

Vec3 ChassisUp(const Quat& q_body) {
    return Normalize(Rotate(q_body, Vec3{0.0, 1.0, 0.0}));
}

std::uint16_t RangeToWireMm(Real range_m) {
    if (range_m < kMinRangeM) {
        return physics_sim::kMatrixLidarInvalidMm;
    }
    int mm = static_cast<int>(std::lround(range_m * 1000.0));
    mm = std::max(static_cast<int>(kMinRangeM * 1000.0), std::min(mm, static_cast<int>(kMaxRangeM * 1000.0)));
    const int q = static_cast<int>(std::lround(static_cast<float>(mm) / static_cast<float>(kMmResolution)))
        * kMmResolution;
    const int clamped = std::max(1, std::min(q, 65534));
    return static_cast<std::uint16_t>(clamped);
}

} // namespace

void FillSimMatrixLidar64x8(const World& world,
                            const HexapodSceneObjects& scene,
                            const TerrainPatch& terrain_patch,
                            const Body& chassis,
                            physics_sim::StateResponse& rsp) {
    rsp.matrix_lidar_valid = 0;
    rsp.matrix_lidar_model = physics_sim::MatrixLidarModel::None;
    rsp.matrix_lidar_cols = 0;
    rsp.matrix_lidar_rows = 0;
    rsp.matrix_lidar_ranges_mm.fill(physics_sim::kMatrixLidarInvalidMm);

    const Quat q = Normalize(chassis.orientation);
    const Vec3 forward = ChassisForward(q);
    const Vec3 up = ChassisUp(q);
    // Pitch the optical axis slightly down so central beams are not parallel to an infinite ground plane.
    const Real kOpticalAxisPitchDownRad = static_cast<float>(matrix_lidar_geom::kOpticalAxisPitchDownRad);
    const Vec3 optical_axis =
        Normalize(forward * std::cos(kOpticalAxisPitchDownRad) - up * std::sin(kOpticalAxisPitchDownRad));
    const Vec3 optical_right = Normalize(Cross(up, optical_axis));
    const Vec3 optical_up = Normalize(Cross(optical_axis, optical_right));

    // Sensor origin: shared placement with server `matrix_lidar_geom` (minphys body frame).
    const Vec3 sensor_origin = chassis.position +
                               Rotate(q,
                                      Vec3{static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyXM),
                                           static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyYM),
                                           static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyZM)});

    constexpr int kCols = 64;
    constexpr int kRows = 8;

    // Single skip-body bitset for the whole scan; see BuildLidarSkipBodies.
    const std::vector<std::uint8_t> skip_bodies = BuildLidarSkipBodies(world, scene);

    for (int row = 0; row < kRows; ++row) {
        for (int col = 0; col < kCols; ++col) {
            double az_d = 0.0;
            double el_d = 0.0;
            matrix_lidar_geom::cellAzElRad(
                row,
                kRows,
                col,
                kCols,
                matrix_lidar_geom::kMatrix64x8FovHRad,
                matrix_lidar_geom::kMatrix64x8FovVRad,
                az_d,
                el_d);
            const Real az = static_cast<float>(az_d);
            const Real el = static_cast<float>(el_d);

            const Real c_el = std::cos(el);
            const Vec3 dir = Normalize(
                optical_axis * (c_el * std::cos(az)) + optical_right * (c_el * std::sin(az))
                + optical_up * std::sin(el));

            const Real hit_m = CastRay(world, scene, terrain_patch, skip_bodies, sensor_origin, dir);
            const std::size_t idx = static_cast<std::size_t>(row * kCols + col);
            if (hit_m < 0.0) {
                rsp.matrix_lidar_ranges_mm[idx] = physics_sim::kMatrixLidarInvalidMm;
            } else {
                rsp.matrix_lidar_ranges_mm[idx] = RangeToWireMm(hit_m);
            }
        }
    }

    rsp.matrix_lidar_valid = 1;
    rsp.matrix_lidar_model = physics_sim::MatrixLidarModel::Matrix64x8;
    rsp.matrix_lidar_cols = static_cast<std::uint8_t>(kCols);
    rsp.matrix_lidar_rows = static_cast<std::uint8_t>(kRows);
}

void AppendMatrixLidarTerrainSamples(const World& world,
                                     const HexapodSceneObjects& scene,
                                     const TerrainPatch& terrain_patch,
                                     const Body& chassis,
                                     const physics_sim::StateResponse& rsp,
                                     const TerrainPatchConfig& terrain_config,
                                     std::vector<TerrainSample>& out_samples) {
    if (rsp.matrix_lidar_valid == 0 || terrain_config.lidar_sample_weight <= 1.0e-6) {
        return;
    }
    if (rsp.matrix_lidar_model != physics_sim::MatrixLidarModel::Matrix64x8) {
        return;
    }
    const int stride = std::max(1, terrain_config.lidar_sample_stride);

    const Quat q = Normalize(chassis.orientation);
    const Vec3 forward = ChassisForward(q);
    const Vec3 up = ChassisUp(q);
    const Real kOpticalAxisPitchDownRad = static_cast<float>(matrix_lidar_geom::kOpticalAxisPitchDownRad);
    const Vec3 optical_axis =
        Normalize(forward * std::cos(kOpticalAxisPitchDownRad) - up * std::sin(kOpticalAxisPitchDownRad));
    const Vec3 optical_right = Normalize(Cross(up, optical_axis));
    const Vec3 optical_up = Normalize(Cross(optical_axis, optical_right));

    const Vec3 sensor_origin = chassis.position +
                               Rotate(q,
                                      Vec3{static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyXM),
                                           static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyYM),
                                           static_cast<float>(matrix_lidar_geom::kSensorOffsetMinphysBodyZM)});

    constexpr int kCols = 64;
    constexpr int kRows = 8;

    const std::vector<std::uint8_t> skip_bodies = BuildLidarSkipBodies(world, scene);

    for (int row = 0; row < kRows; row += stride) {
        for (int col = 0; col < kCols; col += stride) {
            double az_d = 0.0;
            double el_d = 0.0;
            matrix_lidar_geom::cellAzElRad(
                row,
                kRows,
                col,
                kCols,
                matrix_lidar_geom::kMatrix64x8FovHRad,
                matrix_lidar_geom::kMatrix64x8FovVRad,
                az_d,
                el_d);
            const Real az = static_cast<float>(az_d);
            const Real el = static_cast<float>(el_d);

            const Real c_el = std::cos(el);
            const Vec3 dir = Normalize(
                optical_axis * (c_el * std::cos(az)) + optical_right * (c_el * std::sin(az))
                + optical_up * std::sin(el));

            Real t_terrain = -1.0;
            const bool terrain_hit = terrain_patch.RaycastWorld(sensor_origin, dir, t_terrain);
            if (!terrain_hit || t_terrain <= 1.0e-4) {
                continue;
            }
            const Real t_full = CastRay(world, scene, terrain_patch, skip_bodies, sensor_origin, dir);
            if (t_full < 0.0 || !std::isfinite(t_full)) {
                continue;
            }
            if (std::abs(t_full - t_terrain) > 0.04) {
                continue;
            }

            const Vec3 hit = sensor_origin + dir * t_full;
            Real surface_confidence = 0.0;
            (void)terrain_patch.SampleHeightAndConfidenceWorld(hit.x, hit.z, &surface_confidence);
            if (!std::isfinite(surface_confidence) ||
                surface_confidence < terrain_config.lidar_min_surface_confidence) {
                continue;
            }
            const Real incidence = std::max(0.15, std::abs(dir.y));
            const Real conf =
                terrain_config.lidar_sample_weight * surface_confidence * incidence * std::max(0.2, std::abs(c_el));
            out_samples.push_back(TerrainSample{hit, hit.y, std::clamp(conf, 0.0, 1.0)});
        }
    }
}

} // namespace minphys3d::demo
