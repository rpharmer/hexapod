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

constexpr float kMinRangeM = static_cast<float>(matrix_lidar_geom::kMinRangeMatrix64x8Mm) / 1000.0f;
constexpr float kMaxRangeM = static_cast<float>(matrix_lidar_geom::kMaxRangeMatrix64x8Mm) / 1000.0f;
constexpr int kMmResolution = 14;

bool IsRobotBody(std::uint32_t body_id, const HexapodSceneObjects& scene) {
    // `scene.body_ids` includes the static ground plane; that must remain a ray target.
    if (body_id == scene.plane) {
        return false;
    }
    for (const std::uint32_t id : scene.body_ids) {
        if (id == scene.plane) {
            continue;
        }
        if (id == body_id) {
            return true;
        }
    }
    return false;
}

bool IsTerrainBody(std::uint32_t body_id, const TerrainPatch& terrain_patch) {
    const auto& ids = terrain_patch.body_ids();
    return std::find(ids.begin(), ids.end(), body_id) != ids.end();
}

bool RayPlaneHit(const Vec3& o,
                 const Vec3& d,
                 const Vec3& plane_point,
                 const Vec3& plane_normal_unit,
                 float& t_out) {
    const float denom = Dot(d, plane_normal_unit);
    if (std::abs(denom) < 1.0e-8f) {
        return false;
    }
    const float t = Dot(plane_point - o, plane_normal_unit) / denom;
    if (!std::isfinite(t) || t <= 1.0e-4f) {
        return false;
    }
    t_out = t;
    return true;
}

bool RaySphereHit(const Vec3& o, const Vec3& d, const Vec3& center, float radius, float& t_out) {
    if (radius <= 1.0e-6f) {
        return false;
    }
    const Vec3 oc = o - center;
    const float b = Dot(oc, d);
    const float c = Dot(oc, oc) - radius * radius;
    const float disc = b * b - c;
    if (disc < 0.0f) {
        return false;
    }
    const float s = std::sqrt(disc);
    float t = -b - s;
    if (t <= 1.0e-4f) {
        t = -b + s;
    }
    if (!std::isfinite(t) || t <= 1.0e-4f) {
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
               float& t_out) {
    const Quat q_inv = Conjugate(Normalize(orientation));
    const Vec3 o_l = Rotate(q_inv, o - center);
    const Vec3 d_l = Rotate(q_inv, d);

    float t_min = -std::numeric_limits<float>::infinity();
    float t_max = std::numeric_limits<float>::infinity();

    const float ox = o_l.x;
    const float oy = o_l.y;
    const float oz = o_l.z;
    const float dx = d_l.x;
    const float dy = d_l.y;
    const float dz = d_l.z;
    const float hx = half_extents.x;
    const float hy = half_extents.y;
    const float hz = half_extents.z;

    auto slab = [&](float origin, float dir, float he, float& in_min, float& in_max) -> bool {
        if (std::abs(dir) < 1.0e-8f) {
            if (origin < -he || origin > he) {
                return false;
            }
            return true;
        }
        float t1 = (-he - origin) / dir;
        float t2 = (he - origin) / dir;
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

    float t_hit = t_min;
    if (t_hit <= 1.0e-4f) {
        t_hit = t_max;
    }
    if (!std::isfinite(t_hit) || t_hit <= 1.0e-4f) {
        return false;
    }
    t_out = t_hit;
    return true;
}

bool RayAabbHit(const Vec3& o, const Vec3& d, const AABB& box, float& t_out) {
    float t_min = -std::numeric_limits<float>::infinity();
    float t_max = std::numeric_limits<float>::infinity();

    auto axis = [&](float o_a, float d_a, float b_min, float b_max) -> bool {
        if (std::abs(d_a) < 1.0e-8f) {
            if (o_a < b_min || o_a > b_max) {
                return false;
            }
            return true;
        }
        float inv = 1.0f / d_a;
        float t1 = (b_min - o_a) * inv;
        float t2 = (b_max - o_a) * inv;
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

    float t_hit = t_min;
    if (t_hit <= 1.0e-4f) {
        t_hit = t_max;
    }
    if (!std::isfinite(t_hit) || t_hit <= 1.0e-4f) {
        return false;
    }
    t_out = t_hit;
    return true;
}

float CastRay(const World& world,
              const HexapodSceneObjects& scene,
              const TerrainPatch& terrain_patch,
              const Vec3& origin,
              const Vec3& dir_unit) {
    float best = kMaxRangeM + 1.0f;

    float terrain_hit = -1.0f;
    if (terrain_patch.RaycastWorld(origin, dir_unit, terrain_hit)) {
        best = terrain_hit;
    }

    for (std::uint32_t id = 0; id < world.GetBodyCount(); ++id) {
        if (IsRobotBody(id, scene)) {
            continue;
        }
        if (IsTerrainBody(id, terrain_patch)) {
            continue;
        }
        const Body& b = world.GetBody(id);
        float t_hit = std::numeric_limits<float>::infinity();

        if (b.shape == ShapeType::Plane) {
            Vec3 n{};
            if (!TryNormalize(b.planeNormal, n)) {
                continue;
            }
            if (!RayPlaneHit(origin, dir_unit, b.position, n, t_hit)) {
                continue;
            }
        } else if (b.shape == ShapeType::Sphere) {
            if (!RaySphereHit(origin, dir_unit, b.position, b.radius, t_hit)) {
                continue;
            }
        } else if (b.shape == ShapeType::Box) {
            const Vec3 c = BodyWorldShapeOrigin(b);
            if (!RayObbHit(origin, dir_unit, c, b.orientation, b.halfExtents, t_hit)) {
                continue;
            }
        } else {
            const AABB aabb = b.ComputeAABB();
            if (!RayAabbHit(origin, dir_unit, aabb, t_hit)) {
                continue;
            }
        }

        if (t_hit < best) {
            best = t_hit;
        }
    }

    if (best > kMaxRangeM || !std::isfinite(best)) {
        return -1.0f;
    }
    return best;
}

Vec3 ChassisForward(const Quat& q_body) {
    // Body local −Z is toward the front leg row (see `scenes.cpp` mount offsets).
    return Normalize(Rotate(q_body, Vec3{0.0f, 0.0f, -1.0f}));
}

Vec3 ChassisUp(const Quat& q_body) {
    return Normalize(Rotate(q_body, Vec3{0.0f, 1.0f, 0.0f}));
}

std::uint16_t RangeToWireMm(float range_m) {
    if (range_m < kMinRangeM) {
        return physics_sim::kMatrixLidarInvalidMm;
    }
    int mm = static_cast<int>(std::lround(range_m * 1000.0f));
    mm = std::max(static_cast<int>(kMinRangeM * 1000.0f), std::min(mm, static_cast<int>(kMaxRangeM * 1000.0f)));
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
    const float kOpticalAxisPitchDownRad = static_cast<float>(matrix_lidar_geom::kOpticalAxisPitchDownRad);
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
            const float az = static_cast<float>(az_d);
            const float el = static_cast<float>(el_d);

            const float c_el = std::cos(el);
            const Vec3 dir = Normalize(
                optical_axis * (c_el * std::cos(az)) + optical_right * (c_el * std::sin(az))
                + optical_up * std::sin(el));

            const float hit_m = CastRay(world, scene, terrain_patch, sensor_origin, dir);
            const std::size_t idx = static_cast<std::size_t>(row * kCols + col);
            if (hit_m < 0.0f) {
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
    if (rsp.matrix_lidar_valid == 0 || terrain_config.lidar_sample_weight <= 1.0e-6f) {
        return;
    }
    if (rsp.matrix_lidar_model != physics_sim::MatrixLidarModel::Matrix64x8) {
        return;
    }
    const int stride = std::max(1, terrain_config.lidar_sample_stride);

    const Quat q = Normalize(chassis.orientation);
    const Vec3 forward = ChassisForward(q);
    const Vec3 up = ChassisUp(q);
    const float kOpticalAxisPitchDownRad = static_cast<float>(matrix_lidar_geom::kOpticalAxisPitchDownRad);
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
            const float az = static_cast<float>(az_d);
            const float el = static_cast<float>(el_d);

            const float c_el = std::cos(el);
            const Vec3 dir = Normalize(
                optical_axis * (c_el * std::cos(az)) + optical_right * (c_el * std::sin(az))
                + optical_up * std::sin(el));

            float t_terrain = -1.0f;
            const bool terrain_hit = terrain_patch.RaycastWorld(sensor_origin, dir, t_terrain);
            if (!terrain_hit || t_terrain <= 1.0e-4f) {
                continue;
            }
            const float t_full = CastRay(world, scene, terrain_patch, sensor_origin, dir);
            if (t_full < 0.0f || !std::isfinite(t_full)) {
                continue;
            }
            if (std::abs(t_full - t_terrain) > 0.04f) {
                continue;
            }

            const Vec3 hit = sensor_origin + dir * t_full;
            float surface_confidence = 0.0f;
            (void)terrain_patch.SampleHeightAndConfidenceWorld(hit.x, hit.z, &surface_confidence);
            if (!std::isfinite(surface_confidence) ||
                surface_confidence < terrain_config.lidar_min_surface_confidence) {
                continue;
            }
            const float incidence = std::max(0.15f, std::abs(dir.y));
            const float conf =
                terrain_config.lidar_sample_weight * surface_confidence * incidence * std::max(0.2f, std::abs(c_el));
            out_samples.push_back(TerrainSample{hit, hit.y, std::clamp(conf, 0.0f, 1.0f)});
        }
    }
}

} // namespace minphys3d::demo
