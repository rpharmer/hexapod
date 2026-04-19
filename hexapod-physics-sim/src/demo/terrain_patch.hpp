#pragma once

#include "minphys3d/core/world.hpp"
#include "minphys3d/math/vec3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace minphys3d::demo {

struct TerrainSample {
    Vec3 world_position{};
    float height_m{0.0f};
    float confidence{0.0f};
};

struct TerrainPatchConfig {
    int rows{15};
    int cols{15};
    float cell_size_m{0.05f};
    float base_margin_m{0.08f};
    float min_cell_thickness_m{0.012f};
    float influence_sigma_m{0.12f};
    float plane_confidence{0.12f};
    float confidence_half_life_s{1.5f};
    float base_update_blend{0.35f};
    float decay_update_boost{0.45f};
};

class TerrainPatch {
public:
    explicit TerrainPatch(TerrainPatchConfig config = {})
        : config_(config) {}

    const TerrainPatchConfig& config() const { return config_; }
    bool initialized() const { return initialized_; }
    const std::vector<std::uint32_t>& body_ids() const { return body_ids_; }
    int rows() const { return config_.rows; }
    int cols() const { return config_.cols; }
    float cell_size_m() const { return config_.cell_size_m; }
    const Vec3& center_world() const { return center_world_; }
    float base_height_m() const { return base_height_m_; }
    const Vec3& last_normal() const { return last_normal_; }
    float last_plane_height_m() const { return last_plane_height_m_; }
    const std::vector<float>& surface_heights_m() const { return surface_heights_; }
    const std::vector<float>& confidences() const { return confidences_; }

    void initialize(World& world, const Vec3& center, float plane_height_m, const Vec3& normal = {0.0f, 1.0f, 0.0f}) {
        if (config_.rows <= 0 || config_.cols <= 0) {
            return;
        }

        center_world_ = center;
        base_height_m_ = plane_height_m - config_.base_margin_m;
        last_plane_height_m_ = plane_height_m;
        if (Length(normal) > 1.0e-6f) {
            last_normal_ = Normalize(normal);
        } else {
            last_normal_ = {0.0f, 1.0f, 0.0f};
        }

        const std::size_t cell_count = static_cast<std::size_t>(config_.rows * config_.cols);
        body_ids_.assign(cell_count, 0);
        surface_heights_.assign(cell_count, plane_height_m);
        confidences_.assign(cell_count, config_.plane_confidence);

        const float half_span_x = 0.5f * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const float half_span_z = 0.5f * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const std::size_t idx = Index(row, col);
                Body cell{};
                cell.shape = ShapeType::Box;
                cell.position = {
                    center_world_.x + (static_cast<float>(col) * config_.cell_size_m) - half_span_x,
                    plane_height_m + 0.5f * config_.min_cell_thickness_m,
                    center_world_.z + (static_cast<float>(row) * config_.cell_size_m) - half_span_z,
                };
                cell.halfExtents = {
                    0.5f * config_.cell_size_m,
                    0.5f * config_.min_cell_thickness_m,
                    0.5f * config_.cell_size_m,
                };
                cell.mass = 0.0f;
                cell.restitution = 0.0f;
                cell.staticFriction = 0.95f;
                cell.dynamicFriction = 0.75f;
                cell.collisionGroup = 0x0004;
                cell.collisionMask = ~0x0004u;
                body_ids_[idx] = world.CreateBody(cell);
            }
        }
        initialized_ = true;
    }

    void update(World& world,
                const Vec3& center,
                float plane_height_m,
                const Vec3& normal,
                const std::vector<TerrainSample>& samples,
                float age_seconds = 0.0f) {
        if (!initialized_) {
            initialize(world, center, plane_height_m);
        }
        if (!initialized_) {
            return;
        }

        center_world_ = center;
        last_plane_height_m_ = plane_height_m;
        if (Length(normal) > 1.0e-6f) {
            last_normal_ = Normalize(normal);
        }

        const float half_span_x = 0.5f * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const float half_span_z = 0.5f * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        const float age_decay = ComputeAgeDecay(age_seconds);
        const float blend = ComputeBlend(age_decay);

        float min_height = std::numeric_limits<float>::infinity();
        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const float x = center_world_.x + (static_cast<float>(col) * config_.cell_size_m) - half_span_x;
                const float z = center_world_.z + (static_cast<float>(row) * config_.cell_size_m) - half_span_z;

                const float target_height = SampleTargetHeight(center_world_, last_normal_, plane_height_m, x, z, samples);
                const float target_confidence = SampleTargetConfidence(samples, x, z);
                const std::size_t idx = Index(row, col);

                surface_heights_[idx] = surface_heights_[idx] * (1.0f - blend) + target_height * blend;
                confidences_[idx] = confidences_[idx] * (1.0f - blend) + target_confidence * blend;
                min_height = std::min(min_height, surface_heights_[idx]);
            }
        }

        if (!std::isfinite(min_height)) {
            min_height = plane_height_m;
        }
        base_height_m_ = min_height - config_.base_margin_m;

        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const float x = center_world_.x + (static_cast<float>(col) * config_.cell_size_m) - half_span_x;
                const float z = center_world_.z + (static_cast<float>(row) * config_.cell_size_m) - half_span_z;
                const std::size_t idx = Index(row, col);
                Body& cell = world.GetBody(body_ids_[idx]);
                const float top_height = surface_heights_[idx];
                const float thickness = std::max(top_height - base_height_m_, config_.min_cell_thickness_m);
                const float half_thickness = 0.5f * thickness;
                cell.position = {x, base_height_m_ + half_thickness, z};
                cell.halfExtents = {
                    0.5f * config_.cell_size_m,
                    half_thickness,
                    0.5f * config_.cell_size_m,
                };
                cell.orientation = {};
                cell.velocity = {};
                cell.angularVelocity = {};
                cell.force = {};
                cell.torque = {};
                cell.isSleeping = false;
                cell.sleepCounter = 0;
            }
        }
    }

    float SampleHeightWorld(float x, float z) const {
        if (!initialized_ || config_.rows <= 0 || config_.cols <= 0 || surface_heights_.empty()) {
            return last_plane_height_m_;
        }

        const GridCoord coord = ToGridCoord(x, z);
        const int x0 = coord.x0;
        const int x1 = coord.x1;
        const int z0 = coord.z0;
        const int z1 = coord.z1;
        const float tx = coord.tx;
        const float tz = coord.tz;

        const float h00 = surface_heights_[Index(z0, x0)];
        const float h10 = surface_heights_[Index(z0, x1)];
        const float h01 = surface_heights_[Index(z1, x0)];
        const float h11 = surface_heights_[Index(z1, x1)];
        const float hx0 = h00 * (1.0f - tx) + h10 * tx;
        const float hx1 = h01 * (1.0f - tx) + h11 * tx;
        return hx0 * (1.0f - tz) + hx1 * tz;
    }

    Vec3 SampleNormalWorld(float x, float z) const {
        const float delta = std::max(0.5f * config_.cell_size_m, 1.0e-3f);
        const float h_l = SampleHeightWorld(x - delta, z);
        const float h_r = SampleHeightWorld(x + delta, z);
        const float h_d = SampleHeightWorld(x, z - delta);
        const float h_u = SampleHeightWorld(x, z + delta);
        Vec3 n{h_l - h_r, 2.0f * delta, h_d - h_u};
        if (!TryNormalize(n, n)) {
            return {0.0f, 1.0f, 0.0f};
        }
        return n;
    }

    bool RaycastWorld(const Vec3& origin, const Vec3& dir_unit, float& t_out) const {
        if (!initialized_) {
            return false;
        }
        const Vec3 d = Normalize(dir_unit);
        if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z)) {
            return false;
        }

        constexpr float kMaxRaycastMeters = 8.0f;
        const float step = std::max(0.0125f, 0.35f * config_.cell_size_m);
        float t_prev = 0.0f;
        float h_prev = origin.y - SampleHeightWorld(origin.x, origin.z);
        if (h_prev <= 0.0f) {
            return false;
        }
        for (float t = step; t <= kMaxRaycastMeters; t += step) {
            const Vec3 p = origin + d * t;
            const float terrain_h = SampleHeightWorld(p.x, p.z);
            const float h = p.y - terrain_h;
            if (h <= 0.0f) {
                float lo = t_prev;
                float hi = t;
                for (int iter = 0; iter < 12; ++iter) {
                    const float mid = 0.5f * (lo + hi);
                    const Vec3 pm = origin + d * mid;
                    const float hm = pm.y - SampleHeightWorld(pm.x, pm.z);
                    if (hm > 0.0f) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                t_out = hi;
                return true;
            }
            t_prev = t;
            h_prev = h;
        }
        (void)h_prev;
        return false;
    }

private:
    struct GridCoord {
        int x0{};
        int x1{};
        int z0{};
        int z1{};
        float tx{0.0f};
        float tz{0.0f};
    };

    TerrainPatchConfig config_{};
    std::vector<std::uint32_t> body_ids_{};
    std::vector<float> surface_heights_{};
    std::vector<float> confidences_{};
    bool initialized_{false};
    Vec3 center_world_{0.0f, 0.0f, 0.0f};
    float base_height_m_{0.0f};
    Vec3 last_normal_{0.0f, 1.0f, 0.0f};
    float last_plane_height_m_{0.0f};

    std::size_t Index(int row, int col) const {
        return static_cast<std::size_t>(row * config_.cols + col);
    }

    int ClampCol(int col) const {
        return std::clamp(col, 0, std::max(0, config_.cols - 1));
    }

    int ClampRow(int row) const {
        return std::clamp(row, 0, std::max(0, config_.rows - 1));
    }

    GridCoord ToGridCoord(float x, float z) const {
        const float half_span_x = 0.5f * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const float half_span_z = 0.5f * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        const float grid_x = (x - (center_world_.x - half_span_x)) / config_.cell_size_m;
        const float grid_z = (z - (center_world_.z - half_span_z)) / config_.cell_size_m;

        const int x0 = ClampCol(static_cast<int>(std::floor(grid_x)));
        const int z0 = ClampRow(static_cast<int>(std::floor(grid_z)));
        const int x1 = ClampCol(x0 + 1);
        const int z1 = ClampRow(z0 + 1);
        const float tx = std::clamp(grid_x - static_cast<float>(x0), 0.0f, 1.0f);
        const float tz = std::clamp(grid_z - static_cast<float>(z0), 0.0f, 1.0f);
        return GridCoord{x0, x1, z0, z1, tx, tz};
    }

    float SampleTargetConfidence(const std::vector<TerrainSample>& samples, float x, float z) const {
        float weight_sum = config_.plane_confidence;
        for (const TerrainSample& sample : samples) {
            const float dx = x - sample.world_position.x;
            const float dz = z - sample.world_position.z;
            const float d2 = dx * dx + dz * dz;
            const float sigma2 = std::max(1.0e-6f, config_.influence_sigma_m * config_.influence_sigma_m);
            const float influence = std::clamp(
                sample.confidence * std::exp(-0.5f * d2 / sigma2),
                0.0f,
                1.0f);
            weight_sum += influence;
        }
        return std::clamp(weight_sum / (1.0f + weight_sum), 0.0f, 1.0f);
    }

    float SampleTargetHeight(const Vec3& center,
                             const Vec3& normal,
                             float plane_height_m,
                             float x,
                             float z,
                             const std::vector<TerrainSample>& samples) const {
        const float plane_height = PlaneHeightAt(center, normal, plane_height_m, x, z);
        float weighted_height = plane_height * config_.plane_confidence;
        float weight_sum = config_.plane_confidence;

        for (const TerrainSample& sample : samples) {
            const float dx = x - sample.world_position.x;
            const float dz = z - sample.world_position.z;
            const float d2 = dx * dx + dz * dz;
            const float sigma2 = std::max(1.0e-6f, config_.influence_sigma_m * config_.influence_sigma_m);
            const float influence = std::clamp(
                sample.confidence * std::exp(-0.5f * d2 / sigma2),
                0.0f,
                1.0f);
            if (influence <= 1.0e-5f) {
                continue;
            }
            weighted_height += sample.height_m * influence;
            weight_sum += influence;
        }

        if (weight_sum <= 1.0e-6f) {
            return plane_height;
        }
        return weighted_height / weight_sum;
    }

    static float PlaneHeightAt(const Vec3& origin, const Vec3& normal, float plane_height, float x, float z) {
        Vec3 n = normal;
        if (Length(n) <= 1.0e-6f) {
            n = {0.0f, 1.0f, 0.0f};
        } else {
            n = Normalize(n);
        }
        const float ny = std::max(std::abs(n.y), 0.25f);
        return plane_height - (n.x * (x - origin.x) + n.z * (z - origin.z)) / ny;
    }

    float ComputeAgeDecay(float age_seconds) const {
        if (config_.confidence_half_life_s <= 1.0e-6f) {
            return 0.0f;
        }
        if (age_seconds <= 0.0f) {
            return 1.0f;
        }
        const float t = std::max(0.0f, age_seconds);
        return std::pow(0.5f, t / config_.confidence_half_life_s);
    }

    float ComputeBlend(float age_decay) const {
        const float decay_component = 1.0f - std::clamp(age_decay, 0.0f, 1.0f);
        const float blend = config_.base_update_blend + decay_component * config_.decay_update_boost;
        return std::clamp(blend, 0.0f, 1.0f);
    }
};

} // namespace minphys3d::demo
