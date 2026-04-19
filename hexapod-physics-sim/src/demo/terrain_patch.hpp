#pragma once

// TerrainPatch: local height field (belief surface) over XZ with optional physics
// collision prisms. LiDAR / RaycastWorld use bilinear SampleHeightWorld; per-cell
// static boxes approximate contact. Optional conservative_collision raises box tops
// toward neighbour maxima so colliders are not below the smooth field near steps.

#include "minphys3d/core/world.hpp"
#include "minphys3d/math/vec3.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
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
    /// When true, SampleTarget* only scans terrain samples in spatial bins near each cell (see sample_bin_size_m).
    bool use_sample_binning{false};
    float sample_bin_size_m{0.25f};
    /// When true, physics box tops use max of neighbours (conservative vs bilinear belief).
    bool use_conservative_collision{false};
    /// World-anchored grid: blit overlapping heights when the patch centre moves in XZ.
    bool scroll_world_fixed{false};
    /// After each sim step, fuse matrix LiDAR ground hits (down-weight vs feet in correction path).
    bool lidar_fusion_enable{false};
    int lidar_sample_stride{4};
    /// Scales per-beam confidence before exp-based weighting (0 disables lidar fusion in practice).
    float lidar_sample_weight{0.18f};
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
    /// Populated when use_conservative_collision; same layout as surface_heights_m.
    const std::vector<float>& collision_heights_m() const { return collision_heights_; }
    bool has_collision_layer() const { return config_.use_conservative_collision && !collision_heights_.empty(); }

    void initialize(World& world, const Vec3& center, float plane_height_m, const Vec3& normal = {0.0f, 1.0f, 0.0f}) {
        if (config_.rows <= 0 || config_.cols <= 0) {
            return;
        }

        center_world_ = center;
        const float half_span_x = 0.5f * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const float half_span_z = 0.5f * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        grid_world_origin_x_ = center_world_.x - half_span_x;
        grid_world_origin_z_ = center_world_.z - half_span_z;
        scroll_state_valid_ = config_.scroll_world_fixed;

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
        collision_heights_.assign(cell_count, plane_height_m);

        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const std::size_t idx = Index(row, col);
                Body cell{};
                cell.shape = ShapeType::Box;
                cell.position = {
                    grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m,
                    plane_height_m + 0.5f * config_.min_cell_thickness_m,
                    grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m,
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
        const float xmin_new = center_world_.x - half_span_x;
        const float zmin_new = center_world_.z - half_span_z;

        if (config_.scroll_world_fixed) {
            if (!scroll_state_valid_) {
                grid_world_origin_x_ = xmin_new;
                grid_world_origin_z_ = zmin_new;
                scroll_state_valid_ = true;
            } else {
                const int d_col = static_cast<int>(std::lround((xmin_new - grid_world_origin_x_) / config_.cell_size_m));
                const int d_row = static_cast<int>(std::lround((zmin_new - grid_world_origin_z_) / config_.cell_size_m));
                if (d_col != 0 || d_row != 0) {
                    ScrollBlitHeights(d_col, d_row, xmin_new, zmin_new, plane_height_m, center, normal, samples);
                }
                grid_world_origin_x_ = xmin_new;
                grid_world_origin_z_ = zmin_new;
            }
        } else {
            grid_world_origin_x_ = xmin_new;
            grid_world_origin_z_ = zmin_new;
        }

        const float age_decay = ComputeAgeDecay(age_seconds);
        const float blend = ComputeBlend(age_decay);

        std::vector<std::vector<std::size_t>> cell_sample_lists;
        std::vector<const std::vector<std::size_t>*> cell_sample_buckets;
        if (config_.use_sample_binning && samples.size() >= 8) {
            FillSampleBinsForCells(samples, cell_sample_lists, cell_sample_buckets);
        }

        float min_height = std::numeric_limits<float>::infinity();
        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const float x = grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m;
                const float z = grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m;

                const float target_height =
                    SampleTargetHeightBinned(center_world_, last_normal_, plane_height_m, x, z, samples, cell_sample_buckets);
                const float target_confidence =
                    SampleTargetConfidenceBinned(samples, x, z, cell_sample_buckets);
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

        if (config_.use_conservative_collision) {
            collision_heights_.resize(surface_heights_.size());
            for (int row = 0; row < config_.rows; ++row) {
                for (int col = 0; col < config_.cols; ++col) {
                    const std::size_t idx = Index(row, col);
                    float mx = surface_heights_[idx];
                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            const int nr = row + dr;
                            const int nc = col + dc;
                            if (nr < 0 || nr >= config_.rows || nc < 0 || nc >= config_.cols) {
                                continue;
                            }
                            mx = std::max(mx, surface_heights_[Index(nr, nc)]);
                        }
                    }
                    collision_heights_[idx] = mx;
                }
            }
        } else {
            collision_heights_.clear();
        }

        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const float x = grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m;
                const float z = grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m;
                const std::size_t idx = Index(row, col);
                Body& cell = world.GetBody(body_ids_[idx]);
                const float top_height = config_.use_conservative_collision ? collision_heights_[idx] : surface_heights_[idx];
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
        float conf_ignored = 0.0f;
        return SampleHeightAndConfidenceWorld(x, z, &conf_ignored);
    }

    /// Bilinear height and confidence at (x,z) for planner / diagnostics.
    float SampleHeightAndConfidenceWorld(float x, float z, float* out_confidence) const {
        if (!initialized_ || config_.rows <= 0 || config_.cols <= 0 || surface_heights_.empty()) {
            if (out_confidence != nullptr) {
                *out_confidence = 0.0f;
            }
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
        const float h = hx0 * (1.0f - tz) + hx1 * tz;

        if (out_confidence != nullptr) {
            const float c00 = confidences_[Index(z0, x0)];
            const float c10 = confidences_[Index(z0, x1)];
            const float c01 = confidences_[Index(z1, x0)];
            const float c11 = confidences_[Index(z1, x1)];
            const float cx0 = c00 * (1.0f - tx) + c10 * tx;
            const float cx1 = c01 * (1.0f - tx) + c11 * tx;
            *out_confidence = cx0 * (1.0f - tz) + cx1 * tz;
        }
        return h;
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
        const float xmin = grid_world_origin_x_;
        const float zmin = grid_world_origin_z_;
        const float xmax = xmin + static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const float zmax = zmin + static_cast<float>(config_.rows - 1) * config_.cell_size_m;

        auto height_residual = [&](float t) {
            const Vec3 p = origin + d * t;
            return p.y - SampleHeightWorld(p.x, p.z);
        };

        const float h0 = height_residual(0.0f);
        if (h0 <= 0.0f) {
            return false;
        }

        const float kEps = 1.0e-5f;
        if (std::abs(d.x) < 1.0e-6f && std::abs(d.z) < 1.0e-6f) {
            if (origin.x < xmin - kEps || origin.x > xmax + kEps || origin.z < zmin - kEps || origin.z > zmax + kEps) {
                return false;
            }
            if (std::abs(d.y) < 1.0e-6f) {
                return false;
            }
            const float h_ground = SampleHeightWorld(origin.x, origin.z);
            const float t_hit = (h_ground - origin.y) / d.y;
            if (!std::isfinite(t_hit) || t_hit <= kEps || t_hit > kMaxRaycastMeters) {
                return false;
            }
            if (height_residual(t_hit) > 1.0e-3f) {
                return false;
            }
            t_out = t_hit;
            return true;
        }

        float t_enter = 0.0f;
        float t_exit = kMaxRaycastMeters;
        if (!RaySlabEnterExitXz(origin, d, xmin, xmax, zmin, zmax, t_enter, t_exit)) {
            return false;
        }
        if (t_exit <= kEps) {
            return false;
        }
        float t = std::max(0.0f, t_enter);
        if (t >= t_exit) {
            return false;
        }

        const int max_steps = (config_.cols + config_.rows) * 8 + 64;
        for (int step = 0; step < max_steps && t < t_exit - kEps; ++step) {
            const Vec3 p = origin + d * t;
            int col = static_cast<int>(std::floor((p.x - xmin) / config_.cell_size_m));
            int row = static_cast<int>(std::floor((p.z - zmin) / config_.cell_size_m));
            col = std::clamp(col, 0, std::max(0, config_.cols - 2));
            row = std::clamp(row, 0, std::max(0, config_.rows - 2));

            const float cell_x0 = xmin + static_cast<float>(col) * config_.cell_size_m;
            const float cell_z0 = zmin + static_cast<float>(row) * config_.cell_size_m;
            float dt_x = std::numeric_limits<float>::infinity();
            float dt_z = std::numeric_limits<float>::infinity();
            if (d.x > kEps) {
                dt_x = (cell_x0 + config_.cell_size_m - p.x) / d.x;
            } else if (d.x < -kEps) {
                dt_x = (cell_x0 - p.x) / d.x;
            }
            if (d.z > kEps) {
                dt_z = (cell_z0 + config_.cell_size_m - p.z) / d.z;
            } else if (d.z < -kEps) {
                dt_z = (cell_z0 - p.z) / d.z;
            }

            float dt = std::min(dt_x, dt_z);
            if (!std::isfinite(dt)) {
                dt = std::max(dt_x, dt_z);
            }
            if (!std::isfinite(dt) || dt <= 0.0f) {
                dt = t_exit - t;
            }
            float t_end = std::min(t_exit, t + dt);
            t_end = std::max(t_end, t + 1.0e-5f);
            const float fa = height_residual(t);
            const float fb = height_residual(t_end);
            if (fa <= 0.0f) {
                t_out = t;
                return true;
            }
            if (fb <= 0.0f) {
                float lo = t;
                float hi = t_end;
                for (int iter = 0; iter < 22; ++iter) {
                    const float mid = 0.5f * (lo + hi);
                    if (height_residual(mid) > 0.0f) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                t_out = hi;
                return true;
            }
            t = t_end + 1.0e-4f;
        }
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
    std::vector<float> collision_heights_{};
    bool initialized_{false};
    Vec3 center_world_{0.0f, 0.0f, 0.0f};
    float base_height_m_{0.0f};
    Vec3 last_normal_{0.0f, 1.0f, 0.0f};
    float last_plane_height_m_{0.0f};
    float grid_world_origin_x_{0.0f};
    float grid_world_origin_z_{0.0f};
    bool scroll_state_valid_{false};

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
        const float grid_x = (x - grid_world_origin_x_) / config_.cell_size_m;
        const float grid_z = (z - grid_world_origin_z_) / config_.cell_size_m;

        const int x0 = ClampCol(static_cast<int>(std::floor(grid_x)));
        const int z0 = ClampRow(static_cast<int>(std::floor(grid_z)));
        const int x1 = ClampCol(x0 + 1);
        const int z1 = ClampRow(z0 + 1);
        const float tx = std::clamp(grid_x - static_cast<float>(x0), 0.0f, 1.0f);
        const float tz = std::clamp(grid_z - static_cast<float>(z0), 0.0f, 1.0f);
        return GridCoord{x0, x1, z0, z1, tx, tz};
    }

    static bool RaySlabEnterExitXz(const Vec3& o,
                                   const Vec3& d,
                                   float xmin,
                                   float xmax,
                                   float zmin,
                                   float zmax,
                                   float& t_enter,
                                   float& t_exit) {
        float t0 = 0.0f;
        float t1 = std::numeric_limits<float>::infinity();
        auto axis = [&](float pos, float dir, float min_v, float max_v) -> bool {
            if (std::abs(dir) < 1.0e-8f) {
                if (pos < min_v || pos > max_v) {
                    return false;
                }
                return true;
            }
            float inv = 1.0f / dir;
            float t_near = (min_v - pos) * inv;
            float t_far = (max_v - pos) * inv;
            if (t_near > t_far) {
                std::swap(t_near, t_far);
            }
            t0 = std::max(t0, t_near);
            t1 = std::min(t1, t_far);
            return t0 <= t1;
        };
        if (!axis(o.x, d.x, xmin, xmax)) {
            return false;
        }
        if (!axis(o.z, d.z, zmin, zmax)) {
            return false;
        }
        t_enter = t0;
        t_exit = t1;
        return std::isfinite(t_exit) && t_exit > 0.0f;
    }

    void ScrollBlitHeights(int d_col,
                           int d_row,
                           float xmin_new,
                           float zmin_new,
                           float plane_height_m,
                           const Vec3& center,
                           const Vec3& normal,
                           const std::vector<TerrainSample>& samples) {
        std::vector<float> new_heights(surface_heights_.size());
        std::vector<float> new_conf(confidences_.size());
        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const int src_col = col + d_col;
                const int src_row = row + d_row;
                const std::size_t dst = Index(row, col);
                if (src_row >= 0 && src_row < config_.rows && src_col >= 0 && src_col < config_.cols) {
                    const std::size_t src = Index(src_row, src_col);
                    new_heights[dst] = surface_heights_[src];
                    new_conf[dst] = confidences_[src];
                } else {
                    const float x = xmin_new + static_cast<float>(col) * config_.cell_size_m;
                    const float z = zmin_new + static_cast<float>(row) * config_.cell_size_m;
                    new_heights[dst] = SampleTargetHeight(center, normal, plane_height_m, x, z, samples);
                    new_conf[dst] = SampleTargetConfidence(samples, x, z);
                }
            }
        }
        surface_heights_.swap(new_heights);
        confidences_.swap(new_conf);
    }

    void FillSampleBinsForCells(const std::vector<TerrainSample>& samples,
                                std::vector<std::vector<std::size_t>>& cell_lists,
                                std::vector<const std::vector<std::size_t>*>& out_ptrs) const {
        const std::size_t cell_count = static_cast<std::size_t>(config_.rows * config_.cols);
        cell_lists.assign(cell_count, {});
        const float bin_w = std::max(1.0e-3f, config_.sample_bin_size_m);
        const float radius = 3.5f * config_.influence_sigma_m;
        const float anchor_x = grid_world_origin_x_;
        const float anchor_z = grid_world_origin_z_;

        std::unordered_map<std::int64_t, std::vector<std::size_t>> bins;
        bins.reserve(samples.size() * 2 + 8);
        for (std::size_t si = 0; si < samples.size(); ++si) {
            const TerrainSample& s = samples[si];
            const int ix = static_cast<int>(std::floor((s.world_position.x - anchor_x) / bin_w));
            const int iz = static_cast<int>(std::floor((s.world_position.z - anchor_z) / bin_w));
            const std::int64_t key =
                (static_cast<std::int64_t>(ix) * 1000003LL) + static_cast<std::int64_t>(iz) * 9176LL;
            bins[key].push_back(si);
        }

        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const float wx = grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m;
                const float wz = grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m;
                const int ix0 = static_cast<int>(std::floor((wx - radius - anchor_x) / bin_w));
                const int ix1 = static_cast<int>(std::floor((wx + radius - anchor_x) / bin_w));
                const int iz0 = static_cast<int>(std::floor((wz - radius - anchor_z) / bin_w));
                const int iz1 = static_cast<int>(std::floor((wz + radius - anchor_z) / bin_w));
                std::vector<std::size_t>& out = cell_lists[Index(row, col)];
                for (int bz = iz0; bz <= iz1; ++bz) {
                    for (int bx = ix0; bx <= ix1; ++bx) {
                        const std::int64_t key =
                            (static_cast<std::int64_t>(bx) * 1000003LL) + static_cast<std::int64_t>(bz) * 9176LL;
                        const auto found = bins.find(key);
                        if (found == bins.end()) {
                            continue;
                        }
                        for (std::size_t idx : found->second) {
                            out.push_back(idx);
                        }
                    }
                }
            }
        }

        out_ptrs.assign(cell_count, nullptr);
        for (std::size_t i = 0; i < cell_count; ++i) {
            out_ptrs[i] = &cell_lists[i];
        }
    }

    float SampleTargetConfidenceBinned(const std::vector<TerrainSample>& samples,
                                       float x,
                                       float z,
                                       const std::vector<const std::vector<std::size_t>*>& buckets) const {
        if (buckets.empty()) {
            return SampleTargetConfidence(samples, x, z);
        }
        const int col = static_cast<int>(
            std::floor((x - grid_world_origin_x_) / std::max(1.0e-6f, config_.cell_size_m)));
        const int row = static_cast<int>(
            std::floor((z - grid_world_origin_z_) / std::max(1.0e-6f, config_.cell_size_m)));
        const std::size_t idx = static_cast<std::size_t>(std::clamp(row, 0, config_.rows - 1) * config_.cols +
                                                         std::clamp(col, 0, config_.cols - 1));
        if (idx < buckets.size() && buckets[idx] != nullptr) {
            return SampleTargetConfidenceIndices(samples, x, z, *buckets[idx]);
        }
        return SampleTargetConfidence(samples, x, z);
    }

    float SampleTargetHeightBinned(const Vec3& center,
                                   const Vec3& normal,
                                   float plane_height_m,
                                   float x,
                                   float z,
                                   const std::vector<TerrainSample>& samples,
                                   const std::vector<const std::vector<std::size_t>*>& buckets) const {
        if (buckets.empty()) {
            return SampleTargetHeight(center, normal, plane_height_m, x, z, samples);
        }
        const int col = static_cast<int>(
            std::floor((x - grid_world_origin_x_) / std::max(1.0e-6f, config_.cell_size_m)));
        const int row = static_cast<int>(
            std::floor((z - grid_world_origin_z_) / std::max(1.0e-6f, config_.cell_size_m)));
        const std::size_t idx = static_cast<std::size_t>(std::clamp(row, 0, config_.rows - 1) * config_.cols +
                                                         std::clamp(col, 0, config_.cols - 1));
        if (idx < buckets.size() && buckets[idx] != nullptr) {
            return SampleTargetHeightIndices(center, normal, plane_height_m, x, z, samples, *buckets[idx]);
        }
        return SampleTargetHeight(center, normal, plane_height_m, x, z, samples);
    }

    float SampleTargetConfidenceIndices(const std::vector<TerrainSample>& samples,
                                        float x,
                                        float z,
                                        const std::vector<std::size_t>& indices) const {
        float weight_sum = config_.plane_confidence;
        for (std::size_t i : indices) {
            if (i >= samples.size()) {
                continue;
            }
            const TerrainSample& sample = samples[i];
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

    float SampleTargetHeightIndices(const Vec3& center,
                                    const Vec3& normal,
                                    float plane_height_m,
                                    float x,
                                    float z,
                                    const std::vector<TerrainSample>& samples,
                                    const std::vector<std::size_t>& indices) const {
        const float plane_height = PlaneHeightAt(center, normal, plane_height_m, x, z);
        float weighted_height = plane_height * config_.plane_confidence;
        float weight_sum = config_.plane_confidence;

        for (std::size_t i : indices) {
            if (i >= samples.size()) {
                continue;
            }
            const TerrainSample& sample = samples[i];
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
