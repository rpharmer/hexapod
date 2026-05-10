#pragma once

// TerrainPatch: local height field (belief surface) over XZ with optional physics
// collision cache. LiDAR / RaycastWorld use bilinear SampleHeightWorld; collision uses
// the same field directly with an optional conservative top layer near steps.

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
    Real height_m{0.0};
    Real confidence{0.0};
};

struct TerrainPatchConfig {
    int rows{15};
    int cols{15};
    Real cell_size_m{0.05};
    Real base_margin_m{0.08};
    Real min_cell_thickness_m{0.012};
    Real influence_sigma_m{0.12};
    Real plane_confidence{0.12};
    Real confidence_half_life_s{1.5};
    Real base_update_blend{0.35};
    Real decay_update_boost{0.45};
    /// When true, SampleTarget* only scans terrain samples in spatial bins near each cell (see sample_bin_size_m).
    bool use_sample_binning{false};
    Real sample_bin_size_m{0.25};
    /// When true, physics box tops use max of neighbours (conservative vs bilinear belief).
    bool use_conservative_collision{false};
    /// World-anchored grid: blit overlapping heights when the patch centre moves in XZ.
    bool scroll_world_fixed{false};
    /// After each sim step, fuse matrix LiDAR ground hits (down-weight vs feet in correction path).
    bool lidar_fusion_enable{false};
    int lidar_sample_stride{4};
    /// Scales per-beam confidence before exp-based weighting (0 disables lidar fusion in practice).
    Real lidar_sample_weight{0.18};
    /// Minimum existing belief confidence required before a LiDAR terrain hit can reinforce the map.
    Real lidar_min_surface_confidence{0.10};
    /// Near confirmed stance feet, contact samples arbitrate against LiDAR terrain samples.
    Real lidar_contact_arbitration_radius_m{0.10};
    /// LiDAR samples that disagree vertically with nearby stance contact by more than this are gated.
    Real lidar_contact_disagreement_m{0.05};
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
    Real cell_size_m() const { return config_.cell_size_m; }
    const Vec3& center_world() const { return center_world_; }
    Real base_height_m() const { return base_height_m_; }
    Real grid_origin_x() const { return grid_world_origin_x_; }
    Real grid_origin_z() const { return grid_world_origin_z_; }
    Vec3 grid_origin_world() const { return {grid_world_origin_x_, 0.0, grid_world_origin_z_}; }
    const Vec3& last_normal() const { return last_normal_; }
    Real last_plane_height_m() const { return last_plane_height_m_; }
    const std::vector<float>& surface_heights_m() const { return surface_heights_; }
    const std::vector<float>& confidences() const { return confidences_; }
    /// Populated when use_conservative_collision; same layout as surface_heights_m.
    const std::vector<float>& collision_heights_m() const { return collision_heights_; }
    bool has_collision_layer() const { return config_.use_conservative_collision && !collision_heights_.empty(); }
    World::TerrainHeightfieldAttachment BuildTerrainHeightfieldAttachment() const {
        World::TerrainHeightfieldAttachment attachment{};
        attachment.enabled = initialized_ && config_.rows > 1 && config_.cols > 1
            && surface_heights_.size() == static_cast<std::size_t>(config_.rows * config_.cols);
        attachment.rows = config_.rows;
        attachment.cols = config_.cols;
        attachment.cellSizeM = config_.cell_size_m;
        attachment.gridOriginWorld = grid_origin_world();
        attachment.centerWorld = center_world_;
        attachment.planeNormal = last_normal_;
        attachment.planeHeightM = last_plane_height_m_;
        attachment.baseHeightM = base_height_m_;
        attachment.useConservativeCollision = config_.use_conservative_collision;
        attachment.surfaceHeightsM = surface_heights_;
        if (has_collision_layer()) {
            attachment.collisionHeightsM = collision_heights_;
        }
        return attachment;
    }

    void initialize(World& world, const Vec3& center, Real plane_height_m, const Vec3& normal = {0.0, 1.0, 0.0}) {
        (void)world;
        if (config_.rows <= 0 || config_.cols <= 0) {
            return;
        }

        center_world_ = center;
        const Real half_span_x = 0.5 * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const Real half_span_z = 0.5 * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        grid_world_origin_x_ = center_world_.x - half_span_x;
        grid_world_origin_z_ = center_world_.z - half_span_z;
        scroll_state_valid_ = config_.scroll_world_fixed;

        base_height_m_ = plane_height_m - config_.base_margin_m;
        last_plane_height_m_ = plane_height_m;
        if (Length(normal) > 1.0e-6) {
            last_normal_ = Normalize(normal);
        } else {
            last_normal_ = {0.0, 1.0, 0.0};
        }

        const std::size_t cell_count = static_cast<std::size_t>(config_.rows * config_.cols);
        body_ids_.clear();
        surface_heights_.assign(cell_count, plane_height_m);
        confidences_.assign(cell_count, config_.plane_confidence);
        collision_heights_.assign(cell_count, plane_height_m);
        initialized_ = true;
    }

    void update(World& world,
                const Vec3& center,
                Real plane_height_m,
                const Vec3& normal,
                const std::vector<TerrainSample>& samples,
                Real age_seconds = 0.0) {
        if (!initialized_) {
            initialize(world, center, plane_height_m);
        }
        if (!initialized_) {
            return;
        }

        center_world_ = center;
        last_plane_height_m_ = plane_height_m;
        if (Length(normal) > 1.0e-6) {
            last_normal_ = Normalize(normal);
        }

        const Real half_span_x = 0.5 * static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const Real half_span_z = 0.5 * static_cast<float>(config_.rows - 1) * config_.cell_size_m;
        const Real xmin_new = center_world_.x - half_span_x;
        const Real zmin_new = center_world_.z - half_span_z;

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

        const Real age_decay = ComputeAgeDecay(age_seconds);
        const Real blend = ComputeBlend(age_decay);

        std::vector<std::vector<std::size_t>> cell_sample_lists;
        std::vector<const std::vector<std::size_t>*> cell_sample_buckets;
        if (config_.use_sample_binning && samples.size() >= 8) {
            FillSampleBinsForCells(samples, cell_sample_lists, cell_sample_buckets);
        }

        Real min_height = std::numeric_limits<Real>::infinity();
        for (int row = 0; row < config_.rows; ++row) {
            for (int col = 0; col < config_.cols; ++col) {
                const Real x = grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m;
                const Real z = grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m;

                const Real target_height =
                    SampleTargetHeightBinned(center_world_, last_normal_, plane_height_m, x, z, samples, cell_sample_buckets);
                const Real target_confidence =
                    SampleTargetConfidenceBinned(samples, x, z, cell_sample_buckets);
                const std::size_t idx = Index(row, col);

                surface_heights_[idx] = surface_heights_[idx] * (1.0 - blend) + target_height * blend;
                confidences_[idx] = confidences_[idx] * (1.0 - blend) + target_confidence * blend;
                min_height = std::min(min_height, static_cast<Real>(surface_heights_[idx]));
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
                    Real mx = surface_heights_[idx];
                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            const int nr = row + dr;
                            const int nc = col + dc;
                            if (nr < 0 || nr >= config_.rows || nc < 0 || nc >= config_.cols) {
                                continue;
                            }
                            mx = std::max(mx, static_cast<Real>(surface_heights_[Index(nr, nc)]));
                        }
                    }
                    collision_heights_[idx] = mx;
                }
            }
        } else {
            collision_heights_.clear();
        }
    }

    Real SampleHeightWorld(Real x, Real z) const {
        Real conf_ignored = 0.0;
        return SampleHeightAndConfidenceWorld(x, z, &conf_ignored);
    }

    /// Bilinear height and confidence at (x,z) for planner / diagnostics.
    Real SampleHeightAndConfidenceWorld(Real x, Real z, Real* out_confidence) const {
        if (!initialized_ || config_.rows <= 0 || config_.cols <= 0 || surface_heights_.empty()) {
            if (out_confidence != nullptr) {
                *out_confidence = 0.0;
            }
            return last_plane_height_m_;
        }

        const GridCoord coord = ToGridCoord(x, z);
        const int x0 = coord.x0;
        const int x1 = coord.x1;
        const int z0 = coord.z0;
        const int z1 = coord.z1;
        const Real tx = coord.tx;
        const Real tz = coord.tz;

        const Real h00 = surface_heights_[Index(z0, x0)];
        const Real h10 = surface_heights_[Index(z0, x1)];
        const Real h01 = surface_heights_[Index(z1, x0)];
        const Real h11 = surface_heights_[Index(z1, x1)];
        const Real hx0 = h00 * (1.0 - tx) + h10 * tx;
        const Real hx1 = h01 * (1.0 - tx) + h11 * tx;
        const Real h = hx0 * (1.0 - tz) + hx1 * tz;

        if (out_confidence != nullptr) {
            const Real c00 = confidences_[Index(z0, x0)];
            const Real c10 = confidences_[Index(z0, x1)];
            const Real c01 = confidences_[Index(z1, x0)];
            const Real c11 = confidences_[Index(z1, x1)];
            const Real cx0 = c00 * (1.0 - tx) + c10 * tx;
            const Real cx1 = c01 * (1.0 - tx) + c11 * tx;
            *out_confidence = cx0 * (1.0 - tz) + cx1 * tz;
        }
        return h;
    }

    Vec3 SampleNormalWorld(Real x, Real z) const {
        const Real delta = std::max(0.5 * config_.cell_size_m, 1.0e-3);
        const Real h_l = SampleHeightWorld(x - delta, z);
        const Real h_r = SampleHeightWorld(x + delta, z);
        const Real h_d = SampleHeightWorld(x, z - delta);
        const Real h_u = SampleHeightWorld(x, z + delta);
        Vec3 n{h_l - h_r, 2.0 * delta, h_d - h_u};
        if (!TryNormalize(n, n)) {
            return {0.0, 1.0, 0.0};
        }
        return n;
    }

    bool RaycastWorld(const Vec3& origin, const Vec3& dir_unit, Real& t_out) const {
        if (!initialized_) {
            return false;
        }
        const Vec3 d = Normalize(dir_unit);
        if (!std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z)) {
            return false;
        }

        constexpr Real kMaxRaycastMeters = 8.0;
        const Real xmin = grid_world_origin_x_;
        const Real zmin = grid_world_origin_z_;
        const Real xmax = xmin + static_cast<float>(config_.cols - 1) * config_.cell_size_m;
        const Real zmax = zmin + static_cast<float>(config_.rows - 1) * config_.cell_size_m;

        auto height_residual = [&](Real t) {
            const Vec3 p = origin + d * t;
            return p.y - SampleHeightWorld(p.x, p.z);
        };

        const Real h0 = height_residual(0.0);
        if (h0 <= 0.0) {
            return false;
        }

        const Real kEps = 1.0e-5;
        if (std::abs(d.x) < 1.0e-6 && std::abs(d.z) < 1.0e-6) {
            if (origin.x < xmin - kEps || origin.x > xmax + kEps || origin.z < zmin - kEps || origin.z > zmax + kEps) {
                return false;
            }
            if (std::abs(d.y) < 1.0e-6) {
                return false;
            }
            const Real h_ground = SampleHeightWorld(origin.x, origin.z);
            const Real t_hit = (h_ground - origin.y) / d.y;
            if (!std::isfinite(t_hit) || t_hit <= kEps || t_hit > kMaxRaycastMeters) {
                return false;
            }
            if (height_residual(t_hit) > 1.0e-3) {
                return false;
            }
            t_out = t_hit;
            return true;
        }

        Real t_enter = 0.0;
        Real t_exit = kMaxRaycastMeters;
        if (!RaySlabEnterExitXz(origin, d, xmin, xmax, zmin, zmax, t_enter, t_exit)) {
            return false;
        }
        if (t_exit <= kEps) {
            return false;
        }
        Real t = std::max(0.0, t_enter);
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

            const Real cell_x0 = xmin + static_cast<float>(col) * config_.cell_size_m;
            const Real cell_z0 = zmin + static_cast<float>(row) * config_.cell_size_m;
            Real dt_x = std::numeric_limits<float>::infinity();
            Real dt_z = std::numeric_limits<float>::infinity();
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

            Real dt = std::min(dt_x, dt_z);
            if (!std::isfinite(dt)) {
                dt = std::max(dt_x, dt_z);
            }
            if (!std::isfinite(dt) || dt <= 0.0) {
                dt = t_exit - t;
            }
            Real t_end = std::min(t_exit, t + dt);
            t_end = std::max(t_end, t + 1.0e-5);
            Real analytic_hit = 0.0;
            if (FindBilinearCellHit(row, col, origin, d, cell_x0, cell_z0, t, t_end, analytic_hit)) {
                t_out = analytic_hit;
                return true;
            }
            t = t_end + 1.0e-4;
        }
        return false;
    }

private:
    struct GridCoord {
        int x0{};
        int x1{};
        int z0{};
        int z1{};
        Real tx{0.0};
        Real tz{0.0};
    };

    TerrainPatchConfig config_{};
    std::vector<std::uint32_t> body_ids_{};
    std::vector<float> surface_heights_{};
    std::vector<float> confidences_{};
    std::vector<float> collision_heights_{};
    bool initialized_{false};
    Vec3 center_world_{0.0, 0.0, 0.0};
    Real base_height_m_{0.0};
    Vec3 last_normal_{0.0, 1.0, 0.0};
    Real last_plane_height_m_{0.0};
    Real grid_world_origin_x_{0.0};
    Real grid_world_origin_z_{0.0};
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

    GridCoord ToGridCoord(Real x, Real z) const {
        const Real grid_x = (x - grid_world_origin_x_) / config_.cell_size_m;
        const Real grid_z = (z - grid_world_origin_z_) / config_.cell_size_m;

        const int x0 = ClampCol(static_cast<int>(std::floor(grid_x)));
        const int z0 = ClampRow(static_cast<int>(std::floor(grid_z)));
        const int x1 = ClampCol(x0 + 1);
        const int z1 = ClampRow(z0 + 1);
        const Real tx = std::clamp(grid_x - static_cast<float>(x0), 0.0, 1.0);
        const Real tz = std::clamp(grid_z - static_cast<float>(z0), 0.0, 1.0);
        return GridCoord{x0, x1, z0, z1, tx, tz};
    }

    static Real EvalQuadratic(Real a, Real b, Real c, Real t) {
        return (a * t + b) * t + c;
    }

    bool FindBilinearCellHit(int row,
                             int col,
                             const Vec3& origin,
                             const Vec3& d,
                             Real cell_x0,
                             Real cell_z0,
                             Real t0,
                             Real t1,
                             Real& t_hit) const {
        constexpr Real kCoeffEps = 1.0e-8;
        constexpr Real kTimeEps = 1.0e-5;
        constexpr Real kResidualEps = 1.0e-5;
        if (t1 < t0) {
            return false;
        }

        const Real inv_cell = 1.0 / std::max(config_.cell_size_m, 1.0e-6);
        const Real h00 = surface_heights_[Index(row, col)];
        const Real h10 = surface_heights_[Index(row, col + 1)];
        const Real h01 = surface_heights_[Index(row + 1, col)];
        const Real h11 = surface_heights_[Index(row + 1, col + 1)];
        const Real base = h00;
        const Real hx = h10 - h00;
        const Real hz = h01 - h00;
        const Real hxz = h00 - h10 - h01 + h11;

        const Real tx0 = (origin.x - cell_x0) * inv_cell;
        const Real tz0 = (origin.z - cell_z0) * inv_cell;
        const Real txv = d.x * inv_cell;
        const Real tzv = d.z * inv_cell;

        const Real qa = -hxz * txv * tzv;
        const Real qb = d.y - hx * txv - hz * tzv - hxz * (tx0 * tzv + tz0 * txv);
        const Real qc = origin.y - base - hx * tx0 - hz * tz0 - hxz * tx0 * tz0;

        const Real fa = EvalQuadratic(qa, qb, qc, t0);
        if (fa <= kResidualEps && t0 > kTimeEps) {
            t_hit = t0;
            return true;
        }

        Real best = std::numeric_limits<float>::infinity();
        auto consider_root = [&](Real root) {
            if (!std::isfinite(root) || root < t0 - kTimeEps || root > t1 + kTimeEps || root <= kTimeEps) {
                return;
            }
            const Real clamped = std::clamp(root, t0, t1);
            if (std::abs(EvalQuadratic(qa, qb, qc, clamped)) <= 2.0e-4) {
                best = std::min(best, clamped);
            }
        };

        if (std::abs(qa) <= kCoeffEps) {
            if (std::abs(qb) > kCoeffEps) {
                consider_root(-qc / qb);
            } else if (std::abs(qc) <= kResidualEps && t0 > kTimeEps) {
                best = t0;
            }
        } else {
            Real disc = qb * qb - 4.0 * qa * qc;
            const Real disc_scale = std::max(1.0, qb * qb + std::abs(qa * qc));
            if (disc < 0.0 && disc > -1.0e-6 * disc_scale) {
                disc = 0.0;
            }
            if (disc >= 0.0) {
                const Real sqrt_disc = std::sqrt(disc);
                consider_root((-qb - sqrt_disc) / (2.0 * qa));
                consider_root((-qb + sqrt_disc) / (2.0 * qa));
            }
        }

        const Real fb = EvalQuadratic(qa, qb, qc, t1);
        if (fb <= kResidualEps && t1 > kTimeEps) {
            best = std::min(best, t1);
        }

        if (std::isfinite(best)) {
            t_hit = best;
            return true;
        }
        return false;
    }

    static bool RaySlabEnterExitXz(const Vec3& o,
                                   const Vec3& d,
                                   Real xmin,
                                   Real xmax,
                                   Real zmin,
                                   Real zmax,
                                   Real& t_enter,
                                   Real& t_exit) {
        Real t0 = 0.0;
        Real t1 = std::numeric_limits<float>::infinity();
        auto axis = [&](Real pos, Real dir, Real min_v, Real max_v) -> bool {
            if (std::abs(dir) < 1.0e-8) {
                if (pos < min_v || pos > max_v) {
                    return false;
                }
                return true;
            }
            Real inv = 1.0 / dir;
            Real t_near = (min_v - pos) * inv;
            Real t_far = (max_v - pos) * inv;
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
        return std::isfinite(t_exit) && t_exit > 0.0;
    }

    void ScrollBlitHeights(int d_col,
                           int d_row,
                           Real xmin_new,
                           Real zmin_new,
                           Real plane_height_m,
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
                    const Real x = xmin_new + static_cast<float>(col) * config_.cell_size_m;
                    const Real z = zmin_new + static_cast<float>(row) * config_.cell_size_m;
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
        const Real bin_w = std::max(1.0e-3, config_.sample_bin_size_m);
        const Real radius = 3.5 * config_.influence_sigma_m;
        const Real anchor_x = grid_world_origin_x_;
        const Real anchor_z = grid_world_origin_z_;

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
                const Real wx = grid_world_origin_x_ + static_cast<float>(col) * config_.cell_size_m;
                const Real wz = grid_world_origin_z_ + static_cast<float>(row) * config_.cell_size_m;
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

    Real SampleTargetConfidenceBinned(const std::vector<TerrainSample>& samples,
                                       Real x,
                                       Real z,
                                       const std::vector<const std::vector<std::size_t>*>& buckets) const {
        if (buckets.empty()) {
            return SampleTargetConfidence(samples, x, z);
        }
        const int col = static_cast<int>(
            std::floor((x - grid_world_origin_x_) / std::max(1.0e-6, config_.cell_size_m)));
        const int row = static_cast<int>(
            std::floor((z - grid_world_origin_z_) / std::max(1.0e-6, config_.cell_size_m)));
        const std::size_t idx = static_cast<std::size_t>(std::clamp(row, 0, config_.rows - 1) * config_.cols +
                                                         std::clamp(col, 0, config_.cols - 1));
        if (idx < buckets.size() && buckets[idx] != nullptr) {
            return SampleTargetConfidenceIndices(samples, x, z, *buckets[idx]);
        }
        return SampleTargetConfidence(samples, x, z);
    }

    Real SampleTargetHeightBinned(const Vec3& center,
                                   const Vec3& normal,
                                   Real plane_height_m,
                                   Real x,
                                   Real z,
                                   const std::vector<TerrainSample>& samples,
                                   const std::vector<const std::vector<std::size_t>*>& buckets) const {
        if (buckets.empty()) {
            return SampleTargetHeight(center, normal, plane_height_m, x, z, samples);
        }
        const int col = static_cast<int>(
            std::floor((x - grid_world_origin_x_) / std::max(1.0e-6, config_.cell_size_m)));
        const int row = static_cast<int>(
            std::floor((z - grid_world_origin_z_) / std::max(1.0e-6, config_.cell_size_m)));
        const std::size_t idx = static_cast<std::size_t>(std::clamp(row, 0, config_.rows - 1) * config_.cols +
                                                         std::clamp(col, 0, config_.cols - 1));
        if (idx < buckets.size() && buckets[idx] != nullptr) {
            return SampleTargetHeightIndices(center, normal, plane_height_m, x, z, samples, *buckets[idx]);
        }
        return SampleTargetHeight(center, normal, plane_height_m, x, z, samples);
    }

    Real SampleTargetConfidenceIndices(const std::vector<TerrainSample>& samples,
                                        Real x,
                                        Real z,
                                        const std::vector<std::size_t>& indices) const {
        Real weight_sum = config_.plane_confidence;
        for (std::size_t i : indices) {
            if (i >= samples.size()) {
                continue;
            }
            const TerrainSample& sample = samples[i];
            const Real dx = x - sample.world_position.x;
            const Real dz = z - sample.world_position.z;
            const Real d2 = dx * dx + dz * dz;
            const Real sigma2 = std::max(1.0e-6, config_.influence_sigma_m * config_.influence_sigma_m);
            const Real influence = std::clamp(
                sample.confidence * std::exp(-0.5 * d2 / sigma2),
                0.0,
                1.0);
            weight_sum += influence;
        }
        return std::clamp(weight_sum / (1.0 + weight_sum), 0.0, 1.0);
    }

    Real SampleTargetHeightIndices(const Vec3& center,
                                    const Vec3& normal,
                                    Real plane_height_m,
                                    Real x,
                                    Real z,
                                    const std::vector<TerrainSample>& samples,
                                    const std::vector<std::size_t>& indices) const {
        const Real plane_height = PlaneHeightAt(center, normal, plane_height_m, x, z);
        Real weighted_height = plane_height * config_.plane_confidence;
        Real weight_sum = config_.plane_confidence;

        for (std::size_t i : indices) {
            if (i >= samples.size()) {
                continue;
            }
            const TerrainSample& sample = samples[i];
            const Real dx = x - sample.world_position.x;
            const Real dz = z - sample.world_position.z;
            const Real d2 = dx * dx + dz * dz;
            const Real sigma2 = std::max(1.0e-6, config_.influence_sigma_m * config_.influence_sigma_m);
            const Real influence = std::clamp(
                sample.confidence * std::exp(-0.5 * d2 / sigma2),
                0.0,
                1.0);
            if (influence <= 1.0e-5) {
                continue;
            }
            weighted_height += sample.height_m * influence;
            weight_sum += influence;
        }

        if (weight_sum <= 1.0e-6) {
            return plane_height;
        }
        return weighted_height / weight_sum;
    }

    Real SampleTargetConfidence(const std::vector<TerrainSample>& samples, Real x, Real z) const {
        Real weight_sum = config_.plane_confidence;
        for (const TerrainSample& sample : samples) {
            const Real dx = x - sample.world_position.x;
            const Real dz = z - sample.world_position.z;
            const Real d2 = dx * dx + dz * dz;
            const Real sigma2 = std::max(1.0e-6, config_.influence_sigma_m * config_.influence_sigma_m);
            const Real influence = std::clamp(
                sample.confidence * std::exp(-0.5 * d2 / sigma2),
                0.0,
                1.0);
            weight_sum += influence;
        }
        return std::clamp(weight_sum / (1.0 + weight_sum), 0.0, 1.0);
    }

    Real SampleTargetHeight(const Vec3& center,
                             const Vec3& normal,
                             Real plane_height_m,
                             Real x,
                             Real z,
                             const std::vector<TerrainSample>& samples) const {
        const Real plane_height = PlaneHeightAt(center, normal, plane_height_m, x, z);
        Real weighted_height = plane_height * config_.plane_confidence;
        Real weight_sum = config_.plane_confidence;

        for (const TerrainSample& sample : samples) {
            const Real dx = x - sample.world_position.x;
            const Real dz = z - sample.world_position.z;
            const Real d2 = dx * dx + dz * dz;
            const Real sigma2 = std::max(1.0e-6, config_.influence_sigma_m * config_.influence_sigma_m);
            const Real influence = std::clamp(
                sample.confidence * std::exp(-0.5 * d2 / sigma2),
                0.0,
                1.0);
            if (influence <= 1.0e-5) {
                continue;
            }
            weighted_height += sample.height_m * influence;
            weight_sum += influence;
        }

        if (weight_sum <= 1.0e-6) {
            return plane_height;
        }
        return weighted_height / weight_sum;
    }

    static Real PlaneHeightAt(const Vec3& origin, const Vec3& normal, Real plane_height, Real x, Real z) {
        Vec3 n = normal;
        if (Length(n) <= 1.0e-6) {
            n = {0.0, 1.0, 0.0};
        } else {
            n = Normalize(n);
        }
        const Real ny = std::max(std::abs(n.y), 0.25);
        return plane_height - (n.x * (x - origin.x) + n.z * (z - origin.z)) / ny;
    }

    Real ComputeAgeDecay(Real age_seconds) const {
        if (config_.confidence_half_life_s <= 1.0e-6) {
            return 0.0;
        }
        if (age_seconds <= 0.0) {
            return 1.0;
        }
        const Real t = std::max(0.0, age_seconds);
        return std::pow(0.5, t / config_.confidence_half_life_s);
    }

    Real ComputeBlend(Real age_decay) const {
        const Real decay_component = 1.0 - std::clamp(age_decay, 0.0, 1.0);
        const Real blend = config_.base_update_blend + decay_component * config_.decay_update_boost;
        return std::clamp(blend, 0.0, 1.0);
    }
};

} // namespace minphys3d::demo
