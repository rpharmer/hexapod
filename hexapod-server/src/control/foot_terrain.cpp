#include "foot_terrain.hpp"

#include "kinematics/math_types.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

[[nodiscard]] Mat3 bodyToWorldRot(const RobotState& est) {
    if (!est.has_body_twist_state) {
        return Mat3::identity();
    }
    const double roll = est.body_twist_state.twist_pos_rad.x;
    const double pitch = est.body_twist_state.twist_pos_rad.y;
    const double yaw = est.body_twist_state.twist_pos_rad.z;
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
        return Mat3::identity();
    }
    return Mat3::rotZ(yaw) * Mat3::rotY(pitch) * Mat3::rotX(roll);
}

[[nodiscard]] Vec3 bodyPointToWorld(const RobotState& est, const Vec3& p_body) {
    if (!est.has_body_twist_state) {
        return p_body;
    }
    const Mat3 R = bodyToWorldRot(est);
    const Vec3 t = est.body_twist_state.body_trans_m;
    return t + (R * p_body);
}

double fusionTrustScale(const RobotState& est) {
    if (!est.has_fusion_diagnostics) {
        return 1.0;
    }
    return std::clamp(est.fusion.model_trust, 0.20, 1.0);
}

} // namespace

double sampleMaxHitZWorldM(const LocalMapSnapshot& snap, const double world_x_m, const double world_y_m) {
    if (!snap.elevation_has_data || snap.elevation_max_hit_z.empty() || snap.raw.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    int cx = 0;
    int cy = 0;
    if (!snap.raw.worldToCell(world_x_m, world_y_m, cx, cy)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return snap.elevation_max_hit_z.maxHitZAtCell(cx, cy);
}

double sampleGroundMeanZWorldM(const LocalMapSnapshot& snap, const double world_x_m, const double world_y_m) {
    if (!snap.ground_elevation_has_data || snap.elevation_ground_mean_z.empty() || snap.raw.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    int cx = 0;
    int cy = 0;
    if (!snap.raw.worldToCell(world_x_m, world_y_m, cx, cy)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return snap.elevation_ground_mean_z.maxHitZAtCell(cx, cy);
}

void applyTerrainStanceZBias(const LocalMapSnapshot& snap,
                             const RobotState& est,
                             const MotionIntent& intent,
                             const control_config::FootTerrainConfig& cfg,
                             const double blend_scale,
                             std::array<Vec3, kNumLegs>* nominal_body_m) {
    if (!nominal_body_m || !est.has_body_twist_state) {
        return;
    }
    if (!cfg.enable_stance_plane_bias || !snap.fresh || !snap.ground_elevation_has_data ||
        cfg.stance_plane_blend <= 0.0) {
        return;
    }

    const Vec3 planar_body_offset = Vec3{intent.twist.body_trans_m.x, intent.twist.body_trans_m.y, 0.0};

    std::array<double, kNumLegs> gz{};
    int finite_count = 0;
    double sum = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        const Vec3 foot_b = (*nominal_body_m)[static_cast<std::size_t>(leg)] - planar_body_offset;
        const Vec3 p_w = bodyPointToWorld(est, foot_b);
        const double zg = sampleGroundMeanZWorldM(snap, p_w.x, p_w.y);
        gz[static_cast<std::size_t>(leg)] = zg;
        if (std::isfinite(zg)) {
            sum += zg;
            ++finite_count;
        }
    }

    const int min_s = std::max(1, cfg.stance_ground_min_samples);
    if (finite_count < min_s) {
        return;
    }

    const double mean = sum / static_cast<double>(finite_count);
    const double dz_cap = std::max(0.0, cfg.stance_plane_dz_max_m);
    const double blend = std::clamp(cfg.stance_plane_blend * std::clamp(blend_scale, 0.0, 1.0) *
                                        fusionTrustScale(est),
                                    0.0,
                                    1.0);

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double zg = gz[static_cast<std::size_t>(leg)];
        if (!std::isfinite(zg)) {
            continue;
        }
        const double dz_world = zg - mean;
        const double dz_body = std::clamp(blend * dz_world, -dz_cap, dz_cap);
        (*nominal_body_m)[static_cast<std::size_t>(leg)].z += dz_body;
    }
}

void applyTerrainSwingXYNudge(const LocalMapSnapshot& snap,
                              const RobotState& est,
                              const control_config::FootTerrainConfig& cfg,
                              const double tau01,
                              Vec3* foot_pos_body_m) {
    if (!foot_pos_body_m || !est.has_body_twist_state) {
        return;
    }
    if (!cfg.enable_swing_xy_nudge || !snap.fresh || !snap.elevation_has_data ||
        cfg.swing_xy_nudge_max_m <= 0.0 ||
        cfg.swing_xy_nudge_blend <= 0.0) {
        return;
    }
    if (tau01 + 1e-9 < cfg.swing_xy_nudge_tau_min) {
        return;
    }

    const Mat3 R = bodyToWorldRot(est);
    const Vec3 t = est.body_twist_state.body_trans_m;
    const Vec3 p_w = t + (R * (*foot_pos_body_m));
    if (!std::isfinite(p_w.x) || !std::isfinite(p_w.y)) {
        return;
    }

    int cx = 0;
    int cy = 0;
    if (!snap.raw.worldToCell(p_w.x, p_w.y, cx, cy)) {
        return;
    }

    const int Rcells = std::clamp(cfg.swing_xy_nudge_window_cells, 0, 8);
    double best_z = std::numeric_limits<double>::infinity();
    int best_x = cx;
    int best_y = cy;
    bool found = false;
    for (int dy = -Rcells; dy <= Rcells; ++dy) {
        for (int dx = -Rcells; dx <= Rcells; ++dx) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (!snap.raw.containsCell(nx, ny)) {
                continue;
            }
            const NavPose2d cell_pose = snap.raw.cellCenterPose(nx, ny);
            const double z_ground = sampleGroundMeanZWorldM(snap, cell_pose.x_m, cell_pose.y_m);
            const double z = std::isfinite(z_ground) ? z_ground : snap.elevation_max_hit_z.maxHitZAtCell(nx, ny);
            if (std::isfinite(z) && z < best_z) {
                best_z = z;
                best_x = nx;
                best_y = ny;
                found = true;
            }
        }
    }
    if (!found) {
        return;
    }

    const NavPose2d cell_pose = snap.raw.cellCenterPose(best_x, best_y);
    Vec3 delta_w{cell_pose.x_m - p_w.x, cell_pose.y_m - p_w.y, 0.0};
    if (!std::isfinite(delta_w.x) || !std::isfinite(delta_w.y)) {
        return;
    }

    const Vec3 delta_b = R.transpose() * delta_w;
    const double n = std::hypot(delta_b.x, delta_b.y);
    if (n <= 1.0e-9) {
        return;
    }
    const double step = std::min(cfg.swing_xy_nudge_max_m, n) *
                        std::clamp(cfg.swing_xy_nudge_blend * fusionTrustScale(est), 0.0, 1.0);
    foot_pos_body_m->x += (delta_b.x / n) * step;
    foot_pos_body_m->y += (delta_b.y / n) * step;
}

void applyTerrainSwingClearance(const LocalMapSnapshot& snap,
                                const RobotState& est,
                                const control_config::FootTerrainConfig& cfg,
                                Vec3* foot_pos_body_m) {
    if (!foot_pos_body_m || !est.has_body_twist_state) {
        return;
    }
    if (!cfg.enable_swing_clearance || !snap.fresh || !snap.elevation_has_data) {
        return;
    }

    const Vec3 p_w = bodyPointToWorld(est, *foot_pos_body_m);
    if (!std::isfinite(p_w.x) || !std::isfinite(p_w.y)) {
        return;
    }

    const double z_top = sampleMaxHitZWorldM(snap, p_w.x, p_w.y);
    const double z_ground = sampleGroundMeanZWorldM(snap, p_w.x, p_w.y);
    const double z_ref = std::isfinite(z_ground) ? z_ground : z_top;
    if (!std::isfinite(z_ref)) {
        return;
    }

    const double foot_z_world_est = p_w.z;
    const double margin = std::max(0.0, cfg.swing_margin_m);
    const double max_lift = std::max(0.0, cfg.swing_max_lift_m);
    const double blend = std::clamp(cfg.swing_blend * fusionTrustScale(est), 0.0, 1.0);
    if (z_ref <= foot_z_world_est + margin) {
        return;
    }

    const double need_m = (z_ref - foot_z_world_est - margin) * blend;
    const double dz = std::min(max_lift, std::max(0.0, need_m));
    if (dz > 0.0) {
        foot_pos_body_m->z += dz;
    }
}
