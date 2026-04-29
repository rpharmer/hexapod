#include "gait_params.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr std::array<double, kNumLegs> kTripodOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
constexpr std::array<double, kNumLegs> kSequentialOffsets = {
    0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};
constexpr double kMinSwingHeightM = 0.014;
constexpr double kLowSpeedYawSwingBoostM = 0.004;
constexpr double kSwingHeightFloorFractionOfBase = 0.70;

double swingHeightFloorForPreset(const GaitPresetTemplate& preset) {
    return std::max(kMinSwingHeightM, preset.base_swing_height_m * kSwingHeightFloorFractionOfBase);
}

/** Normalized walk command magnitude in [0, ~1.5] (same construction as legacy stride scaling). */
double normalizedWalkCommandMag(const double vx_mps,
                                const double vy_mps,
                                const double yaw_rate_radps,
                                const control_config::GaitConfig& gait_cfg) {
    const double v_nom = std::max(gait_cfg.nominal_planar_speed_mps, 1e-6);
    const double w_nom = std::max(gait_cfg.nominal_yaw_rate_radps, 1e-6);
    const double r_turn = std::max(gait_cfg.turn_nominal_radius_m, 1e-6);

    const double planar = std::hypot(vx_mps, vy_mps);
    const double turn_equiv = std::abs(yaw_rate_radps) * r_turn;
    const double cmd_mag =
        std::hypot(planar / v_nom, turn_equiv / w_nom) / 1.4142135623730951;
    const double cmd_clamped = std::clamp(cmd_mag, 0.0, 1.5);
    const double fallback_norm = std::clamp(gait_cfg.fallback_speed_mag.value / v_nom, 0.0, 1.0);
    return std::max(cmd_clamped, fallback_norm);
}

} // namespace

double gaitSmoothstep01(double t01) {
    const double t = std::clamp(t01, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double gaitWrap01(double x) {
    double y = x;
    while (y >= 1.0) {
        y -= 1.0;
    }
    while (y < 0.0) {
        y += 1.0;
    }
    return y;
}

double gaitLerpWrapped01(double a01, double b01, double t01) {
    const double t = std::clamp(t01, 0.0, 1.0);
    double da = b01 - a01;
    while (da > 0.5) {
        da -= 1.0;
    }
    while (da < -0.5) {
        da += 1.0;
    }
    return gaitWrap01(a01 + da * t);
}

GaitPresetTemplate gaitPresetTemplate(const GaitType gait) {
    GaitPresetTemplate out{};
    switch (gait) {
    case GaitType::TRIPOD:
        out.duty_factor = 0.5;
        out.phase_offset = kTripodOffsets;
        out.base_step_frequency_hz = 1.0;
        out.base_step_length_m = 0.06;
        out.base_swing_height_m = 0.03;
        break;
    case GaitType::RIPPLE:
        out.duty_factor = 0.56;
        out.phase_offset = kSequentialOffsets;
        out.base_step_frequency_hz = 1.15;
        out.base_step_length_m = 0.055;
        out.base_swing_height_m = 0.028;
        break;
    case GaitType::WAVE:
        out.duty_factor = 0.82;
        out.phase_offset = kSequentialOffsets;
        out.base_step_frequency_hz = 0.95;
        out.base_step_length_m = 0.05;
        out.base_swing_height_m = 0.026;
        break;
    case GaitType::CRAWL:
        out.duty_factor = 0.90;
        out.phase_offset = kSequentialOffsets;
        out.base_step_frequency_hz = 0.48;
        out.base_step_length_m = 0.04;
        out.base_swing_height_m = 0.022;
        break;
    case GaitType::TURN_IN_PLACE:
        out.duty_factor = 0.5;
        out.phase_offset = kTripodOffsets;
        out.base_step_frequency_hz = 0.85;
        out.base_step_length_m = 0.045;
        out.base_swing_height_m = 0.025;
        break;
    }
    return out;
}

double gaitPresetSwingHeightFloor(const GaitType gait) {
    return swingHeightFloorForPreset(gaitPresetTemplate(gait));
}

UnifiedGaitDescription buildTargetUnifiedGait(const GaitType gait,
                                             const double vx_mps,
                                             const double vy_mps,
                                             const double yaw_rate_radps,
                                             const control_config::GaitConfig& gait_cfg,
                                             const double cmd_ax_mps2,
                                             const double cmd_ay_mps2) {
    const GaitPresetTemplate preset = gaitPresetTemplate(gait);
    UnifiedGaitDescription desc{};
    desc.duty_factor = std::clamp(preset.duty_factor, 0.08, 0.95);
    desc.phase_offset = preset.phase_offset;

    const double speed_mag = normalizedWalkCommandMag(vx_mps, vy_mps, yaw_rate_radps, gait_cfg);
    const double a_planar = std::hypot(cmd_ax_mps2, cmd_ay_mps2);
    const double accel_cadence_boost = 1.0 + 0.12 * std::clamp(a_planar / 0.55, 0.0, 1.2);
    const double accel_stride_boost = 1.0 + 0.08 * std::clamp(a_planar / 0.55, 0.0, 1.2);

    desc.step_frequency_hz = std::clamp(
        preset.base_step_frequency_hz * (0.40 + 1.28 * speed_mag) * accel_cadence_boost, 0.28, 3.8);
    desc.step_length_m =
        std::clamp(preset.base_step_length_m * (0.30 + 1.12 * speed_mag) * accel_stride_boost, 0.012, 0.12);
    desc.swing_height_m = std::clamp(
        std::clamp(preset.base_swing_height_m * (0.36 + 1.05 * speed_mag), 0.008, 0.06) *
            std::max(gait_cfg.swing_height_scale, 1e-6),
        0.008,
        0.06);
    // Keep slow and yaw-heavy gaits from collapsing swing clearance so far that the feet start
    // scraping the floor. Forward walking still uses the nominal floor.
    const double planar_speed_mps = std::hypot(vx_mps, vy_mps);
    const double low_speed_scale = std::clamp((0.12 - planar_speed_mps) / 0.12, 0.0, 1.0);
    const double yaw_scale = std::clamp(std::abs(yaw_rate_radps) / 0.45, 0.0, 1.0);
    const double swing_height_floor_m =
        std::max(swingHeightFloorForPreset(preset),
                 kMinSwingHeightM + kLowSpeedYawSwingBoostM * low_speed_scale * (0.5 + 0.5 * yaw_scale));
    desc.swing_height_m = std::max(desc.swing_height_m, swing_height_floor_m);
    desc.swing_time_ease =
        std::clamp(0.52 + 0.46 * (1.0 - std::min(speed_mag, 1.25) / 1.25), 0.40, 1.0);
    {
        const double emin = std::min(gait_cfg.swing_ease_min, gait_cfg.swing_ease_max);
        const double emax = std::max(gait_cfg.swing_ease_min, gait_cfg.swing_ease_max);
        desc.swing_time_ease = std::clamp(desc.swing_time_ease, emin, emax);
    }

    const double hz = std::max(desc.step_frequency_hz, 1e-6);
    desc.stance_duration_s = desc.duty_factor / hz;
    desc.swing_duration_s = (1.0 - desc.duty_factor) / hz;
    return desc;
}

UnifiedGaitDescription buildAdaptiveTripodCrawlGait(const double vx_mps,
                                                    const double vy_mps,
                                                    const double yaw_rate_radps,
                                                    const double cmd_ax_mps2,
                                                    const double cmd_ay_mps2,
                                                    const control_config::GaitConfig& gait_cfg) {
    UnifiedGaitDescription out =
        buildTargetUnifiedGait(GaitType::TRIPOD, vx_mps, vy_mps, yaw_rate_radps, gait_cfg, cmd_ax_mps2, cmd_ay_mps2);
    out.swing_height_m = std::max(out.swing_height_m, swingHeightFloorForPreset(gaitPresetTemplate(GaitType::TRIPOD)));
    return out;
}

UnifiedGaitDescription buildAdaptiveRippleCrawlGait(const double vx_mps,
                                                    const double vy_mps,
                                                    const double yaw_rate_radps,
                                                    const double cmd_ax_mps2,
                                                    const double cmd_ay_mps2,
                                                    const control_config::GaitConfig& gait_cfg) {
    UnifiedGaitDescription out =
        buildTargetUnifiedGait(GaitType::RIPPLE, vx_mps, vy_mps, yaw_rate_radps, gait_cfg, cmd_ax_mps2, cmd_ay_mps2);
    out.swing_height_m = std::max(out.swing_height_m, swingHeightFloorForPreset(gaitPresetTemplate(GaitType::RIPPLE)));
    return out;
}

UnifiedGaitDescription buildAdaptiveWaveCrawlGait(const double vx_mps,
                                                  const double vy_mps,
                                                  const double yaw_rate_radps,
                                                  const double cmd_ax_mps2,
                                                  const double cmd_ay_mps2,
                                                  const control_config::GaitConfig& gait_cfg) {
    UnifiedGaitDescription out =
        buildTargetUnifiedGait(GaitType::WAVE, vx_mps, vy_mps, yaw_rate_radps, gait_cfg, cmd_ax_mps2, cmd_ay_mps2);
    out.swing_height_m = std::max(out.swing_height_m, swingHeightFloorForPreset(gaitPresetTemplate(GaitType::WAVE)));
    return out;
}

UnifiedGaitDescription blendUnifiedGait(const UnifiedGaitDescription& from,
                                      const UnifiedGaitDescription& to,
                                      const double alpha_smooth01) {
    const double a = gaitSmoothstep01(alpha_smooth01);
    UnifiedGaitDescription out{};
    out.duty_factor = std::clamp(from.duty_factor * (1.0 - a) + to.duty_factor * a, 0.08, 0.95);
    out.step_frequency_hz =
        std::clamp(from.step_frequency_hz * (1.0 - a) + to.step_frequency_hz * a, 0.32, 3.6);
    out.step_length_m = std::max(0.0, from.step_length_m * (1.0 - a) + to.step_length_m * a);
    out.swing_height_m = std::max(0.0, from.swing_height_m * (1.0 - a) + to.swing_height_m * a);
    out.swing_time_ease = std::clamp(from.swing_time_ease * (1.0 - a) + to.swing_time_ease * a, 0.0, 1.0);
    for (int i = 0; i < kNumLegs; ++i) {
        out.phase_offset[static_cast<std::size_t>(i)] =
            gaitLerpWrapped01(from.phase_offset[static_cast<std::size_t>(i)],
                               to.phase_offset[static_cast<std::size_t>(i)], a);
    }
    const double hz = std::max(out.step_frequency_hz, 1e-6);
    out.stance_duration_s = out.duty_factor / hz;
    out.swing_duration_s = (1.0 - out.duty_factor) / hz;
    return out;
}
