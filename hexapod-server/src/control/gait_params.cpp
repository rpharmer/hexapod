#include "gait_params.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr std::array<double, kNumLegs> kTripodOffsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5};
constexpr std::array<double, kNumLegs> kSequentialOffsets = {
    0.0, 1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 4.0 / 6.0, 5.0 / 6.0};

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

UnifiedGaitDescription buildTargetUnifiedGait(const GaitType gait,
                                             const double vx_mps,
                                             const double vy_mps,
                                             const double yaw_rate_radps,
                                             const control_config::GaitConfig& gait_cfg) {
    const GaitPresetTemplate preset = gaitPresetTemplate(gait);
    UnifiedGaitDescription desc{};
    desc.duty_factor = std::clamp(preset.duty_factor, 0.08, 0.95);
    desc.phase_offset = preset.phase_offset;

    const double v_nom = std::max(gait_cfg.nominal_planar_speed_mps, 1e-6);
    const double w_nom = std::max(gait_cfg.nominal_yaw_rate_radps, 1e-6);
    const double r_turn = std::max(gait_cfg.turn_nominal_radius_m, 1e-6);

    const double planar = std::hypot(vx_mps, vy_mps);
    const double turn_equiv = std::abs(yaw_rate_radps) * r_turn;
    const double cmd_mag =
        std::hypot(planar / v_nom, turn_equiv / w_nom) / 1.4142135623730951;
    const double cmd_clamped = std::clamp(cmd_mag, 0.0, 1.5);
    const double fallback_norm = std::clamp(gait_cfg.fallback_speed_mag.value / v_nom, 0.0, 1.0);
    const double speed_mag = std::max(cmd_clamped, fallback_norm);

    desc.step_frequency_hz =
        std::clamp(preset.base_step_frequency_hz * (0.48 + 1.15 * speed_mag), 0.32, 3.6);
    desc.step_length_m =
        std::clamp(preset.base_step_length_m * (0.32 + 1.05 * speed_mag), 0.012, 0.12);
    desc.swing_height_m =
        std::clamp(preset.base_swing_height_m * (0.38 + 1.0 * speed_mag), 0.008, 0.06);

    const double hz = std::max(desc.step_frequency_hz, 1e-6);
    desc.stance_duration_s = desc.duty_factor / hz;
    desc.swing_duration_s = (1.0 - desc.duty_factor) / hz;
    return desc;
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
