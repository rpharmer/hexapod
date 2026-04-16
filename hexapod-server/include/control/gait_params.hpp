#pragma once

#include "control_config.hpp"
#include "types.hpp"

#include <array>

// Gait presets and blending: timing, duty, phases, cadence, and swing *timing* only — not spatial
// foot paths (those live in foot / swing trajectory planners).
struct UnifiedGaitDescription {
    double duty_factor{0.5};
    std::array<double, kNumLegs> phase_offset{};
    double step_frequency_hz{1.0};
    double step_length_m{0.06};
    double swing_height_m{0.03};
    /** Passed to swing generator: S-curve amount on normalized swing phase. */
    double swing_time_ease{1.0};
    double stance_duration_s{0.5};
    double swing_duration_s{0.5};
};

// Static preset template (before velocity-dependent scaling).
struct GaitPresetTemplate {
    double duty_factor{0.5};
    std::array<double, kNumLegs> phase_offset{};
    double base_step_frequency_hz{1.0};
    double base_step_length_m{0.06};
    double base_swing_height_m{0.03};
};

GaitPresetTemplate gaitPresetTemplate(GaitType gait);

UnifiedGaitDescription buildTargetUnifiedGait(GaitType gait,
                                              double vx_mps,
                                              double vy_mps,
                                              double yaw_rate_radps,
                                              const control_config::GaitConfig& gait_cfg,
                                              double cmd_ax_mps2 = 0.0,
                                              double cmd_ay_mps2 = 0.0);

/** Blends CRAWL-like timing at low command magnitude with TRIPOD at high (for `GaitType::TRIPOD` intent). */
UnifiedGaitDescription buildAdaptiveTripodCrawlGait(double vx_mps,
                                                     double vy_mps,
                                                     double yaw_rate_radps,
                                                     double cmd_ax_mps2,
                                                     double cmd_ay_mps2,
                                                     const control_config::GaitConfig& gait_cfg);

/** Same pattern for `GaitType::RIPPLE`: crawl-like at low speed, ripple at high. */
UnifiedGaitDescription buildAdaptiveRippleCrawlGait(double vx_mps,
                                                     double vy_mps,
                                                     double yaw_rate_radps,
                                                     double cmd_ax_mps2,
                                                     double cmd_ay_mps2,
                                                     const control_config::GaitConfig& gait_cfg);

/** Same pattern for `GaitType::WAVE`: crawl-like at low speed, wave at high. */
UnifiedGaitDescription buildAdaptiveWaveCrawlGait(double vx_mps,
                                                  double vy_mps,
                                                  double yaw_rate_radps,
                                                  double cmd_ax_mps2,
                                                  double cmd_ay_mps2,
                                                  const control_config::GaitConfig& gait_cfg);

UnifiedGaitDescription blendUnifiedGait(const UnifiedGaitDescription& from,
                                        const UnifiedGaitDescription& to,
                                        double alpha_smooth01);

double gaitSmoothstep01(double t01);
double gaitWrap01(double x);
double gaitLerpWrapped01(double a01, double b01, double t01);
