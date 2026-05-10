#pragma once

/**
 * Shared CAD / actuator nominal constants for hexapod-server gravity feedforward and
 * hexapod-physics-sim demo scenes. Single source of truth; keep numerically aligned with
 * protocol geometry (tibia link = assembly pitch length minus spherical foot radius).
 */
namespace hexapod_dynamics {

inline constexpr double kStandardGravityMps2 = 9.80665;

// Link masses (kg) — match hexapod-physics-sim demo preset
inline constexpr double kBodyMassKg = 0.40;
inline constexpr double kCoxaMassKg = 0.055;
inline constexpr double kFemurMassKg = 0.070;
inline constexpr double kTibiaMassKg = 0.055;
inline constexpr double kFootMassKg = 0.008;

/** COM fraction along coxa (hip → femur pivot). */
inline constexpr double kCoxaComFrac = 0.5;
/** COM fraction along femur (hip → knee). */
inline constexpr double kFemurComFrac = 0.5;
/**
 * Fraction from knee (tibia pivot) toward foot along the tibia+foot line for combined
 * tibia+foot COM (matches prior analytical note ~0.563).
 */
inline constexpr double kTibiaPlusFootComFrac = 0.563;

// Rigid link lengths (m) — match server default geometry / sim scenes
inline constexpr double kCoxaLengthM = 0.043;
inline constexpr double kFemurLengthM = 0.060;
/** Tibia rigid link (assembly tibia length minus foot sphere radius). */
inline constexpr double kTibiaLinkLengthM = 0.104 - 0.018;

// Position-loop servo nominal (Catto-style PD used in sim): ωₙ (rad/s), ζ, τ_max (N·m)
inline constexpr double kServoOmegaN = 160.0;
inline constexpr double kServoZeta = 1.24;
inline constexpr double kServoMaxTorqueNm = 28.0;

inline constexpr double kServoOmegaNSq() {
    return kServoOmegaN * kServoOmegaN;
}

} // namespace hexapod_dynamics
