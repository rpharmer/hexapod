#pragma once

#include "kinematics/math_types.hpp"

/**
 * Body-frame rigid twist used for locomotion (roadmap: twist-field layer).
 * Stance feet oppose this motion so world-fixed contacts stay planted; swing and foothold
 * planners consume the same twist (often filtered / predicted over T_swing).
 */
struct BodyTwist {
    Vec3 linear_mps{};
    Vec3 angular_radps{};
};

/** Legacy name used across foot planners and body controller. */
using BodyVelocityCommand = BodyTwist;

/**
 * Kinematic twist field: velocity of a body-fixed point p (m, body frame) under twist (v, ω).
 * v(p) = v + ω × p. Stance support uses the world-fixed-foot convention v_stance ≈ −v(p).
 */
struct TwistField {
    static Vec3 pointVelocity(const BodyTwist& twist, const Vec3& p_body) {
        return twist.linear_mps + cross(twist.angular_radps, p_body);
    }

    static Vec3 stanceFootVelocity(const BodyTwist& twist, const Vec3& foot_body) {
        return pointVelocity(twist, foot_body) * -1.0;
    }

    /**
     * First-order body-frame displacement over `dt` (s) of a world-fixed contact at `foot_body`
     * (m, body frame). Equals `stanceFootVelocity * dt` — the integral used for stance tracking
     * and twist-aware foothold lookahead.
     */
    static Vec3 worldFixedContactBodyDelta(const BodyTwist& twist, const Vec3& foot_body, double dt) {
        const double t = dt > 0.0 ? dt : 0.0;
        return stanceFootVelocity(twist, foot_body) * t;
    }
};
