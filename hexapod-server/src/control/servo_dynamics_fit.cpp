#include "servo_dynamics_fit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

double legJoint(const LegState& leg_state, int joint) {
    return leg_state.joint_state[joint].pos_rad.value;
}

SingleJointDynamicsFit fitSingleJointDirection(
    const std::vector<ServoDynamicsSample>& samples,
    int joint,
    bool positive_direction,
    const ServoDynamicsFitOptions& options) {
    SingleJointDynamicsFit fit{};
    if (samples.size() < 2) {
        return fit;
    }

    std::vector<std::pair<double, double>> err_and_velocity{};
    err_and_velocity.reserve(samples.size() - 1);
    double vmax_observed = 0.0;

    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].time_s - samples[i - 1].time_s;
        if (dt < options.min_dt_s) {
            continue;
        }

        const double measured_prev = legJoint(samples[i - 1].measured_servo_angles, joint);
        const double measured_now = legJoint(samples[i].measured_servo_angles, joint);
        const double commanded_now = legJoint(samples[i].command_servo_angles, joint);
        const double velocity = (measured_now - measured_prev) / dt;
        const double err = commanded_now - measured_prev;

        if (std::abs(err) < options.min_err_rad) {
            continue;
        }

        if (positive_direction && velocity <= 0.0) {
            continue;
        }
        if (!positive_direction && velocity >= 0.0) {
            continue;
        }

        vmax_observed = std::max(vmax_observed, std::abs(velocity));
        err_and_velocity.emplace_back(err, velocity);
    }

    if (static_cast<int>(err_and_velocity.size()) < options.min_samples ||
        vmax_observed <= 0.0) {
        return fit;
    }

    const double unsat_v = options.unsaturated_velocity_fraction * vmax_observed;
    double err_sq_sum = 0.0;
    double err_vel_sum = 0.0;
    int used = 0;

    for (const auto& [err, vel] : err_and_velocity) {
        if (std::abs(vel) >= unsat_v) {
            continue;
        }
        err_sq_sum += err * err;
        err_vel_sum += err * vel;
        ++used;
    }

    if (used < options.min_samples || err_sq_sum <= 1e-12) {
        return fit;
    }

    const double k = err_vel_sum / err_sq_sum; // dq = k * err
    if (!std::isfinite(k) || std::abs(k) <= 1e-9) {
        return fit;
    }

    fit.success = true;
    fit.tau_s = 1.0 / std::abs(k);
    fit.vmax_radps = vmax_observed;
    fit.used_samples = used;
    return fit;
}
