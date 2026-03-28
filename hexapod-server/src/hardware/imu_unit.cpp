#include "imu_unit.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

#include "geometry_config.hpp"
#include "leg_kinematics_utils.hpp"

namespace hardware {
namespace {

class NoopImuUnit final : public IImuUnit {
public:
    bool init() override { return true; }
    bool read(ImuSample&) override { return false; }
};

constexpr std::array<double, kNumLegs> kTripodLegPhaseOffset{
    0.0, 3.14159265358979323846, 0.0, 3.14159265358979323846, 0.0, 3.14159265358979323846};

} // namespace

DirectImuUnit::DirectImuUnit(std::string device_path)
    : device_path_(std::move(device_path)) {}

DirectImuUnit::~DirectImuUnit() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool DirectImuUnit::init() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    return fd_ >= 0;
}

bool DirectImuUnit::parseLine(const std::string& line, ImuSample& out) const {
    // Expected CSV payload from the IMU forwarder:
    // roll,pitch,yaw,gx,gy,gz,ax,ay,az,mx,my,mz
    ImuSample parsed{};
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    const int assigned = std::sscanf(line.c_str(),
                                     "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                                     &roll,
                                     &pitch,
                                     &yaw,
                                     &gx,
                                     &gy,
                                     &gz,
                                     &ax,
                                     &ay,
                                     &az,
                                     &mx,
                                     &my,
                                     &mz);
    if (assigned != 12) {
        return false;
    }

    parsed.orientation_rad = EulerAnglesRad3{roll, pitch, yaw};
    parsed.angular_velocity_radps = AngularVelocityRadPerSec3{gx, gy, gz};
    parsed.linear_accel_mps2 = Vec3{ax, ay, az};
    parsed.magnetic_field_ut = Vec3{mx, my, mz};
    parsed.valid = true;

    out = parsed;
    return true;
}

bool DirectImuUnit::read(ImuSample& out) {
    if (fd_ < 0) {
        return false;
    }

    char buffer[512]{};
    bool produced_sample = false;

    while (true) {
        const ssize_t bytes = ::read(fd_, buffer, sizeof(buffer));
        if (bytes <= 0) {
            if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                return false;
            }
            break;
        }

        line_buffer_.append(buffer, static_cast<std::size_t>(bytes));

        std::size_t start = 0;
        while (true) {
            const std::size_t end = line_buffer_.find('\n', start);
            if (end == std::string::npos) {
                line_buffer_.erase(0, start);
                break;
            }

            const std::string line = line_buffer_.substr(start, end - start);
            ImuSample parsed{};
            if (parseLine(line, parsed)) {
                parsed.sample_id = ++sample_counter_;
                parsed.timestamp_us = now_us();
                out = parsed;
                produced_sample = true;
            }
            start = end + 1;
        }
    }

    return produced_sample;
}

DummyImuUnit::DummyImuUnit(std::chrono::milliseconds sample_period)
    : sample_period_(sample_period) {}

bool DummyImuUnit::init() {
    simulated_joint_pos_ = {};
    last_stance_points_body_m_ = {};
    last_stance_valid_ = {};
    last_orientation_rad_ = EulerAnglesRad3{};
    last_angular_velocity_radps_ = AngularVelocityRadPerSec3{};
    sample_counter_ = 0;
    next_sample_at_ = now_us();
    return true;
}

bool DummyImuUnit::read(ImuSample& out) {
    const TimePointUs now = now_us();
    if (!next_sample_at_.isZero() && now.value < next_sample_at_.value) {
        return false;
    }

    const double dt = std::max(1e-4, static_cast<double>(sample_period_.count()) * 1e-3);
    const double t = static_cast<double>(sample_counter_) * dt;
    const double stride_omega = 2.0 * 3.14159265358979323846 * 0.55;
    const std::array<double, kJointsPerLeg> tau_s{0.08, 0.11, 0.13};
    const std::array<double, kJointsPerLeg> vmax_radps{2.8, 3.2, 3.8};
    std::array<double, kNumLegs> phase_per_leg{};

    for (int leg = 0; leg < kNumLegs; ++leg) {
        const double phase = stride_omega * t + kTripodLegPhaseOffset[leg];
        phase_per_leg[leg] = phase;
        const std::array<double, kJointsPerLeg> desired{
            0.17 * std::sin(phase),
            -0.42 + 0.25 * std::sin(phase + 0.85),
            0.78 + 0.35 * std::sin(phase + 0.65)};

        for (int joint = 0; joint < kJointsPerLeg; ++joint) {
            const double error = desired[joint] - simulated_joint_pos_[leg][joint];
            const double alpha = clamp01(1.0 - std::exp(-dt / tau_s[joint]));
            double delta = alpha * error;
            const double max_delta = vmax_radps[joint] * dt;
            if (max_delta > 0.0) {
                delta = std::clamp(delta, -max_delta, max_delta);
            }
            simulated_joint_pos_[leg][joint] += delta;
        }
    }

    auto average_joint = [&](int leg_start, int leg_step, int joint) {
        double sum = 0.0;
        int count = 0;
        for (int leg = leg_start; leg < kNumLegs; leg += leg_step) {
            sum += simulated_joint_pos_[leg][joint];
            ++count;
        }
        return (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
    };

    const double left_femur = average_joint(0, 2, FEMUR);
    const double right_femur = average_joint(1, 2, FEMUR);
    const double left_tibia = average_joint(0, 2, TIBIA);
    const double right_tibia = average_joint(1, 2, TIBIA);
    const double front_femur =
        (simulated_joint_pos_[0][FEMUR] + simulated_joint_pos_[1][FEMUR]) * 0.5;
    const double rear_femur =
        (simulated_joint_pos_[4][FEMUR] + simulated_joint_pos_[5][FEMUR]) * 0.5;

    double left_coxa = 0.0;
    double right_coxa = 0.0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        if ((leg % 2) == 0) {
            left_coxa += simulated_joint_pos_[leg][COXA];
        } else {
            right_coxa += simulated_joint_pos_[leg][COXA];
        }
    }
    left_coxa /= 3.0;
    right_coxa /= 3.0;

    EulerAnglesRad3 orientation{};
    orientation.x = 0.22 * (right_femur - left_femur) + 0.06 * (right_tibia - left_tibia);
    orientation.y = 0.18 * (rear_femur - front_femur) + 0.03 * std::sin(stride_omega * 0.5 * t + 0.4);
    orientation.z = 0.14 * (right_coxa - left_coxa) + 0.09 * std::sin(stride_omega * 0.35 * t);

    AngularVelocityRadPerSec3 angular_velocity{};
    angular_velocity.x = (orientation.x - last_orientation_rad_.x) / dt;
    angular_velocity.y = (orientation.y - last_orientation_rad_.y) / dt;
    angular_velocity.z = (orientation.z - last_orientation_rad_.z) / dt;

    out = ImuSample{};
    out.sample_id = ++sample_counter_;
    out.timestamp_us = now;
    out.orientation_rad = orientation;
    out.angular_velocity_radps = angular_velocity;

    const HexapodGeometry& geometry = geometry_config::activeHexapodGeometry();
    Vec3 summed_planar_body_velocity_mps{};
    int stance_velocity_samples = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
        LegState synthetic_leg{};
        synthetic_leg.joint_state[COXA].pos_rad = AngleRad{simulated_joint_pos_[leg][COXA]};
        synthetic_leg.joint_state[FEMUR].pos_rad = AngleRad{simulated_joint_pos_[leg][FEMUR]};
        synthetic_leg.joint_state[TIBIA].pos_rad = AngleRad{simulated_joint_pos_[leg][TIBIA]};
        const Vec3 current_foot_point = kinematics::footInBodyFrame(synthetic_leg, geometry.legGeometry[leg]);

        const double cycle_phase = std::fmod(phase_per_leg[leg], 2.0 * 3.14159265358979323846);
        const bool in_stance = cycle_phase < 3.14159265358979323846;
        if (in_stance && last_stance_valid_[leg]) {
            const Vec3 contact_delta_body = current_foot_point - last_stance_points_body_m_[leg];
            summed_planar_body_velocity_mps =
                summed_planar_body_velocity_mps + ((-1.0 / dt) * contact_delta_body);
            ++stance_velocity_samples;
        }
        last_stance_points_body_m_[leg] = current_foot_point;
        last_stance_valid_[leg] = in_stance;
    }
    if (stance_velocity_samples > 0) {
        const Vec3 averaged_planar =
            summed_planar_body_velocity_mps * (1.0 / static_cast<double>(stance_velocity_samples));
        out.body_linear_velocity_mps = VelocityMps3{Vec3{averaged_planar.x, averaged_planar.y, 0.0}};
        out.has_body_linear_velocity = true;
    }

    out.linear_accel_mps2 = Vec3{
        0.42 * std::sin(2.0 * stride_omega * t) + 0.16 * angular_velocity.y,
        0.33 * std::cos(2.0 * stride_omega * t + 0.35) - 0.12 * angular_velocity.x,
        9.81 + 0.58 * std::sin(2.0 * stride_omega * t + 1.1)};
    out.magnetic_field_ut = Vec3{
        22.5 + 1.4 * std::cos(orientation.z),
        3.0 * std::sin(orientation.z),
        40.5 + 0.9 * std::sin(orientation.x)};
    out.valid = true;

    last_orientation_rad_ = orientation;
    last_angular_velocity_radps_ = angular_velocity;
    next_sample_at_ = TimePointUs{now.value + static_cast<uint64_t>(sample_period_.count()) * 1000ULL};
    return true;
}

std::unique_ptr<IImuUnit> makeNoopImuUnit() {
    return std::make_unique<NoopImuUnit>();
}

} // namespace hardware
