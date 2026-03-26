#include "imu_unit.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <string_view>
#include <unistd.h>

namespace hardware {
namespace {

class NoopImuUnit final : public IImuUnit {
public:
    bool init() override { return true; }
    bool read(ImuSample&) override { return false; }
};

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
    sample_counter_ = 0;
    next_sample_at_ = now_us();
    return true;
}

bool DummyImuUnit::read(ImuSample& out) {
    const TimePointUs now = now_us();
    if (!next_sample_at_.isZero() && now.value < next_sample_at_.value) {
        return false;
    }

    const double t = static_cast<double>(sample_counter_) *
                     static_cast<double>(sample_period_.count()) * 1e-3;

    out = ImuSample{};
    out.sample_id = ++sample_counter_;
    out.timestamp_us = now;
    out.orientation_rad = EulerAnglesRad3{0.04 * std::sin(0.8 * t),
                                          0.03 * std::sin(0.6 * t + 0.3),
                                          0.2 * std::sin(0.25 * t)};
    out.angular_velocity_radps =
        AngularVelocityRadPerSec3{0.032 * std::cos(0.8 * t),
                                  0.018 * std::cos(0.6 * t + 0.3),
                                  0.05 * std::cos(0.25 * t)};
    out.linear_accel_mps2 = Vec3{0.1 * std::sin(1.1 * t), 0.08 * std::cos(0.9 * t), 9.81};
    out.magnetic_field_ut = Vec3{22.0, 2.0 * std::sin(0.1 * t), 41.0};
    out.valid = true;

    next_sample_at_ = TimePointUs{now.value + static_cast<uint64_t>(sample_period_.count()) * 1000ULL};
    return true;
}

std::unique_ptr<IImuUnit> makeNoopImuUnit() {
    return std::make_unique<NoopImuUnit>();
}

} // namespace hardware
