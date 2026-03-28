#pragma once

#include "types.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace hardware {

struct ImuSample {
    EulerAnglesRad3 orientation_rad{};
    AngularVelocityRadPerSec3 angular_velocity_radps{};
    VelocityMps3 body_linear_velocity_mps{};
    bool has_body_linear_velocity{false};
    Vec3 linear_accel_mps2{};
    Vec3 magnetic_field_ut{};
    TimePointUs timestamp_us{};
    uint64_t sample_id{0};
    bool valid{false};
};

class IImuUnit {
public:
    virtual ~IImuUnit() = default;
    virtual bool init() = 0;
    virtual bool read(ImuSample& out) = 0;
};

class DirectImuUnit final : public IImuUnit {
public:
    explicit DirectImuUnit(std::string device_path = "/dev/ttyIMU0");
    ~DirectImuUnit() override;

    bool init() override;
    bool read(ImuSample& out) override;

private:
    bool parseLine(const std::string& line, ImuSample& out) const;

    std::string device_path_;
    int fd_{-1};
    std::string line_buffer_;
    uint64_t sample_counter_{0};
};

class DummyImuUnit final : public IImuUnit {
public:
    explicit DummyImuUnit(std::chrono::milliseconds sample_period = std::chrono::milliseconds{10});

    bool init() override;
    bool read(ImuSample& out) override;

private:
    std::array<std::array<double, kJointsPerLeg>, kNumLegs> simulated_joint_pos_{};
    std::array<Vec3, kNumLegs> last_stance_points_body_m_{};
    std::array<bool, kNumLegs> last_stance_valid_{};
    EulerAnglesRad3 last_orientation_rad_{};
    AngularVelocityRadPerSec3 last_angular_velocity_radps_{};
    std::chrono::milliseconds sample_period_;
    TimePointUs next_sample_at_{};
    uint64_t sample_counter_{0};
};

std::unique_ptr<IImuUnit> makeNoopImuUnit();

} // namespace hardware
