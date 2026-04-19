#include "control_config.hpp"
#include "estimator.hpp"
#include "physics_sim_bridge.hpp"
#include "robot_runtime.hpp"
#include "telemetry_publisher.hpp"
#include "types.hpp"

#include <chrono>
#include <atomic>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <vector>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#error "test_robot_runtime_physics_sim_corrections requires POSIX sockets"
#endif

namespace {

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-6) {
    return std::abs(lhs - rhs) <= eps;
}

class UdpPhysicsSimStub {
public:
    UdpPhysicsSimStub() {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            std::perror("socket");
            ok_.store(false);
            return;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(0);
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            std::perror("bind");
            ok_.store(false);
            return;
        }

        socklen_t addr_len = sizeof(addr);
        if (::getsockname(sock_, reinterpret_cast<sockaddr*>(&addr), &addr_len) != 0) {
            std::perror("getsockname");
            ok_.store(false);
            return;
        }
        port_ = ntohs(addr.sin_port);

        timeval tv{};
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        (void)::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ~UdpPhysicsSimStub() {
        stop_requested_.store(true);
        if (sock_ >= 0) {
            ::close(sock_);
            sock_ = -1;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    bool ok() const { return ok_.load(); }
    std::uint16_t port() const { return port_; }

    void start() {
        thread_ = std::thread([this]() { run(); });
    }

    int correctionCount() const { return correction_count_.load(); }
    physics_sim::StateCorrection lastCorrection() const {
        const std::lock_guard<std::mutex> lock(correction_mutex_);
        return last_correction_;
    }

private:
    void run() {
        while (!stop_requested_.load() && ok_.load()) {
            std::array<std::byte, 4096> buf{};
            sockaddr_in peer{};
            socklen_t peer_len = sizeof(peer);
            const ssize_t n = ::recvfrom(
                sock_,
                reinterpret_cast<char*>(buf.data()),
                buf.size(),
                0,
                reinterpret_cast<sockaddr*>(&peer),
                &peer_len);
            if (n < 0) {
                if (stop_requested_.load()) {
                    return;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                ok_.store(false);
                return;
            }

            const auto* bytes = reinterpret_cast<const std::byte*>(buf.data());
            const std::uint8_t mtype = static_cast<std::uint8_t>(*bytes);

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::ConfigCommand)) {
                physics_sim::ConfigAck ack{};
                ack.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::ConfigAck);
                ack.body_count = 1;
                ack.joint_count = 18;
                if (::sendto(sock_, &ack, physics_sim::kConfigAckBytes, 0,
                             reinterpret_cast<sockaddr*>(&peer), peer_len) !=
                    static_cast<ssize_t>(physics_sim::kConfigAckBytes)) {
                    ok_.store(false);
                    return;
                }
                continue;
            }

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StepCommand)) {
                physics_sim::StepCommand step{};
                if (!physics_sim::tryDecodeStepCommand(bytes, static_cast<std::size_t>(n), step)) {
                    ok_.store(false);
                    return;
                }

                physics_sim::StateResponse rsp{};
                rsp.message_type = static_cast<std::uint8_t>(physics_sim::MessageType::StateResponse);
                rsp.sequence_id = step.sequence_id;
                rsp.body_position = {0.0f, 0.0f, 0.0f};
                rsp.body_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
                rsp.body_linear_velocity = {0.0f, 0.0f, 0.0f};
                rsp.body_angular_velocity = {0.0f, 0.0f, 0.0f};
                rsp.joint_angles.fill(0.0f);
                rsp.joint_velocities.fill(0.0f);
                if (::sendto(sock_, &rsp, physics_sim::kStateResponseBytes, 0,
                             reinterpret_cast<sockaddr*>(&peer), peer_len) !=
                    static_cast<ssize_t>(physics_sim::kStateResponseBytes)) {
                    ok_.store(false);
                    return;
                }
                continue;
            }

            if (mtype == static_cast<std::uint8_t>(physics_sim::MessageType::StateCorrection)) {
                physics_sim::StateCorrection correction{};
                if (!physics_sim::tryDecodeStateCorrection(bytes, static_cast<std::size_t>(n), correction)) {
                    ok_.store(false);
                    return;
                }
                {
                    const std::lock_guard<std::mutex> lock(correction_mutex_);
                    last_correction_ = correction;
                }
                correction_count_.fetch_add(1);
                continue;
            }

            ok_.store(false);
            return;
        }
    }

    int sock_{-1};
    std::uint16_t port_{0};
    std::atomic<bool> ok_{true};
    std::atomic<bool> stop_requested_{false};
    std::atomic<int> correction_count_{0};
    mutable std::mutex correction_mutex_{};
    physics_sim::StateCorrection last_correction_{};
    std::thread thread_{};
};

class ScriptedEstimator final : public IEstimator {
public:
    void enqueue(const RobotState& sample) {
        queue_.push_back(sample);
    }

    void reset() override {
        ++reset_count;
        last_.timestamp_us = TimePointUs{};
    }

    RobotState update(const RobotState& raw) override {
        if (!queue_.empty()) {
            last_ = queue_.front();
            queue_.erase(queue_.begin());
            return last_;
        }

        if (last_.timestamp_us.isZero()) {
            last_ = raw;
        }
        return last_;
    }

    int reset_count{0};

private:
    std::vector<RobotState> queue_{};
    RobotState last_{};
};

class CountingTelemetryPublisher final : public telemetry::ITelemetryPublisher {
public:
    void publishGeometry(const HexapodGeometry&) override {}
    void publishControlStep(const telemetry::ControlStepTelemetry& sample) override {
        last_control_step = sample;
        ++control_step_count;
    }
    telemetry::TelemetryPublishCounters counters() const override {
        telemetry::TelemetryPublishCounters value{};
        value.packets_sent = control_step_count;
        return value;
    }

    int control_step_count{0};
    telemetry::ControlStepTelemetry last_control_step{};
};

RobotState makeEstimatorSample(uint64_t sample_id,
                               uint64_t timestamp_us,
                               bool hard_reset = false,
                               bool resync_requested = false,
                               double model_trust = 1.0,
                               double position_error = 0.0,
                               double orientation_error = 0.0,
                               double contact_mismatch = 0.0,
                               double terrain_residual = 0.0) {
    RobotState est{};
    est.sample_id = sample_id;
    est.timestamp_us = TimePointUs{timestamp_us};
    est.valid = true;
    est.has_valid_flag = true;
    est.has_body_twist_state = true;
    est.body_twist_state.body_trans_m = PositionM3{0.05, -0.02, 0.10};
    est.body_twist_state.body_trans_mps = VelocityMps3{0.01, -0.03, 0.02};
    est.body_twist_state.twist_pos_rad = EulerAnglesRad3{0.02, -0.01, 0.10};
    est.body_twist_state.twist_vel_radps = AngularVelocityRadPerSec3{0.03, -0.02, 0.01};

    for (std::size_t leg = 0; leg < est.foot_contact_fusion.size(); ++leg) {
        est.foot_contact_fusion[leg].phase =
            hard_reset ? ContactPhase::ConfirmedStance : ContactPhase::Search;
        est.foot_contact_fusion[leg].confidence = hard_reset ? 0.92f : 0.05f;
    }

    if (hard_reset || resync_requested || model_trust < 1.0 || position_error > 0.0 ||
        orientation_error > 0.0 || contact_mismatch > 0.0 || terrain_residual > 0.0) {
        est.has_fusion_diagnostics = true;
        est.fusion.model_trust = model_trust;
        est.fusion.resync_requested = resync_requested || hard_reset;
        est.fusion.hard_reset_requested = hard_reset;
        est.fusion.residuals.max_body_position_error_m = position_error;
        est.fusion.residuals.max_body_orientation_error_rad = orientation_error;
        est.fusion.residuals.contact_mismatch_ratio = contact_mismatch;
        est.fusion.residuals.terrain_residual_m = terrain_residual;
    }
    return est;
}

MotionIntent makeIntentSample(uint64_t sample_id, uint64_t timestamp_us) {
    MotionIntent intent{};
    intent.requested_mode = RobotMode::WALK;
    intent.sample_id = sample_id;
    intent.timestamp_us = TimePointUs{timestamp_us};
    intent.cmd_vx_mps = LinearRateMps{0.08};
    return intent;
}

} // namespace

int main() {
    UdpPhysicsSimStub stub{};
    if (!expect(stub.ok(), "stub socket should initialize")) {
        return EXIT_FAILURE;
    }
    stub.start();

    auto bridge = std::make_unique<PhysicsSimBridge>("127.0.0.1", stub.port(), 1000, 8, nullptr);
    auto estimator = std::make_unique<ScriptedEstimator>();
    auto* estimator_raw = estimator.get();

    control_config::ControlConfig cfg{};
    cfg.freshness.estimator.max_allowed_age_us = DurationUs{10'000'000};
    cfg.freshness.intent.max_allowed_age_us = DurationUs{10'000'000};
    cfg.telemetry.enabled = true;
    cfg.telemetry.publish_period = std::chrono::milliseconds{0};
    cfg.telemetry.geometry_refresh_period = std::chrono::milliseconds{0};
    auto telemetry = std::make_unique<CountingTelemetryPublisher>();
    auto* telemetry_raw = telemetry.get();

    RobotRuntime runtime(std::move(bridge), std::move(estimator), nullptr, cfg, std::move(telemetry));
    if (!expect(runtime.init(), "runtime init should succeed")) {
        return EXIT_FAILURE;
    }

    runtime.setMotionIntentForTest(makeIntentSample(1, 1'000'000));
    estimator_raw->enqueue(makeEstimatorSample(1, 1'100'000, false));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    if (!expect(stub.correctionCount() == 0, "steady-state step should not emit corrections") ||
        !expect(estimator_raw->reset_count == 1, "init should reset estimator once") ||
        !expect(!telemetry_raw->last_control_step.fusion.correction.has_data,
                "steady-state telemetry should not report a correction")) {
        return EXIT_FAILURE;
    }

    runtime.setMotionIntentForTest(makeIntentSample(2, 1'200'000));
    estimator_raw->enqueue(makeEstimatorSample(2,
                                              1'300'000,
                                              false,
                                              true,
                                              0.58,
                                              0.08,
                                              0.22,
                                              0.18,
                                              0.07));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    for (int i = 0; i < 200; ++i) {
        if (stub.correctionCount() > 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    if (!expect(stub.correctionCount() == 1, "strong correction should emit one correction") ||
        !expect(estimator_raw->reset_count == 1, "strong correction should not reset estimator")) {
        return EXIT_FAILURE;
    }

    const physics_sim::StateCorrection strong_correction = stub.lastCorrection();
    if (!expect((strong_correction.flags & physics_sim::kStateCorrectionHardReset) == 0,
                "strong correction should not be a hard reset") ||
        !expect((strong_correction.flags & physics_sim::kStateCorrectionRelocalize) != 0,
                "strong correction should request relocalization") ||
        !expect((strong_correction.flags & physics_sim::kStateCorrectionContactValid) != 0,
                "contact flag should be set") ||
        !expect(strong_correction.sequence_id == 2, "correction sample id should track estimator sample") ||
        !expect(nearlyEqual(strong_correction.correction_strength, 0.785, 0.05),
                "strong correction should be blended but not full strength") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.has_data,
                "telemetry should report the emitted correction") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.mode == telemetry::FusionCorrectionMode::Strong,
                "telemetry should expose strong correction mode") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.sample_id == 2,
                "telemetry should carry the correction sample id")) {
        return EXIT_FAILURE;
    }

    if (!expect(telemetry_raw->last_control_step.fusion.correction.residuals.max_body_position_error_m == 0.08,
                "telemetry should carry the correction residuals")) {
        return EXIT_FAILURE;
    }

    runtime.setMotionIntentForTest(makeIntentSample(3, 1'400'000));
    estimator_raw->enqueue(makeEstimatorSample(3,
                                              1'500'000,
                                              false,
                                              false,
                                              0.88,
                                              0.03,
                                              0.10,
                                              0.11,
                                              0.02));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    for (int i = 0; i < 200; ++i) {
        if (stub.correctionCount() > 1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    if (!expect(stub.correctionCount() == 2,
                "borderline correction should still emit after the strong sample") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.has_data,
                "telemetry should report the borderline correction") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.mode == telemetry::FusionCorrectionMode::Strong,
                "hysteresis should keep the borderline correction strong") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.sample_id == 3,
                "borderline telemetry should advance sample id")) {
        return EXIT_FAILURE;
    }

    runtime.setMotionIntentForTest(makeIntentSample(4, 1'600'000));
    estimator_raw->enqueue(makeEstimatorSample(4,
                                              1'700'000,
                                              true,
                                              true,
                                              0.12,
                                              0.20,
                                              0.44,
                                              0.31,
                                              0.22));
    runtime.busStep();
    runtime.estimatorStep();
    runtime.safetyStep();
    runtime.controlStep();

    for (int i = 0; i < 200; ++i) {
        if (stub.correctionCount() > 2) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }

    if (!expect(stub.correctionCount() == 3, "hard reset should emit a third correction") ||
        !expect(estimator_raw->reset_count == 2, "hard reset should reset estimator once more") ||
        !expect((stub.lastCorrection().flags & physics_sim::kStateCorrectionHardReset) != 0,
                "hard reset packet should carry hard reset flag") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.mode ==
                    telemetry::FusionCorrectionMode::HardReset,
                "telemetry should expose hard reset mode") ||
        !expect(telemetry_raw->last_control_step.fusion.correction.sample_id == 4,
                "hard reset telemetry should advance sample id")) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
