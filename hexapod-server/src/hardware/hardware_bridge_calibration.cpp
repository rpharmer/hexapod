#include "hardware_bridge.hpp"

#include "bridge_command_api.hpp"
#include "logger.hpp"
#include "protocol_codec.hpp"

namespace {

std::shared_ptr<logging::AsyncLogger> resolveLogger(
    const std::shared_ptr<logging::AsyncLogger>& logger) {
    if (logger) {
        return logger;
    }
    return logging::GetDefaultLogger();
}

}  // namespace

bool SimpleHardwareBridge::set_angle_calibrations(const std::vector<float>& calibs) {
    return complete_command("set_angle_calibrations",
                            send_calibrations_result(calibs),
                            "calibration command failed");
}

bool SimpleHardwareBridge::get_angle_calibrations(std::vector<float>& out_calibs) {
    protocol::Calibrations calibrations{};
    const auto decode_calibrations = [](const std::vector<uint8_t>& payload, protocol::Calibrations& decoded) {
        return protocol::decode_calibrations(payload, decoded);
    };
    const BridgeError command_error = withCommandApi(
        "get_angle_calibrations",
        [&calibrations, &decode_calibrations](BridgeCommandApi& api) {
            return api.request_decoded_with_error(
                CommandCode::GET_ANGLE_CALIBRATIONS, {}, decode_calibrations, calibrations);
        });
    if (!complete_command("get_angle_calibrations", command_error, "command execution failed")) {
        return false;
    }

    out_calibs.assign(calibrations.begin(), calibrations.end());
    return true;
}

BridgeError SimpleHardwareBridge::send_calibrations_result(const std::vector<float>& calibs) {
    if (calibs.size() != (kProtocolJointCount * kProtocolCalibrationPairsPerJoint)) {
        return BridgeError::ProtocolFailure;
    }

    const BridgeError command_error = withCommandApi(
        "set_angle_calibrations",
        [&calibs](BridgeCommandApi& api) {
            protocol::Calibrations payload{};
            for (std::size_t i = 0; i < calibs.size(); ++i) {
                payload[i] = calibs[i];
            }
            return api.request_ack_with_error(
                CommandCode::SET_ANGLE_CALIBRATIONS, protocol::encode_calibrations(payload));
        });

    if (command_error != BridgeError::None) {
        if (auto logger = resolveLogger(logger_)) {
            LOG_ERROR(logger, "calibration packet failed");
        }
        return command_error;
    }

    if (auto logger = resolveLogger(logger_)) {
        LOG_INFO(logger, "calibration packet accepted");
    }
    return BridgeError::None;
}
