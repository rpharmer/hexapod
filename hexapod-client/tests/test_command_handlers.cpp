#include "hexapod-client.hpp"
#include "firmware_context.hpp"
#include "protocol_codec.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool condition, const char* message)
{
  if(!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    return false;
  }
  return true;
}

const SerialCommsClient::SentPacket* last_packet(const FirmwareContext& ctx)
{
  if(ctx.serial.sent_packets.empty())
    return nullptr;
  return &ctx.serial.sent_packets.back();
}

bool expect_last_packet(const FirmwareContext& ctx, uint16_t seq, uint8_t cmd)
{
  const auto* packet = last_packet(ctx);
  return expect(packet != nullptr, "expected packet") &&
         expect(packet->seq == seq, "expected sequence") &&
         expect(packet->cmd == cmd, "expected ACK/NACK command");
}

bool test_set_joint_targets_valid_ack_and_updates_positions()
{
  FirmwareContext ctx{};
  protocol::JointTargets targets{};
  for(std::size_t i = 0; i < targets.size(); ++i)
    targets[i] = static_cast<float>(i) * 0.1f;

  handleSetJointTargetsCommand(ctx, 11, protocol::encode_joint_targets(targets));

  if(!expect_last_packet(ctx, 11, ACK))
    return false;

  for(std::size_t i = 0; i < targets.size(); ++i)
  {
    if(!expect(std::fabs(ctx.jointTargetPositionsRad[i] - targets[i]) < 1e-6f,
               "expected target position update"))
      return false;
    if(!expect(std::fabs(ctx.servos.value(static_cast<int>(i)) - targets[i]) < 1e-6f,
               "expected servo target update"))
      return false;
  }

  return true;
}

bool test_handshake_enables_software_angle_feedback_when_requested()
{
  FirmwareContext ctx{};
  const protocol::HelloRequest request{PROTOCOL_VERSION, CAPABILITY_ANGULAR_FEEDBACK};

  const bool ok = handleHandshake(ctx, 9, protocol::encode_hello_request(request));
  if(!expect(ok, "expected handshake success"))
    return false;
  if(!expect_last_packet(ctx, 9, ACK))
    return false;

  return expect(ctx.softwareAngleFeedbackEstimatorEnabled,
                "expected software angle feedback estimator to be enabled");
}

bool test_get_full_hardware_state_uses_software_angle_feedback_fallback()
{
  FirmwareContext ctx{};
  ctx.softwareAngleFeedbackEstimatorEnabled = true;
  for(std::size_t i = 0; i < kProtocolJointCount; ++i)
  {
    ctx.jointTargetPositionsRad[i] = 0.2f * static_cast<float>(i);
    ctx.servos.value(static_cast<int>(i), -0.25f);
  }

  handleGetFullHardwareStateCommand(ctx, 24);

  if(!expect_last_packet(ctx, 24, ACK))
    return false;

  protocol::FullHardwareState decoded{};
  const auto* packet = last_packet(ctx);
  if(!expect(protocol::decode_full_hardware_state(packet->payload, decoded),
             "expected full hardware state payload"))
    return false;

  for(std::size_t i = 0; i < kProtocolJointCount; ++i)
  {
    if(!expect(std::fabs(decoded.joint_positions_rad[i] - ctx.jointTargetPositionsRad[i]) < 1e-6f,
               "expected fallback joint position from estimator target"))
      return false;
  }

  return true;
}

bool test_set_joint_targets_invalid_payload_length_nack()
{
  FirmwareContext ctx{};
  std::vector<uint8_t> bad_payload(kProtocolJointTargetsPayloadBytes - 1, 0);
  handleSetJointTargetsCommand(ctx, 12, bad_payload);

  if(!expect_last_packet(ctx, 12, NACK))
    return false;

  const auto* packet = last_packet(ctx);
  return expect(packet->payload.size() == 1 && packet->payload[0] == INVALID_PAYLOAD_LENGTH,
                "expected INVALID_PAYLOAD_LENGTH nack");
}

bool test_set_angle_calibrations_valid_ack_and_updates_calibration()
{
  FirmwareContext ctx{};
  protocol::Calibrations calibrations{};
  for(std::size_t i = 0; i < calibrations.size(); ++i)
    calibrations[i] = 1000.0f + static_cast<float>(i);

  handleCalibCommand(ctx, 13, protocol::encode_calibrations(calibrations));

  if(!expect_last_packet(ctx, 13, ACK))
    return false;

  for(std::size_t i = 0; i < kProtocolJointCount; ++i)
  {
    Calibration& cal = ctx.servos.calibration(static_cast<int>(i));
    if(!expect(std::fabs(cal.first_pulse() - calibrations[i * 2]) < 1e-6f,
               "expected first calibration pulse update"))
      return false;
    if(!expect(std::fabs(cal.last_pulse() - calibrations[i * 2 + 1]) < 1e-6f,
               "expected last calibration pulse update"))
      return false;
  }

  return true;
}

bool test_set_angle_calibrations_malformed_buffer_nack()
{
  FirmwareContext ctx{};
  std::vector<uint8_t> malformed(kProtocolCalibrationsPayloadBytes + 1, 0xFF);
  handleCalibCommand(ctx, 14, malformed);

  if(!expect_last_packet(ctx, 14, NACK))
    return false;

  const auto* packet = last_packet(ctx);
  return expect(packet->payload.size() == 1 && packet->payload[0] == INVALID_PAYLOAD_LENGTH,
                "expected INVALID_PAYLOAD_LENGTH nack for malformed calibration payload");
}

bool test_get_sensor_valid_payload_ack()
{
  FirmwareContext ctx{};
  ctx.sen_adc.voltage = 2.75f;

  handleGetSensorCommand(ctx, 21, {3});

  if(!expect_last_packet(ctx, 21, ACK))
    return false;

  const auto* packet = last_packet(ctx);
  protocol::ScalarFloat decoded{};
  return expect(protocol::decode_scalar_float(packet->payload, decoded), "expected scalar payload") &&
         expect(std::fabs(decoded.value - 2.75f) < 1e-6f, "expected sensor voltage in payload") &&
         expect(ctx.mux.selected == (servo2040::SENSOR_1_ADDR + 3), "expected sensor mux select");
}

bool test_get_sensor_invalid_payload_and_out_of_range()
{
  FirmwareContext ctx{};
  handleGetSensorCommand(ctx, 22, {});
  if(!expect_last_packet(ctx, 22, NACK))
    return false;
  if(!expect(last_packet(ctx)->payload[0] == INVALID_PAYLOAD_LENGTH, "expected invalid length nack"))
    return false;

  handleGetSensorCommand(ctx, 23, {kProtocolFootSensorCount});
  if(!expect_last_packet(ctx, 23, NACK))
    return false;

  return expect(last_packet(ctx)->payload[0] == OUT_OF_RANGE_INDEX, "expected out of range nack");
}

bool test_set_power_relay_ack_and_nacks()
{
  FirmwareContext ctx{};

  handleSetPowerRelayCommand(ctx, 31, {1});
  if(!expect_last_packet(ctx, 31, ACK))
    return false;

  handleSetPowerRelayCommand(ctx, 32, {2});
  if(!expect_last_packet(ctx, 32, NACK))
    return false;
  if(!expect(last_packet(ctx)->payload[0] == INVALID_ARGUMENT, "expected invalid argument nack"))
    return false;

  handleSetPowerRelayCommand(ctx, 33, {});
  if(!expect_last_packet(ctx, 33, NACK))
    return false;
  return expect(last_packet(ctx)->payload[0] == INVALID_PAYLOAD_LENGTH,
                "expected invalid length nack for malformed relay payload");
}

bool test_set_servos_enabled_valid_and_invalid_payloads()
{
  FirmwareContext ctx{};
  protocol::ServoEnabled enabled{};
  for(std::size_t i = 0; i < enabled.size(); ++i)
    enabled[i] = (i % 2 == 0) ? 1 : 0;

  handleSetServosEnabledCommand(ctx, 41, protocol::encode_servo_enabled(enabled));
  if(!expect_last_packet(ctx, 41, ACK))
    return false;

  for(std::size_t i = 0; i < enabled.size(); ++i)
  {
    if(!expect(ctx.servos.is_enabled(static_cast<int>(i)) == (enabled[i] != 0),
               "expected servo enable state update"))
      return false;
  }

  handleSetServosEnabledCommand(ctx, 42, std::vector<uint8_t>{1, 2, 3});
  if(!expect_last_packet(ctx, 42, NACK))
    return false;

  return expect(last_packet(ctx)->payload[0] == INVALID_PAYLOAD_LENGTH,
                "expected invalid length nack for malformed servo enable payload");
}

} // namespace

int main()
{
  if(!test_handshake_enables_software_angle_feedback_when_requested())
    return EXIT_FAILURE;
  if(!test_get_full_hardware_state_uses_software_angle_feedback_fallback())
    return EXIT_FAILURE;
  if(!test_set_joint_targets_valid_ack_and_updates_positions())
    return EXIT_FAILURE;
  if(!test_set_joint_targets_invalid_payload_length_nack())
    return EXIT_FAILURE;
  if(!test_set_angle_calibrations_valid_ack_and_updates_calibration())
    return EXIT_FAILURE;
  if(!test_set_angle_calibrations_malformed_buffer_nack())
    return EXIT_FAILURE;
  if(!test_get_sensor_valid_payload_ack())
    return EXIT_FAILURE;
  if(!test_get_sensor_invalid_payload_and_out_of_range())
    return EXIT_FAILURE;
  if(!test_set_power_relay_ack_and_nacks())
    return EXIT_FAILURE;
  if(!test_set_servos_enabled_valid_and_invalid_payloads())
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
