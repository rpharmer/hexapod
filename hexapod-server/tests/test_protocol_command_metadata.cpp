#include "hexapod-common.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

bool test_known_commands_have_metadata_entries()
{
    static_assert(command_metadata_is_complete(),
                  "metadata table should remain complete for all typed command enums");

    for (CommandCode cmd : kAllCommandCodes) {
        const CommandMetadata* metadata = find_command_metadata(cmd);
        if (!expect(metadata != nullptr, "expected metadata for known command")) {
            return false;
        }
        if (!expect(metadata->canonical_name != nullptr && metadata->canonical_name[0] != '\0',
                    "expected non-empty canonical command name")) {
            return false;
        }
    }

    return true;
}

bool test_metadata_payload_contracts_match_protocol_constants()
{
    const CommandMetadata* set_joint_targets = find_command_metadata(CommandCode::SET_JOINT_TARGETS);
    const CommandMetadata* set_calibrations = find_command_metadata(CommandCode::SET_ANGLE_CALIBRATIONS);
    const CommandMetadata* set_servos_enabled = find_command_metadata(CommandCode::SET_SERVOS_ENABLED);
    const CommandMetadata* set_led_colors = find_command_metadata(CommandCode::SET_LED_COLORS);

    return expect(set_joint_targets != nullptr &&
                      set_joint_targets->payload_bytes == kProtocolJointTargetsPayloadBytes &&
                      set_joint_targets->exact_payload_size,
                  "SET_JOINT_TARGETS contract mismatch") &&
           expect(set_calibrations != nullptr &&
                      set_calibrations->payload_bytes == kProtocolCalibrationsPayloadBytes &&
                      set_calibrations->exact_payload_size,
                  "SET_ANGLE_CALIBRATIONS contract mismatch") &&
           expect(set_servos_enabled != nullptr &&
                      set_servos_enabled->payload_bytes == kProtocolServoEnablePayloadBytes &&
                      set_servos_enabled->exact_payload_size,
                  "SET_SERVOS_ENABLED contract mismatch") &&
           expect(set_led_colors != nullptr &&
                      set_led_colors->payload_bytes == kProtocolLedColorsPayloadBytes &&
                      set_led_colors->exact_payload_size,
                  "SET_LED_COLORS contract mismatch");
}

bool test_unknown_command_maps_to_unknown_name()
{
    CommandCode parsed = CommandCode::HELLO;
    return expect(!try_parse_command_code(0xFF, parsed), "unknown command should fail typed parse") &&
           expect(std::string(command_name(0xFF)) == "UNKNOWN",
                  "expected unknown command name fallback");
}

bool test_firmware_dispatch_coverage_flags()
{
    std::size_t dispatch_handled_count = 0;
    for (const CommandMetadata& metadata : kCommandMetadata) {
        if (metadata.handled_by_firmware_dispatch) {
            ++dispatch_handled_count;
        }
    }

    const CommandMetadata* hello = find_command_metadata(CommandCode::HELLO);
    const CommandMetadata* heartbeat = find_command_metadata(CommandCode::HEARTBEAT);
    const CommandMetadata* diagnostic = find_command_metadata(CommandCode::DIAGNOSTIC);

    return expect(dispatch_handled_count == 14, "expected firmware dispatch metadata count to match route table") &&
           expect(hello != nullptr && !hello->handled_by_firmware_dispatch,
                  "HELLO should be handled by handshake flow, not generic dispatch") &&
           expect(heartbeat != nullptr && !heartbeat->handled_by_firmware_dispatch,
                  "HEARTBEAT should be handled by heartbeat flow, not generic dispatch") &&
           expect(diagnostic != nullptr && !diagnostic->handled_by_firmware_dispatch,
                  "DIAGNOSTIC should not be marked as firmware-dispatch handled");
}

} // namespace

int main()
{
    if (!test_known_commands_have_metadata_entries()) {
        return EXIT_FAILURE;
    }
    if (!test_metadata_payload_contracts_match_protocol_constants()) {
        return EXIT_FAILURE;
    }
    if (!test_unknown_command_maps_to_unknown_name()) {
        return EXIT_FAILURE;
    }
    if (!test_firmware_dispatch_coverage_flags()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
