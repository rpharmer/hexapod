#include "hardware_bridge_transport_test_helpers.hpp"

#include <cstdlib>

int main() {
    if (!hardware_bridge_transport_test::run_handshake_tests()) {
        return EXIT_FAILURE;
    }
    if (!hardware_bridge_transport_test::run_failure_and_corruption_tests()) {
        return EXIT_FAILURE;
    }
    if (!hardware_bridge_transport_test::run_command_tests()) {
        return EXIT_FAILURE;
    }
    if (!hardware_bridge_transport_test::run_capability_tests()) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
