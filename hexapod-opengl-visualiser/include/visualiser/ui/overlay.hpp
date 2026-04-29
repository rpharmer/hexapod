#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "visualiser/robot/geometry_state.hpp"
#include "visualiser/ui/overlay_state.hpp"

namespace visualiser::ui {

void DrawUi(AppUiState& ui,
            CameraState& camera,
            const visualiser::robot::HexapodTelemetryState& telemetry,
            const std::string& source_label,
            std::uint64_t packets_received,
            std::uint64_t packets_rejected,
            double last_packet_age_s,
            std::size_t entity_count,
            bool terrain_available,
            const std::string& last_rejection_reason);

}  // namespace visualiser::ui
