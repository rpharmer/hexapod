#include "visualiser/ui/overlay.hpp"

#include <imgui.h>

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
            const std::string& last_rejection_reason) {
  if (!ui.show_overlay) {
    return;
  }
  ImGui::SetNextWindowBgAlpha(0.90f);
  ImGui::Begin("Hexapod Control Room", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Text("Source: %s", source_label.c_str());
  ImGui::Text("Packets: %llu accepted, %llu rejected",
              static_cast<unsigned long long>(packets_received),
              static_cast<unsigned long long>(packets_rejected));
  ImGui::Text("Scene entities: %zu", entity_count);
  ImGui::Text("Terrain: %s", terrain_available ? "available" : "none");
  if (!last_rejection_reason.empty()) {
    ImGui::Text("Last rejection: %s", last_rejection_reason.c_str());
  }
  (void)telemetry;
  (void)last_packet_age_s;
  if (ImGui::Button("Reset View")) {
    camera = CameraState{};
  }
  ImGui::End();
}

}  // namespace visualiser::ui
