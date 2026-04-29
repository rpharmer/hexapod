#pragma once

namespace visualiser::ui {

struct AppUiState {
  bool show_scene = true;
  bool show_robot = true;
  bool show_terrain = true;
  bool follow_active = true;
  bool rotate_scene = false;
  bool show_overlay = true;
  bool show_debug = false;
  int source_mode = 0;
};

struct CameraState {
  float yaw_deg = 28.0f;
  float pitch_deg = 18.0f;
  float distance_scale = 6.0f;
  float pan_x = 0.0f;
  float pan_y = 0.0f;
  float spin_deg_per_s = 8.0f;
};

}  // namespace visualiser::ui
