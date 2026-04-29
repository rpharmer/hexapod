#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "visualiser/math/vec3.hpp"

namespace visualiser::robot {

struct HexapodLegLayout {
  std::string key;
  visualiser::math::Vec3 body_coxa_offset{0.0f, 0.0f, 0.0f};
  float mount_angle_rad = 0.0f;
  float coxa_mm = 35.0f;
  float femur_mm = 70.0f;
  float tibia_mm = 110.0f;
  float coxa_attach_deg = 0.0f;
  float femur_attach_deg = 0.0f;
  float tibia_attach_deg = 0.0f;
  float coxa_sign = 1.0f;
  float femur_sign = 1.0f;
  float tibia_sign = 1.0f;
};

struct HexapodGeometryState {
  bool valid = false;
  float coxa_mm = 35.0f;
  float femur_mm = 70.0f;
  float tibia_mm = 110.0f;
  float body_radius_mm = 60.0f;
  std::array<HexapodLegLayout, 6> legs{};
};

struct HexapodStatusState {
  bool valid = false;
  std::uint64_t timestamp_ms = 0;
  int loop_counter = 0;
  int active_mode = 0;
  int active_fault = 0;
  bool bus_ok = true;
  bool estimator_valid = true;
  float voltage = 0.0f;
  float current = 0.0f;
  std::optional<int> nav_lifecycle{};
  std::optional<int> nav_block_reason{};
  std::optional<int> nav_planner_status{};
  std::optional<bool> nav_map_fresh{};
  std::optional<std::size_t> nav_replan_count{};
  std::optional<double> nav_active_segment_length_m{};
  std::optional<std::size_t> nav_active_segment_waypoint_count{};
  std::optional<double> nav_nearest_obstacle_distance_m{};
  std::optional<double> fusion_model_trust{};
  std::optional<bool> fusion_resync_requested{};
  std::optional<bool> fusion_hard_reset_requested{};
  std::optional<bool> fusion_predictive_mode{};
  std::optional<double> fusion_max_body_position_error_m{};
  std::optional<double> fusion_max_body_orientation_error_rad{};
  std::optional<double> fusion_contact_mismatch_ratio{};
  std::optional<double> fusion_terrain_residual_m{};
};

struct HexapodBodyPoseState {
  bool valid = false;
  visualiser::math::Vec3 position{};
  float yaw_rad = 0.0f;
};

struct HexapodTelemetryState {
  bool has_geometry = false;
  bool has_joints = false;
  HexapodGeometryState geometry{};
  HexapodStatusState status{};
  HexapodBodyPoseState body_pose{};
  std::array<std::array<float, 3>, 6> angles_deg{};
};

}  // namespace visualiser::robot
