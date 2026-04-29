#pragma once

namespace visualiser::math::frame_math {

constexpr float kSceneYawOffsetRad = 1.57079632679489661923f;

inline float ServerYawToSceneYaw(float server_yaw_rad) {
  return server_yaw_rad - kSceneYawOffsetRad;
}

}  // namespace visualiser::math::frame_math

namespace visualiser_frame_math {

constexpr float kSceneYawOffsetRad = visualiser::math::frame_math::kSceneYawOffsetRad;

inline float ServerYawToSceneYaw(float server_yaw_rad) {
  return visualiser::math::frame_math::ServerYawToSceneYaw(server_yaw_rad);
}

}  // namespace visualiser_frame_math
