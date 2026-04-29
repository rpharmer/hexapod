#pragma once

namespace visualiser_frame_math {

constexpr float kSceneYawOffsetRad = 1.57079632679489661923f;

inline float ServerYawToSceneYaw(float server_yaw_rad) {
  return server_yaw_rad - kSceneYawOffsetRad;
}

}  // namespace visualiser_frame_math
