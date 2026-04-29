#pragma once

#include "visualiser/render/mat4.hpp"

namespace visualiser::render {

/// Matches legacy fixed-function chain in DrawScene:
/// glTranslatef(0, -0.35*scene_radius, -camera_distance);
/// glRotatef(pitch, 1,0,0);
/// glRotatef(yaw, 0,1,0);
/// glTranslatef(-(center.x+pan_x), -(center.y+pan_y), -center.z);
Mat4 LegacyViewMatrix(float scene_radius,
                       float camera_distance,
                       float pitch_deg,
                       float yaw_deg,
                       float center_x,
                       float center_y,
                       float center_z,
                       float pan_x,
                       float pan_y);

Mat4 ProjectionFromLegacyFrustum(int width, int height);

}  // namespace visualiser::render
