#pragma once

#include "visualiser/math/vec3.hpp"
#include "visualiser/render/mat4.hpp"

namespace visualiser::scene {

// GLFW cursor positions are in window coordinates, not framebuffer pixels.
// The inverse matrix already contains the framebuffer aspect ratio; only the
// cursor's logical window size belongs in the NDC conversion.
bool PickHorizontalGround(const visualiser::render::Mat4& inverse_view_projection,
                          float cursor_x,
                          float cursor_y,
                          int window_width,
                          int window_height,
                          float ground_height,
                          visualiser::math::Vec3& hit);

}  // namespace visualiser::scene
