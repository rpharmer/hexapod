#pragma once

#include "visualiser/render/line_renderer.hpp"
#include "visualiser/render/mat4.hpp"
#include "visualiser/render/mesh_renderer.hpp"
namespace visualiser::render {

void AppendWireBox(LineRenderer& lr, const Mat4& model, const Vec3& half_extents, const Vec3& color);
void AppendWireSphere(LineRenderer& lr, const Mat4& model, float radius, const Vec3& color);
void AppendWireCapsule(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color);
void AppendWireCylinder(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color);
void AppendWireHalfCylinder(LineRenderer& lr, const Mat4& model, float radius, float half_height, const Vec3& color);
void AppendPlane(MeshRenderer& mesh,
                 LineRenderer& lines,
                 const Mat4& model,
                 const Vec3& plane_normal,
                 float plane_offset,
                 const Vec3& fill_color,
                 const Vec3& grid_color);

}  // namespace visualiser::render
