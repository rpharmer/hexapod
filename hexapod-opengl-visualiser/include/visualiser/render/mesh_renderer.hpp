#pragma once

#include <glad/glad.h>

#include <vector>

#include "visualiser/gl/shader.hpp"
#include "visualiser/render/mat4.hpp"

namespace visualiser::render {

/// Interleaved: pos(3) normal(3) color(3) = 9 floats per vertex, GL_TRIANGLES.
class MeshRenderer {
 public:
  MeshRenderer() = default;
  bool Init(const char* vert_src, const char* frag_src);
  void Shutdown();

  void Clear();
  void AddVertex(const Vec3& pos, const Vec3& normal, const Vec3& color);
  void AddTriangle(const Vec3& p0,
                   const Vec3& p1,
                   const Vec3& p2,
                   const Vec3& n,
                   const Vec3& color);
  /// World-space light direction; converted to view space in Flush.
  /// Vertices/normals are in world space.
  void FlushWorld(const Mat4& proj, const Mat4& view, const Vec3& light_dir_world);

  bool valid() const { return valid_; }

 private:
  bool valid_ = false;
  visualiser::gl::ShaderProgram program_{};
  GLint loc_mvp_ = -1;
  GLint loc_nm_ = -1;
  GLint loc_light_ = -1;
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  std::vector<float> data_;
};

}  // namespace visualiser::render
