#pragma once

#include <glad/glad.h>

#include <vector>

#include "visualiser/gl/shader.hpp"
#include "visualiser/render/mat4.hpp"

namespace visualiser::render {

/// GL_POINTS with gl_PointSize in vertex shader.
class PointRenderer {
 public:
  PointRenderer() = default;
  bool Init(const char* vert_src, const char* frag_src);
  void Shutdown();

  void Clear();
  void AddPoint(const Vec3& p, const Vec3& color);
  void Flush(const Mat4& mvp, float point_size);

  bool valid() const { return valid_; }

 private:
  bool valid_ = false;
  visualiser::gl::ShaderProgram program_{};
  GLint loc_mvp_ = -1;
  GLint loc_ps_ = -1;
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  std::vector<float> data_;
};

}  // namespace visualiser::render
