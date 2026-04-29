#pragma once

#include <glad/glad.h>

#include <cstddef>
#include <vector>

#include "visualiser/gl/shader.hpp"
#include "visualiser/render/mat4.hpp"

namespace visualiser::render {

/// Batched GL_LINES with interleaved [px,py,pz, cr,cg,cb].
class LineRenderer {
 public:
  LineRenderer() = default;
  bool Init(const char* vert_src, const char* frag_src);
  void Shutdown();

  void Clear();
  void AddSegment(const Vec3& a, const Vec3& b, const Vec3& color);
  void Flush(const Mat4& mvp);

  bool valid() const { return valid_; }

 private:
  bool valid_ = false;
  visualiser::gl::ShaderProgram program_{};
  GLint loc_mvp_ = -1;
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  std::vector<float> data_;
};

}  // namespace visualiser::render
