#include "visualiser/render/point_renderer.hpp"

namespace visualiser::render {

bool PointRenderer::Init(const char* vert_src, const char* frag_src) {
  Shutdown();
  program_ = visualiser::gl::ShaderProgram(vert_src, frag_src);
  if (!program_.Valid()) {
    return false;
  }
  loc_mvp_ = program_.UniformLocation("u_mvp");
  loc_ps_ = program_.UniformLocation("u_point_size");
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);
  valid_ = true;
  return true;
}

void PointRenderer::Shutdown() {
  if (vbo_) {
    glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
  }
  if (vao_) {
    glDeleteVertexArrays(1, &vao_);
    vao_ = 0;
  }
  program_.Destroy();
  valid_ = false;
}

void PointRenderer::Clear() { data_.clear(); }

void PointRenderer::AddPoint(const Vec3& p, const Vec3& color) {
  data_.push_back(p.x);
  data_.push_back(p.y);
  data_.push_back(p.z);
  data_.push_back(color.x);
  data_.push_back(color.y);
  data_.push_back(color.z);
}

void PointRenderer::Flush(const Mat4& mvp, float point_size) {
  if (!valid_ || data_.empty()) {
    return;
  }
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data_.size() * sizeof(float)), data_.data(), GL_STREAM_DRAW);
  glUseProgram(program_.Id());
  glUniformMatrix4fv(loc_mvp_, 1, GL_FALSE, mvp.m);
  glUniform1f(loc_ps_, point_size);
  glBindVertexArray(vao_);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(data_.size() / 6));
  glBindVertexArray(0);
  glUseProgram(0);
}

}  // namespace visualiser::render
