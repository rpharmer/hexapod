#include "visualiser/render/line_renderer.hpp"

namespace visualiser::render {

bool LineRenderer::Init(const char* vert_src, const char* frag_src) {
  Shutdown();
  program_ = visualiser::gl::ShaderProgram(vert_src, frag_src);
  if (!program_.Valid()) {
    return false;
  }
  loc_mvp_ = program_.UniformLocation("u_mvp");
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

void LineRenderer::Shutdown() {
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

void LineRenderer::Clear() { data_.clear(); }

void LineRenderer::AddSegment(const Vec3& a, const Vec3& b, const Vec3& color) {
  data_.push_back(a.x);
  data_.push_back(a.y);
  data_.push_back(a.z);
  data_.push_back(color.x);
  data_.push_back(color.y);
  data_.push_back(color.z);
  data_.push_back(b.x);
  data_.push_back(b.y);
  data_.push_back(b.z);
  data_.push_back(color.x);
  data_.push_back(color.y);
  data_.push_back(color.z);
}

void LineRenderer::Flush(const Mat4& mvp) {
  if (!valid_ || data_.empty()) {
    return;
  }
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data_.size() * sizeof(float)), data_.data(), GL_STREAM_DRAW);
  glUseProgram(program_.Id());
  glUniformMatrix4fv(loc_mvp_, 1, GL_FALSE, mvp.m);
  glBindVertexArray(vao_);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(data_.size() / 6));
  glBindVertexArray(0);
  glUseProgram(0);
}

}  // namespace visualiser::render
