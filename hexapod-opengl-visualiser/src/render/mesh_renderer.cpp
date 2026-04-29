#include "visualiser/render/mesh_renderer.hpp"

#include "visualiser/render/mat4.hpp"

namespace visualiser::render {

bool MeshRenderer::Init(const char* vert_src, const char* frag_src) {
  Shutdown();
  program_ = visualiser::gl::ShaderProgram(vert_src, frag_src);
  if (!program_.Valid()) {
    return false;
  }
  loc_mvp_ = program_.UniformLocation("u_mvp");
  loc_nm_ = program_.UniformLocation("u_normal_matrix");
  loc_light_ = program_.UniformLocation("u_light_dir");
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  const GLsizei stride = 9 * sizeof(float);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glBindVertexArray(0);
  valid_ = true;
  return true;
}

void MeshRenderer::Shutdown() {
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

void MeshRenderer::Clear() { data_.clear(); }

void MeshRenderer::AddVertex(const Vec3& pos, const Vec3& normal, const Vec3& color) {
  data_.push_back(pos.x);
  data_.push_back(pos.y);
  data_.push_back(pos.z);
  data_.push_back(normal.x);
  data_.push_back(normal.y);
  data_.push_back(normal.z);
  data_.push_back(color.x);
  data_.push_back(color.y);
  data_.push_back(color.z);
}

void MeshRenderer::AddTriangle(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& n, const Vec3& color) {
  AddVertex(p0, n, color);
  AddVertex(p1, n, color);
  AddVertex(p2, n, color);
}

void MeshRenderer::FlushWorld(const Mat4& proj, const Mat4& view, const Vec3& light_dir_world) {
  if (!valid_ || data_.empty()) {
    return;
  }
  const Mat4 vp = Mat4::Mul(proj, view);
  const Mat3 rv = Mat3::FromMat4Upper(view.m);
  const Mat3 normal_matrix = Mat3::Transpose(Mat3::Inverse(rv));
  const Vec3 light_view = TransformDirection(view, light_dir_world);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data_.size() * sizeof(float)), data_.data(), GL_STREAM_DRAW);
  glUseProgram(program_.Id());
  glUniformMatrix4fv(loc_mvp_, 1, GL_FALSE, vp.m);
  glUniformMatrix3fv(loc_nm_, 1, GL_FALSE, normal_matrix.m);
  glUniform3f(loc_light_, light_view.x, light_view.y, light_view.z);
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(data_.size() / 9));
  glBindVertexArray(0);
  glUseProgram(0);
}

}  // namespace visualiser::render
