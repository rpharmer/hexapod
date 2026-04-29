#pragma once

#include <glad/glad.h>

#include <string>

namespace visualiser::gl {

class ShaderProgram {
 public:
  ShaderProgram() = default;
  ShaderProgram(const char* vert_src, const char* frag_src);
  ~ShaderProgram();

  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator=(const ShaderProgram&) = delete;
  ShaderProgram(ShaderProgram&& other) noexcept;
  ShaderProgram& operator=(ShaderProgram&& other) noexcept;

  void Destroy();
  bool Valid() const { return program_ != 0; }
  GLuint Id() const { return program_; }

  GLint UniformLocation(const char* name) const;

 private:
  GLuint program_ = 0;
};

}  // namespace visualiser::gl
