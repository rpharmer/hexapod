#include "visualiser/gl/shader.hpp"

#include <iostream>
#include <vector>

namespace visualiser::gl {

namespace {

GLuint Compile(GLenum type, const char* src) {
  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    GLint len = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
    std::vector<char> log(static_cast<std::size_t>(std::max(len, 1)));
    glGetShaderInfoLog(shader, len, nullptr, log.data());
    std::cerr << "Shader compile failed:\n" << log.data() << "\n";
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

}  // namespace

ShaderProgram::ShaderProgram(const char* vert_src, const char* frag_src) {
  const GLuint vs = Compile(GL_VERTEX_SHADER, vert_src);
  const GLuint fs = Compile(GL_FRAGMENT_SHADER, frag_src);
  if (vs == 0 || fs == 0) {
    if (vs) glDeleteShader(vs);
    if (fs) glDeleteShader(fs);
    return;
  }
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint linked = 0;
  glGetProgramiv(program_, GL_LINK_STATUS, &linked);
  if (!linked) {
    GLint len = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);
    std::vector<char> log(static_cast<std::size_t>(std::max(len, 1)));
    glGetProgramInfoLog(program_, len, nullptr, log.data());
    std::cerr << "Program link failed:\n" << log.data() << "\n";
    glDeleteProgram(program_);
    program_ = 0;
  }
}

ShaderProgram::~ShaderProgram() { Destroy(); }

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept : program_(other.program_) {
  other.program_ = 0;
}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
  if (this != &other) {
    Destroy();
    program_ = other.program_;
    other.program_ = 0;
  }
  return *this;
}

void ShaderProgram::Destroy() {
  if (program_ != 0) {
    glDeleteProgram(program_);
    program_ = 0;
  }
}

GLint ShaderProgram::UniformLocation(const char* name) const {
  return glGetUniformLocation(program_, name);
}

}  // namespace visualiser::gl
