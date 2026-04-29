#include "visualiser/gl/debug.hpp"

#include <glad/glad.h>

#include <iostream>
#include <string>

#ifndef NDEBUG

namespace {

void APIENTRY DebugCallback(GLenum source,
                            GLenum type,
                            GLuint id,
                            GLenum severity,
                            GLsizei length,
                            const GLchar* message,
                            const void* userParam) {
  (void)source;
  (void)type;
  (void)id;
  (void)severity;
  (void)length;
  (void)userParam;
  if (message == nullptr) {
    std::cerr << "[GL]\n";
    return;
  }
  if (length > 0) {
    std::cerr << "[GL] " << std::string(message, static_cast<std::size_t>(length)) << '\n';
  } else {
    std::cerr << "[GL] " << message << '\n';
  }
}

}  // namespace

#endif

namespace visualiser::gl {

void SetupDebugCallback() {
#ifndef NDEBUG
  if (GLAD_GL_KHR_debug) {
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(DebugCallback, nullptr);
  }
#else
  (void)0;
#endif
}

}  // namespace visualiser::gl
