#include "visualiser/gl/loader.hpp"

#include <glad/glad.h>

#include <GLFW/glfw3.h>

namespace visualiser::gl {

bool InitGlad() {
  return gladLoadGLLoader(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress)) != 0;
}

}  // namespace visualiser::gl
