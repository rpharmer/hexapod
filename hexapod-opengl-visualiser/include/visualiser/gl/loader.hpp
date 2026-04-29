#pragma once

struct GLFWwindow;

namespace visualiser::gl {

/// Call after glfwMakeContextCurrent. Returns false on failure.
bool InitGlad();

}  // namespace visualiser::gl
