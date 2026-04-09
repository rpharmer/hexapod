#include <GLFW/glfw3.h>

#include <cmath>
#include <iostream>

namespace {
constexpr int kWindowWidth = 960;
constexpr int kWindowHeight = 640;
}

int main() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  GLFWwindow* window = glfwCreateWindow(kWindowWidth, kWindowHeight,
                                        "Hexapod OpenGL Visualiser", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  while (!glfwWindowShouldClose(window)) {
    const float t = static_cast<float>(glfwGetTime());
    const float pulse = (std::sin(t) + 1.0F) * 0.5F;

    glViewport(0, 0, kWindowWidth, kWindowHeight);
    glClearColor(0.05F, 0.08F + 0.15F * pulse, 0.12F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef(t * 20.0F, 0.0F, 0.0F, 1.0F);

    glBegin(GL_TRIANGLES);
    glColor3f(0.96F, 0.35F, 0.22F);
    glVertex2f(-0.6F, -0.4F);

    glColor3f(0.20F, 0.76F, 0.38F);
    glVertex2f(0.6F, -0.4F);

    glColor3f(0.23F, 0.45F, 0.95F);
    glVertex2f(0.0F, 0.6F);
    glEnd();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
