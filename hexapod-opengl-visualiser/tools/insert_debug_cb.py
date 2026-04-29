from pathlib import Path

p = Path(__file__).resolve().parents[1] / "src/app/legacy_main.cpp"
t = p.read_text()
old = r"""  if (!visualiser::gl::InitGlad()) {
    std::cerr << "Failed to load OpenGL entry points (GLAD)\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  glEnable(GL_DEPTH_TEST);"""
new = r"""  if (!visualiser::gl::InitGlad()) {
    std::cerr << "Failed to load OpenGL entry points (GLAD)\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  visualiser::gl::SetupDebugCallback();

  glEnable(GL_DEPTH_TEST);"""
if old not in t:
    raise SystemExit("pattern not found")
p.write_text(t.replace(old, new, 1))
print("ok")
