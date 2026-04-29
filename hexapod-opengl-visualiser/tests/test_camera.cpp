#include "visualiser/render/camera.hpp"
#include "visualiser/render/mat4.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool expect(bool ok, const char* msg) {
  if (!ok) {
    std::cerr << "FAIL: " << msg << '\n';
  }
  return ok;
}

bool nearly(float a, float b, float e = 1e-3f) {
  return std::fabs(a - b) <= e;
}

}  // namespace

int main() {
  using namespace visualiser::render;
  const Mat4 v = LegacyViewMatrix(2.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  const Vec3 o = v.TransformPoint({0.0f, 0.0f, 0.0f});
  bool ok = expect(std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z), "view finite");

  const Mat4 p = ProjectionFromLegacyFrustum(1280, 720);
  const Vec3 c = p.TransformPoint({0.0f, 0.0f, -0.2f});
  ok = ok && expect(std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z), "proj finite");

  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
