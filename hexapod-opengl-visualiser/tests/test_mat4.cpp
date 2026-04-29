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

bool nearly(float a, float b, float e = 1e-4f) {
  return std::fabs(a - b) <= e;
}

}  // namespace

int main() {
  using namespace visualiser::render;
  bool ok = true;

  const Mat4 a = Mat4::Translate(1.0f, 2.0f, 3.0f);
  const Vec3 p = a.TransformPoint({0.0f, 0.0f, 0.0f});
  ok = ok && expect(nearly(p.x, 1.0f) && nearly(p.y, 2.0f) && nearly(p.z, 3.0f), "translate point");

  const Mat4 r = Mat4::RotateY(kPi * 0.5f);
  const Vec3 q = r.TransformPoint({1.0f, 0.0f, 0.0f});
  ok = ok && expect(nearly(q.x, 0.0f) && nearly(q.y, 0.0f) && nearly(q.z, -1.0f), "rotate Y 90");

  const Mat4 id = Mat4::Identity();
  const Mat3 m3 = Mat3::FromMat4Upper(id.m);
  const Mat3 inv = Mat3::Inverse(m3);
  ok = ok && expect(nearly(inv.m[0], 1.0f) && nearly(inv.m[4], 1.0f) && nearly(inv.m[8], 1.0f), "mat3 inverse identity");

  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
