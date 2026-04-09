#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#ifndef _WIN32
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

constexpr int kDefaultWindowWidth = 1280;
constexpr int kDefaultWindowHeight = 720;
constexpr int kDefaultUdpPort = 9870;
constexpr float kPi = 3.14159265358979323846f;

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quat {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

enum class ShapeType {
  kUnknown,
  kSphere,
  kBox,
  kPlane,
  kCapsule,
};

struct EntityState {
  std::uint32_t id = 0;
  ShapeType shape = ShapeType::kUnknown;
  float radius = 0.5f;
  float half_height = 0.5f;
  Vec3 half_extents{0.5f, 0.5f, 0.5f};
  Vec3 plane_normal{0.0f, 1.0f, 0.0f};
  float plane_offset = 0.0f;
  Vec3 position{};
  Quat rotation{};
  bool has_static = false;
  bool has_frame = false;
};

struct Options {
  int udp_port = kDefaultUdpPort;
};

float Clamp(float value, float lo, float hi) {
  return std::max(lo, std::min(value, hi));
}

Vec3 Normalize(const Vec3& value) {
  const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  if (length <= 1e-6f) {
    return {0.0f, 1.0f, 0.0f};
  }
  return {value.x / length, value.y / length, value.z / length};
}

Vec3 Cross(const Vec3& a, const Vec3& b) {
  return {
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

void ApplyQuaternion(const Quat& quat) {
  const Quat q = [&]() {
    const float length = std::sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    if (length <= 1e-6f) {
      return Quat{};
    }
    return Quat{quat.w / length, quat.x / length, quat.y / length, quat.z / length};
  }();

  const float angle = 2.0f * std::acos(Clamp(q.w, -1.0f, 1.0f));
  const float sin_half = std::sqrt(std::max(0.0f, 1.0f - q.w * q.w));
  if (sin_half <= 1e-5f) {
    return;
  }

  const float axis_x = q.x / sin_half;
  const float axis_y = q.y / sin_half;
  const float axis_z = q.z / sin_half;
  glRotatef(angle * 180.0f / kPi, axis_x, axis_y, axis_z);
}

std::optional<std::string> ExtractStringField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":\"";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find('"', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  return std::string(payload.substr(value_start, value_end - value_start));
}

std::optional<std::uint32_t> ExtractUintField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(token.c_str(), &end_ptr, 10);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(parsed);
}

std::optional<float> ExtractFloatField(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find_first_of(",}", value_start);
  const std::string token(payload.substr(value_start, value_end - value_start));
  char* end_ptr = nullptr;
  errno = 0;
  const float parsed = std::strtof(token.c_str(), &end_ptr);
  if (end_ptr == token.c_str() || errno != 0) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<std::array<float, 3>> ExtractFloat3Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 3> out{};
  if (!(in >> out[0] >> out[1] >> out[2])) {
    return std::nullopt;
  }
  return out;
}

std::optional<std::array<float, 4>> ExtractFloat4Field(std::string_view payload, std::string_view key) {
  const std::string needle = "\"" + std::string(key) + "\":[";
  const std::size_t start = payload.find(needle);
  if (start == std::string_view::npos) {
    return std::nullopt;
  }

  const std::size_t value_start = start + needle.size();
  const std::size_t value_end = payload.find(']', value_start);
  if (value_end == std::string_view::npos) {
    return std::nullopt;
  }

  std::string values(payload.substr(value_start, value_end - value_start));
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream in(values);
  std::array<float, 4> out{};
  if (!(in >> out[0] >> out[1] >> out[2] >> out[3])) {
    return std::nullopt;
  }
  return out;
}

ShapeType ParseShapeType(const std::optional<std::string>& shape_name) {
  if (!shape_name.has_value()) {
    return ShapeType::kUnknown;
  }
  if (*shape_name == "sphere") {
    return ShapeType::kSphere;
  }
  if (*shape_name == "box") {
    return ShapeType::kBox;
  }
  if (*shape_name == "plane") {
    return ShapeType::kPlane;
  }
  if (*shape_name == "capsule") {
    return ShapeType::kCapsule;
  }
  return ShapeType::kUnknown;
}

void DrawWireCube(const Vec3& half_extents) {
  const std::array<Vec3, 8> vertices = {{
      {-half_extents.x, -half_extents.y, -half_extents.z},
      {half_extents.x, -half_extents.y, -half_extents.z},
      {half_extents.x, half_extents.y, -half_extents.z},
      {-half_extents.x, half_extents.y, -half_extents.z},
      {-half_extents.x, -half_extents.y, half_extents.z},
      {half_extents.x, -half_extents.y, half_extents.z},
      {half_extents.x, half_extents.y, half_extents.z},
      {-half_extents.x, half_extents.y, half_extents.z},
  }};

  const std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
  }};

  glBegin(GL_LINES);
  for (const auto& edge : edges) {
    const Vec3& a = vertices[edge[0]];
    const Vec3& b = vertices[edge[1]];
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
  }
  glEnd();
}

void DrawWireSphere(float radius, int slices = 18, int stacks = 10) {
  for (int stack = 1; stack < stacks; ++stack) {
    const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
    glBegin(GL_LINE_LOOP);
    for (int slice = 0; slice < slices; ++slice) {
      const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
      const float x = radius * std::cos(phi) * std::cos(theta);
      const float y = radius * std::sin(phi);
      const float z = radius * std::cos(phi) * std::sin(theta);
      glVertex3f(x, y, z);
    }
    glEnd();
  }

  for (int slice = 0; slice < slices; ++slice) {
    const float theta = 2.0f * kPi * static_cast<float>(slice) / static_cast<float>(slices);
    glBegin(GL_LINE_STRIP);
    for (int stack = 0; stack <= stacks; ++stack) {
      const float phi = -0.5f * kPi + kPi * static_cast<float>(stack) / static_cast<float>(stacks);
      const float x = radius * std::cos(phi) * std::cos(theta);
      const float y = radius * std::sin(phi);
      const float z = radius * std::cos(phi) * std::sin(theta);
      glVertex3f(x, y, z);
    }
    glEnd();
  }
}

void DrawWireCapsule(float radius, float half_height) {
  glBegin(GL_LINES);
  for (int i = 0; i < 4; ++i) {
    const float angle = static_cast<float>(i) * 0.5f * kPi;
    const float x = std::cos(angle) * radius;
    const float z = std::sin(angle) * radius;
    glVertex3f(x, -half_height, z);
    glVertex3f(x, half_height, z);
  }
  glEnd();

  glPushMatrix();
  glTranslatef(0.0f, half_height, 0.0f);
  DrawWireSphere(radius, 18, 6);
  glPopMatrix();

  glPushMatrix();
  glTranslatef(0.0f, -half_height, 0.0f);
  DrawWireSphere(radius, 18, 6);
  glPopMatrix();
}

void DrawPlane(const Vec3& normal, float offset) {
  const Vec3 up = Normalize(normal);
  const Vec3 tangent_seed = std::fabs(up.y) > 0.9f ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 1.0f, 0.0f};
  const Vec3 tangent = Normalize(Cross(tangent_seed, up));
  const Vec3 bitangent = Normalize(Cross(up, tangent));
  const Vec3 center{up.x * offset, up.y * offset, up.z * offset};
  constexpr float scale = 12.0f;

  const Vec3 corners[4] = {
      {center.x + (tangent.x + bitangent.x) * scale, center.y + (tangent.y + bitangent.y) * scale,
       center.z + (tangent.z + bitangent.z) * scale},
      {center.x + (tangent.x - bitangent.x) * scale, center.y + (tangent.y - bitangent.y) * scale,
       center.z + (tangent.z - bitangent.z) * scale},
      {center.x + (-tangent.x - bitangent.x) * scale, center.y + (-tangent.y - bitangent.y) * scale,
       center.z + (-tangent.z - bitangent.z) * scale},
      {center.x + (-tangent.x + bitangent.x) * scale, center.y + (-tangent.y + bitangent.y) * scale,
       center.z + (-tangent.z + bitangent.z) * scale},
  };

  glColor4f(0.14f, 0.18f, 0.22f, 1.0f);
  glBegin(GL_QUADS);
  for (const Vec3& corner : corners) {
    glVertex3f(corner.x, corner.y, corner.z);
  }
  glEnd();

  glColor3f(0.28f, 0.34f, 0.40f);
  glBegin(GL_LINES);
  for (int i = -6; i <= 6; ++i) {
    const float d = static_cast<float>(i) * 2.0f;
    const Vec3 a{center.x + tangent.x * scale + bitangent.x * d, center.y + tangent.y * scale + bitangent.y * d,
                 center.z + tangent.z * scale + bitangent.z * d};
    const Vec3 b{center.x - tangent.x * scale + bitangent.x * d, center.y - tangent.y * scale + bitangent.y * d,
                 center.z - tangent.z * scale + bitangent.z * d};
    const Vec3 c{center.x + bitangent.x * scale + tangent.x * d, center.y + bitangent.y * scale + tangent.y * d,
                 center.z + bitangent.z * scale + tangent.z * d};
    const Vec3 d2{center.x - bitangent.x * scale + tangent.x * d, center.y - bitangent.y * scale + tangent.y * d,
                  center.z - bitangent.z * scale + tangent.z * d};
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
    glVertex3f(c.x, c.y, c.z);
    glVertex3f(d2.x, d2.y, d2.z);
  }
  glEnd();
}

bool ParsePacket(const std::string& payload, std::map<std::uint32_t, EntityState>& entities) {
  const auto message_type = ExtractStringField(payload, "message_type");
  if (!message_type.has_value()) {
    return false;
  }

  const auto entity_id = ExtractUintField(payload, "entity_id");
  if (!entity_id.has_value()) {
    return false;
  }

  EntityState& entity = entities[*entity_id];
  entity.id = *entity_id;

  if (*message_type == "entity_static") {
    entity.shape = ParseShapeType(ExtractStringField(payload, "shape_type"));
    if (const auto radius = ExtractFloatField(payload, "radius")) {
      entity.radius = *radius;
    }
    if (const auto half_height = ExtractFloatField(payload, "half_height")) {
      entity.half_height = *half_height;
    }
    if (const auto half_extents = ExtractFloat3Field(payload, "half_extents")) {
      entity.half_extents = {(*half_extents)[0], (*half_extents)[1], (*half_extents)[2]};
    }
    if (const auto plane_normal = ExtractFloat3Field(payload, "plane_normal")) {
      entity.plane_normal = {(*plane_normal)[0], (*plane_normal)[1], (*plane_normal)[2]};
    }
    if (const auto plane_offset = ExtractFloatField(payload, "plane_offset")) {
      entity.plane_offset = *plane_offset;
    }
    entity.has_static = true;
    return true;
  }

  if (*message_type == "entity_frame") {
    if (const auto position = ExtractFloat3Field(payload, "position")) {
      entity.position = {(*position)[0], (*position)[1], (*position)[2]};
    }
    if (const auto rotation = ExtractFloat4Field(payload, "rotation")) {
      entity.rotation = {(*rotation)[0], (*rotation)[1], (*rotation)[2], (*rotation)[3]};
    }
    entity.has_frame = true;
    return true;
  }

  return false;
}

#ifndef _WIN32
class UdpReceiver {
 public:
  explicit UdpReceiver(int port) {
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      std::cerr << "Failed to create UDP socket on port " << port << ": " << std::strerror(errno) << "\n";
      return;
    }

    const int reuse = 1;
    (void)::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<std::uint16_t>(port));
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
      std::cerr << "Failed to bind UDP socket on port " << port << ": " << std::strerror(errno) << "\n";
      ::close(socket_fd_);
      socket_fd_ = -1;
      return;
    }

    const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
      (void)::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    valid_ = true;
  }

  ~UdpReceiver() {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
    }
  }

  bool valid() const { return valid_; }

  int Pump(std::map<std::uint32_t, EntityState>& entities) {
    if (!valid_) {
      return 0;
    }

    int packets = 0;
    for (;;) {
      std::array<char, 4096> buffer{};
      const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size() - 1, 0, nullptr, nullptr);
      if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        std::cerr << "UDP receive error: " << std::strerror(errno) << "\n";
        break;
      }
      if (bytes == 0) {
        break;
      }

      buffer[static_cast<std::size_t>(bytes)] = '\0';
      if (ParsePacket(buffer.data(), entities)) {
        ++packets;
      }
    }

    return packets;
  }

 private:
  int socket_fd_ = -1;
  bool valid_ = false;
};
#endif

Options ParseArgs(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--udp-port") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --udp-port\n";
        std::exit(1);
      }
      options.udp_port = std::atoi(argv[++i]);
      if (options.udp_port <= 0 || options.udp_port > 65535) {
        std::cerr << "Invalid UDP port: " << options.udp_port << "\n";
        std::exit(1);
      }
      continue;
    }

    if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: hexapod-opengl-visualiser [--udp-port <port>]\n";
      std::exit(0);
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    std::exit(1);
  }

  return options;
}

void ConfigureProjection(int width, int height) {
  const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  constexpr float near_plane = 0.1f;
  constexpr float far_plane = 100.0f;
  constexpr float fov_y_degrees = 50.0f;
  const float top = std::tan(fov_y_degrees * 0.5f * kPi / 180.0f) * near_plane;
  const float right = top * aspect;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-right, right, -top, top, near_plane, far_plane);
}

void DrawScene(const std::map<std::uint32_t, EntityState>& entities, float time_s) {
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0f, -0.6f, -8.0f);
  glRotatef(18.0f, 1.0f, 0.0f, 0.0f);
  glRotatef(28.0f + time_s * 8.0f, 0.0f, 1.0f, 0.0f);

  for (const auto& [id, entity] : entities) {
    (void)id;
    if (!entity.has_static) {
      continue;
    }

    if (entity.shape == ShapeType::kPlane) {
      DrawPlane(entity.plane_normal, entity.plane_offset);
      continue;
    }

    if (!entity.has_frame) {
      continue;
    }

    glPushMatrix();
    glTranslatef(entity.position.x, entity.position.y, entity.position.z);
    ApplyQuaternion(entity.rotation);

    switch (entity.shape) {
      case ShapeType::kBox:
        glColor3f(0.92f, 0.43f, 0.21f);
        DrawWireCube(entity.half_extents);
        break;
      case ShapeType::kSphere:
        glColor3f(0.22f, 0.75f, 0.91f);
        DrawWireSphere(entity.radius);
        break;
      case ShapeType::kCapsule:
        glColor3f(0.37f, 0.82f, 0.51f);
        DrawWireCapsule(entity.radius, entity.half_height);
        break;
      case ShapeType::kUnknown:
      case ShapeType::kPlane:
        break;
    }

    glPopMatrix();
  }
}

}  // namespace

int main(int argc, char** argv) {
  const Options options = ParseArgs(argc, argv);

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

  GLFWwindow* window = glfwCreateWindow(
      kDefaultWindowWidth, kDefaultWindowHeight, "Hexapod OpenGL Visualiser", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glLineWidth(2.0f);

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  glfwDestroyWindow(window);
  glfwTerminate();
  return 1;
#endif

  std::map<std::uint32_t, EntityState> entities;
  double last_title_update_s = -1.0;

  while (!glfwWindowShouldClose(window)) {
    receiver.Pump(entities);

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    ConfigureProjection(framebuffer_width, framebuffer_height);

    const float time_s = static_cast<float>(glfwGetTime());
    DrawScene(entities, time_s);

    if (last_title_update_s < 0.0 || glfwGetTime() - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | UDP " << options.udp_port << " | entities " << entities.size();
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = glfwGetTime();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
