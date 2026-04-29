#!/usr/bin/env python3
"""Apply GL3 DrawScene + RunApplication migration to legacy_main.cpp."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / "src/app/legacy_main.cpp"
text = path.read_text()

old_block = r"""void ConfigureProjection(int width, int height) {
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

void DrawScene(const std::map<std::uint32_t, EntityState>& entities,
               const TerrainPatchState& terrain_patch,
               const HexapodTelemetryState& telemetry,
               const AppUiState& ui,
               const CameraState& camera,
               float time_s) {
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SceneBounds bounds;
  if (ui.show_scene) {
    bounds = ComputeSceneBounds(entities);
  }
  if (ui.show_terrain) {
    ExpandTerrainPatchBounds(bounds, terrain_patch);
  }
  if (ui.show_robot && telemetry.has_geometry && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    const SceneBounds robot_bounds = ComputeRobotBounds(robot_geometry, telemetry.angles_deg, telemetry.body_pose);
    if (robot_bounds.valid) {
      if (!bounds.valid) {
        bounds = robot_bounds;
      } else {
        ExpandBounds(bounds, robot_bounds.min);
        ExpandBounds(bounds, robot_bounds.max);
      }
    }
  }

  const Vec3 center = ui.follow_active && bounds.valid
      ? Vec3{
            0.5f * (bounds.min.x + bounds.max.x),
            0.5f * (bounds.min.y + bounds.max.y),
            0.5f * (bounds.min.z + bounds.max.z),
        }
      : Vec3{};
  const Vec3 diagonal = bounds.valid
      ? Vec3{
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
        }
      : Vec3{1.0f, 1.0f, 1.0f};
  const float scene_radius =
      std::max(0.25f, 0.5f * std::sqrt(Dot(diagonal, diagonal)));
  const float camera_distance = std::max(2.0f, scene_radius * camera.distance_scale);
  const float yaw = camera.yaw_deg + (ui.rotate_scene ? time_s * camera.spin_deg_per_s : 0.0f);
  glTranslatef(0.0f, -0.35f * scene_radius, -camera_distance);
  glRotatef(camera.pitch_deg, 1.0f, 0.0f, 0.0f);
  glRotatef(yaw, 0.0f, 1.0f, 0.0f);
  glTranslatef(-(center.x + camera.pan_x), -(center.y + camera.pan_y), -center.z);

  if (ui.show_terrain) {
    DrawTerrainPatch(terrain_patch);
  }

  if (ui.show_scene) {
    for (const auto& [id, entity] : entities) {
      (void)id;
      if (!entity.has_static) {
        continue;
      }

      if (entity.shape == ShapeType::kPlane) {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset);
        continue;
      }

      if (!entity.has_frame) {
        continue;
      }

      glPushMatrix();
      glTranslatef(entity.position.x, entity.position.y, entity.position.z);
      ApplyQuaternion(entity.rotation);

      if (entity.shape == ShapeType::kCompound) {
        if (entity.compound_children.empty()) {
          DrawPrimitiveShape(
              ShapeType::kBox,
              entity.radius,
              entity.half_height,
              entity.half_extents,
              entity.plane_normal,
              entity.plane_offset);
        } else {
          for (const CompoundChildState& child : entity.compound_children) {
            glPushMatrix();
            glTranslatef(child.local_position.x, child.local_position.y, child.local_position.z);
            ApplyQuaternion(child.local_rotation);
            DrawPrimitiveShape(
                child.shape,
                child.radius,
                child.half_height,
                child.half_extents,
                entity.plane_normal,
                entity.plane_offset);
            glPopMatrix();
          }
        }
      } else {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset);
      }

      glPopMatrix();
    }
  }

  if (ui.show_robot && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    DrawHexapodModel(robot_geometry, telemetry.angles_deg, telemetry.status, telemetry.body_pose);
  }
}
"""

new_block = r"""void DrawScene(const std::map<std::uint32_t, EntityState>& entities,
               const TerrainPatchState& terrain_patch,
               const HexapodTelemetryState& telemetry,
               const AppUiState& ui,
               const CameraState& camera,
               float time_s,
               int viewport_width,
               int viewport_height) {
  using RM = visualiser::render::Mat4;
  using RV = visualiser::render::Vec3;

  g_line_renderer.Clear();
  g_mesh_renderer.Clear();
  g_point_renderer.Clear();

  glViewport(0, 0, viewport_width, viewport_height);
  glClearColor(0.04f, 0.06f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  SceneBounds bounds;
  if (ui.show_scene) {
    bounds = ComputeSceneBounds(entities);
  }
  if (ui.show_terrain) {
    ExpandTerrainPatchBounds(bounds, terrain_patch);
  }
  if (ui.show_robot && telemetry.has_geometry && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    const SceneBounds robot_bounds = ComputeRobotBounds(robot_geometry, telemetry.angles_deg, telemetry.body_pose);
    if (robot_bounds.valid) {
      if (!bounds.valid) {
        bounds = robot_bounds;
      } else {
        ExpandBounds(bounds, robot_bounds.min);
        ExpandBounds(bounds, robot_bounds.max);
      }
    }
  }

  const Vec3 center = ui.follow_active && bounds.valid
      ? Vec3{
            0.5f * (bounds.min.x + bounds.max.x),
            0.5f * (bounds.min.y + bounds.max.y),
            0.5f * (bounds.min.z + bounds.max.z),
        }
      : Vec3{};
  const Vec3 diagonal = bounds.valid
      ? Vec3{
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
        }
      : Vec3{1.0f, 1.0f, 1.0f};
  const float scene_radius =
      std::max(0.25f, 0.5f * std::sqrt(Dot(diagonal, diagonal)));
  const float camera_distance = std::max(2.0f, scene_radius * camera.distance_scale);
  const float yaw = camera.yaw_deg + (ui.rotate_scene ? time_s * camera.spin_deg_per_s : 0.0f);

  const RM proj = visualiser::render::ProjectionFromLegacyFrustum(viewport_width, viewport_height);
  const RM view = visualiser::render::LegacyViewMatrix(scene_radius,
                                                       camera_distance,
                                                       camera.pitch_deg,
                                                       yaw,
                                                       center.x,
                                                       center.y,
                                                       center.z,
                                                       camera.pan_x,
                                                       camera.pan_y);
  const RM vp = RM::Mul(proj, view);

  if (ui.show_terrain) {
    DrawTerrainPatch(terrain_patch);
  }

  if (ui.show_scene) {
    for (const auto& [id, entity] : entities) {
      (void)id;
      if (!entity.has_static) {
        continue;
      }

      if (entity.shape == ShapeType::kPlane) {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset,
            RM::Identity());
        continue;
      }

      if (!entity.has_frame) {
        continue;
      }

      RM model = RM::Mul(RM::Translate(entity.position.x, entity.position.y, entity.position.z),
                         LegacyMatFromQuat(entity.rotation));

      if (entity.shape == ShapeType::kCompound) {
        if (entity.compound_children.empty()) {
          DrawPrimitiveShape(
              ShapeType::kBox,
              entity.radius,
              entity.half_height,
              entity.half_extents,
              entity.plane_normal,
              entity.plane_offset,
              model);
        } else {
          for (const CompoundChildState& child : entity.compound_children) {
            const RM child_model =
                RM::Mul(model,
                        RM::Mul(RM::Translate(child.local_position.x, child.local_position.y, child.local_position.z),
                                LegacyMatFromQuat(child.local_rotation)));
            DrawPrimitiveShape(
                child.shape,
                child.radius,
                child.half_height,
                child.half_extents,
                entity.plane_normal,
                entity.plane_offset,
                child_model);
          }
        }
      } else {
        DrawPrimitiveShape(
            entity.shape,
            entity.radius,
            entity.half_height,
            entity.half_extents,
            entity.plane_normal,
            entity.plane_offset,
            model);
      }
    }
  }

  if (ui.show_robot && telemetry.has_joints) {
    const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
    DrawHexapodModel(robot_geometry, telemetry.angles_deg, telemetry.status, telemetry.body_pose);
  }

  const RV light_dir = visualiser::render::Normalize(RV{0.35f, 1.0f, 0.25f});
  g_mesh_renderer.FlushWorld(proj, view, light_dir);
  g_line_renderer.Flush(vp);
  g_point_renderer.Flush(vp, 6.0f);
}
"""

if old_block not in text:
    raise SystemExit("old DrawScene block not found")

text = text.replace(old_block, new_block, 1)

old_main = r"""int VisualiserLegacyMain(int argc, char** argv) {
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

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGuiIO& imgui_io = ImGui::GetIO();
  imgui_io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 1;
#endif

  std::map<std::uint32_t, EntityState> entities;
  TerrainPatchState terrain_patch;
  HexapodTelemetryState telemetry;
  AppUiState ui;
  CameraState camera;
  uint64_t accepted_packets = 0;
  uint64_t rejected_packets = 0;
  std::string last_packet_kind = "waiting";
  double last_packet_time_s = std::numeric_limits<double>::quiet_NaN();
  double last_title_update_s = -1.0;
  double last_joint_log_s = -1.0;
  bool overlay_toggle_down = false;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    const bool overlay_toggle_now = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
    if (overlay_toggle_now && !overlay_toggle_down) {
      ui.show_overlay = !ui.show_overlay;
    }
    overlay_toggle_down = overlay_toggle_now;

    const int accepted_this_frame =
        receiver.Pump(entities, terrain_patch, telemetry, accepted_packets, rejected_packets, last_packet_kind);
    if (accepted_this_frame > 0) {
      last_packet_time_s = glfwGetTime();
    }

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
    ConfigureProjection(framebuffer_width, framebuffer_height);

    const float time_s = static_cast<float>(glfwGetTime());
    DrawScene(entities, terrain_patch, telemetry, ui, camera, time_s);
    const double now_s = glfwGetTime();
    if (options.log_joint_positions
        && telemetry.has_joints
        && (last_joint_log_s < 0.0 || now_s - last_joint_log_s >= 1.0)) {
      const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
      LogJointPositions(robot_geometry, telemetry.angles_deg, telemetry.body_pose, now_s);
      last_joint_log_s = now_s;
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const double packet_age_s = std::isfinite(last_packet_time_s) ? now_s - last_packet_time_s
                                                                  : std::numeric_limits<double>::quiet_NaN();
    DrawUi(ui,
           camera,
           telemetry,
           last_packet_kind,
           accepted_packets,
           rejected_packets,
           packet_age_s,
           entities.size(),
           terrain_patch.valid);
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    if (last_title_update_s < 0.0 || now_s - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | " << last_packet_kind << " | UDP " << options.udp_port
            << " | entities " << entities.size() << " | packets " << accepted_packets;
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = now_s;
    }

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
"""

new_main = r"""namespace visualiser::app {

int RunApplication(int argc, char** argv) {
  const Options options = ParseArgs(argc, argv);

  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return 1;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow* window = glfwCreateWindow(
      kDefaultWindowWidth, kDefaultWindowHeight, "Hexapod OpenGL Visualiser", nullptr, nullptr);
  if (window == nullptr) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (!visualiser::gl::InitGlad()) {
    std::cerr << "Failed to load OpenGL entry points (GLAD)\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glLineWidth(1.0f);

  if (!InitModernRenderer()) {
    std::cerr << "Failed to initialize shader-based renderers\n";
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGuiIO& imgui_io = ImGui::GetIO();
  imgui_io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

#ifndef _WIN32
  UdpReceiver receiver(options.udp_port);
  if (!receiver.valid()) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }
#else
  std::cerr << "UDP receiver is not implemented on Windows in this build\n";
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 1;
#endif

  std::map<std::uint32_t, EntityState> entities;
  TerrainPatchState terrain_patch;
  HexapodTelemetryState telemetry;
  AppUiState ui;
  CameraState camera;
  uint64_t accepted_packets = 0;
  uint64_t rejected_packets = 0;
  std::string last_packet_kind = "waiting";
  double last_packet_time_s = std::numeric_limits<double>::quiet_NaN();
  double last_title_update_s = -1.0;
  double last_joint_log_s = -1.0;
  bool overlay_toggle_down = false;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    const bool overlay_toggle_now = glfwGetKey(window, GLFW_KEY_F1) == GLFW_PRESS;
    if (overlay_toggle_now && !overlay_toggle_down) {
      ui.show_overlay = !ui.show_overlay;
    }
    overlay_toggle_down = overlay_toggle_now;

    const int accepted_this_frame =
        receiver.Pump(entities, terrain_patch, telemetry, accepted_packets, rejected_packets, last_packet_kind);
    if (accepted_this_frame > 0) {
      last_packet_time_s = glfwGetTime();
    }

    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

    const float time_s = static_cast<float>(glfwGetTime());
    DrawScene(entities, terrain_patch, telemetry, ui, camera, time_s, framebuffer_width, framebuffer_height);
    const double now_s = glfwGetTime();
    if (options.log_joint_positions
        && telemetry.has_joints
        && (last_joint_log_s < 0.0 || now_s - last_joint_log_s >= 1.0)) {
      const HexapodGeometryState robot_geometry = telemetry.geometry.valid ? telemetry.geometry : MakeDefaultGeometryState();
      LogJointPositions(robot_geometry, telemetry.angles_deg, telemetry.body_pose, now_s);
      last_joint_log_s = now_s;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    const double packet_age_s = std::isfinite(last_packet_time_s) ? now_s - last_packet_time_s
                                                                  : std::numeric_limits<double>::quiet_NaN();
    DrawUi(ui,
           camera,
           telemetry,
           last_packet_kind,
           accepted_packets,
           rejected_packets,
           packet_age_s,
           entities.size(),
           terrain_patch.valid);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (last_title_update_s < 0.0 || now_s - last_title_update_s > 0.25) {
      std::ostringstream title;
      title << "Hexapod OpenGL Visualiser | " << last_packet_kind << " | UDP " << options.udp_port
            << " | entities " << entities.size() << " | packets " << accepted_packets;
      glfwSetWindowTitle(window, title.str().c_str());
      last_title_update_s = now_s;
    }

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

}  // namespace visualiser::app
"""

if old_main not in text:
    raise SystemExit("old VisualiserLegacyMain block not found")

text = text.replace(old_main, new_main, 1)

path.write_text(text)
print("OK", path)
