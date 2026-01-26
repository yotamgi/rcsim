#include "raylib_engine.h"

#if defined(PLATFORM_DESKTOP)
#define GLSL_VERSION 330
#else // PLATFORM_ANDROID, PLATFORM_WEB
#define GLSL_VERSION 100
#endif

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <iostream>
#include <string>

namespace engine {

////////////////////////////////////////////////////////////////////////////////
// Model implementation
////////////////////////////////////////////////////////////////////////////////

mat4 Model::get_world_transform() const {
  if (m_parent) {
    return (raylib::Matrix)m_local_transform *
           (raylib::Matrix)m_parent->get_world_transform();
  } else {
    return m_local_transform;
  }
}

void Model::draw() {
  m_model.SetTransform(get_world_transform());
  DrawModel(m_model, (Vector3){0, 0, 0}, 1.0f, WHITE);
}

////////////////////////////////////////////////////////////////////////////////
// RaylibDevice implementation
////////////////////////////////////////////////////////////////////////////////

RaylibDevice::RaylibDevice(int screen_width, int screen_height,
                           std::string title)
    : m_camera(raylib::Vector3(2, 4, 6),
               raylib::Vector3(0, 0.5, 0.0)) // Initialize a default camera.
{

  SetConfigFlags(FLAG_MSAA_4X_HINT); // Enable Multi Sampling Anti Aliasing 4x
                                     // (if available)
  InitWindow(screen_width, screen_height,
             title.c_str()); // Create window and OpenGL context

  m_lighting_shader = raylib::Shader(
      TextFormat("resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
      TextFormat("resources/shaders/glsl%i/lighting.fs", GLSL_VERSION));

  m_camera.fovy = 45.0;
  m_camera.projection = CAMERA_PERSPECTIVE;

  // Get some required shader locations
  m_lighting_shader.locs[SHADER_LOC_VECTOR_VIEW] =
      m_lighting_shader.GetLocation("viewPos");

  // Ambient light level (some basic lighting)
  int ambientLoc = m_lighting_shader.GetLocation("ambient");
  std::array<float, 4> ambientValues = {0.3f, 0.3f, 0.3f, 0.3f};
  m_lighting_shader.SetValue(ambientLoc, ambientValues.data(),
                             SHADER_UNIFORM_VEC4);

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
}

Light &RaylibDevice::create_light(int type, raylib::Vector3 position,
                                  raylib::Vector3 target, raylib::Color color) {
  m_lights.push_back(
      CreateLight(type, position, target, color, m_lighting_shader));
  return m_lights.back();
}

std::shared_ptr<Model>
RaylibDevice::create_empty(std::shared_ptr<Model> parent) {
  return std::shared_ptr<Model>(new Model(parent));
}

std::shared_ptr<Model> RaylibDevice::load_model(const std::string &file_name,
                                                std::shared_ptr<Model> parent,
                                                bool enable_lighting) {
  std::shared_ptr<Model> model =
      std::shared_ptr<Model>(new Model(file_name, parent));
  if (enable_lighting) {
    for (int i = 0; i < model->m_model.GetMaterialCount(); i++) {
      model->m_model.GetMaterials()[i].shader = m_lighting_shader;
    }
  }
  m_models.push_back(model);
  return model;
}

std::shared_ptr<Model>
RaylibDevice::create_sphere(float radius, int rings, int slices,
                            std::shared_ptr<Model> parent,
                            bool enable_lighting) {
  std::shared_ptr<::Mesh> mesh =
      std::make_shared<::Mesh>(::GenMeshSphere(radius, rings, slices));
  std::shared_ptr<Model> model = std::shared_ptr<Model>(new Model(mesh, parent));
  if (enable_lighting) {
    for (int i = 0; i < model->m_model.GetMaterialCount(); i++) {
      model->m_model.GetMaterials()[i].shader = m_lighting_shader;
    }
  }
  m_models.push_back(model);
  return model;
}

std::shared_ptr<Model> RaylibDevice::create_cube(float width, float height,
                                                 float length,
                                                 std::shared_ptr<Model> parent,
                                                 bool enable_lighting) {
  std::shared_ptr<::Mesh> mesh =
      std::make_shared<::Mesh>(::GenMeshCube(width, height, length));
  std::shared_ptr<Model> model = std::shared_ptr<Model>(new Model(mesh, parent));
  if (enable_lighting) {
    for (int i = 0; i < model->m_model.GetMaterialCount(); i++) {
      model->m_model.GetMaterials()[i].shader = m_lighting_shader;
    }
  }
  m_models.push_back(model);
  return model;
}

void RaylibDevice::add_skybox_from_single_image(std::string image_path) {
  Image img = LoadImage(image_path.c_str());
  add_skybox_from_image(img);
}

void RaylibDevice::add_skybox_from_6_images(std::string right, std::string left,
                                            std::string top, std::string bottom,
                                            std::string back,
                                            std::string front) {
  std::array<raylib::Image, 6> faces = {
      LoadImage(right.c_str()), LoadImage(left.c_str()),
      LoadImage(top.c_str()),   LoadImage(bottom.c_str()),
      LoadImage(back.c_str()),  LoadImage(front.c_str()),
  };
  int size = faces[0].width; // Assuming square images of the same size

  Image atlas = GenImageColor(size, size * 6, BLANK);

  for (int i = 0; i < 6; i++) {
    Rectangle src = {0, 0, (float)size, (float)size};
    Rectangle dst = {0, (float)i * size, (float)size, (float)size};
    ImageDraw(&atlas, faces[i], src, dst, WHITE);
  }
  add_skybox_from_image(atlas);
}

void RaylibDevice::add_skybox_from_image(const raylib::Image &image) {
  // Load skybox model
  Mesh cube = GenMeshCube(1.0f, 1.0f, 1.0f);
  m_skybox_model = LoadModelFromMesh(cube);

  // Load skybox shader and set required locations
  // NOTE: Some locations are automatically set at shader loading
  m_skybox_model->materials[0].shader = LoadShader(
      TextFormat("resources/shaders/glsl%i/skybox.vs", GLSL_VERSION),
      TextFormat("resources/shaders/glsl%i/skybox.fs", GLSL_VERSION));

  SetShaderValue(
      m_skybox_model->materials[0].shader,
      GetShaderLocation(m_skybox_model->materials[0].shader, "environmentMap"),
      (int[1]){MATERIAL_MAP_CUBEMAP}, SHADER_UNIFORM_INT);
  SetShaderValue(
      m_skybox_model->materials[0].shader,
      GetShaderLocation(m_skybox_model->materials[0].shader, "doGamma"),
      (int[1]){0}, SHADER_UNIFORM_INT);
  SetShaderValue(
      m_skybox_model->materials[0].shader,
      GetShaderLocation(m_skybox_model->materials[0].shader, "vflipped"),
      (int[1]){0}, SHADER_UNIFORM_INT);

  m_skybox_model->materials[0].maps[MATERIAL_MAP_CUBEMAP].texture =
      LoadTextureCubemap(image,
                         CUBEMAP_LAYOUT_AUTO_DETECT); // CUBEMAP_LAYOUT_PANORAMA
}

void RaylibDevice::draw_frame() {
  m_camera.Update(CAMERA_CUSTOM);

  // Update the shader with the camera view vector (points towards { 0.0f,
  // 0.0f, 0.0f })
  std::array<float, 3> cameraPos = {m_camera.position.x, m_camera.position.y,
                                    m_camera.position.z};
  m_lighting_shader.SetValue(m_lighting_shader.locs[SHADER_LOC_VECTOR_VIEW],
                             cameraPos.data(), SHADER_UNIFORM_VEC3);

  // Update light values (actually, only enable/disable them)
  for (const auto &light : m_lights) {
    UpdateLightValues(m_lighting_shader, light);
  }
  BeginDrawing();

  ClearBackground(RAYWHITE);

  BeginMode3D(m_camera);

  if (m_skybox_model.has_value()) {
    // We are inside the cube, we need to disable backface culling!
    rlDisableBackfaceCulling();
    rlDisableDepthMask();
    DrawModel(*m_skybox_model, (Vector3){0, 0, 0}, 1.0f, WHITE);
    rlEnableBackfaceCulling();
    rlEnableDepthMask();
  }
  m_lighting_shader.BeginMode();

  // Draw all models.
  for (const auto &model : m_models) {
    model->draw();
  }

  m_lighting_shader.EndMode();
  EndMode3D();
  DrawFPS(10, 10);
  DrawText("to be put...", 10, 40, 20, DARKGRAY);
  EndDrawing();
}

RaylibDevice::~RaylibDevice() {
  m_lighting_shader.Unload(); // Unload shader
  CloseWindow();
}

////////////////////////////////////////////////////////////////////////////////
// Joystick control
////////////////////////////////////////////////////////////////////////////////

#if defined(PLATFORM_DESKTOP)
// Speial handling for platform desktop, since raylib doesn't seem to be able to
// recognize simple joysticks.

static int is_rc_control_heuristic(int jid) {

  // Num axes heuristic.
  int num_axes;
  glfwGetJoystickAxes(jid, &num_axes);
  if (num_axes < 4) {
    return false;
  }

  // If is considered a gamepad by GLFW, prefer it.
  if (glfwJoystickIsGamepad(jid)) {
    return 2;
  }

  // Name heuristic.
  std::string name(glfwGetJoystickName(jid));

  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (name.find("keyboard") != std::string::npos ||
      name.find("mouse") != std::string::npos ||
      name.find("touchpad") != std::string::npos ||
      name.find("elan") != std::string::npos) {
    return false;
  }

  return 1;
}

static std::vector<int> get_rc_controllers() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return {};
  }

  std::vector<int> result;
  for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; jid++) {
    if (glfwJoystickPresent(jid)) {
      int rc_controller_priority = is_rc_control_heuristic(jid);
      if (rc_controller_priority == 2) {
        return {jid};
      } else if (rc_controller_priority == 1) {
        std::cout << "Found joystick " << jid << ": "
                  << glfwGetJoystickName(jid) << std::endl;
        result.push_back(jid);
      }
    }
  }
  return result;
}

std::optional<Joystick> Joystick::get_available() {
  auto jids = get_rc_controllers();
  if (jids.empty()) {
    return std::nullopt;
  }
  return Joystick{jids[0]};
}

std::vector<float> Joystick::get_axes() const {
  int count;
  const float *axes = glfwGetJoystickAxes(m_jid, &count);
  return std::vector<float>(axes, axes + count);
}

#else
// For other platforms, use raylib's joystick handling.

std::optional<Joystick> Joystick::get_available() {
  for (int jid = 0; jid < 4; jid++) {
    if (IsGamepadAvailable(jid)) {
      return Joystick{jid};
    }
  }
  return std::nullopt;
}

std::vector<float> Joystick::get_axes() const {
  std::vector<float> axes;
  int axis_count = GetGamepadAxisCount(m_jid);
  for (int i = 0; i < axis_count; i++) {
    axes.push_back(GetGamepadAxisMovement(m_jid, i));
  }
  return axes;
}

#endif

} // namespace engine