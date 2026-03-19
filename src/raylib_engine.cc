#include "raylib_engine.h"

#ifdef PLATFORM_DESKTOP
const std::string LIGHTING_SHADER_PATH =
    "resources/shaders/glsl330/lighting_with_shadow";
const std::string SKYBOX_SHADER_PATH = "resources/shaders/glsl330/skybox";
#else // PLATFORM_ANDROID, PLATFORM_WEB
const std::string LIGHTING_SHADER_PATH =
    "resources/shaders/glsl300es/lighting_with_shadow";
const std::string SKYBOX_SHADER_PATH = "resources/shaders/glsl100/skybox";
#endif

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <iostream>
#include <numeric>
#include <string>

namespace engine {

// Values aligned with the shader.
const size_t MAX_LIGHTS = 4;
const size_t MAX_SHADOW_GROUPS = 4;

////////////////////////////////////////////////////////////////////////////////
// Light implementation
////////////////////////////////////////////////////////////////////////////////

Light::Light(vec3 position, vec3 target, Color color, LightType type, int index)
    : m_type(type), m_enabled(1), m_position(position), m_target(target),
      m_color(color), m_index(index), m_changed(true) {}

void Light::write_to_shader(raylib::Shader &shader) const {
  if (!m_changed) {
    return;
  }

  int enabledLoc =
      shader.GetLocation(TextFormat("lights[%i].enabled", m_index));
  int typeLoc = shader.GetLocation(TextFormat("lights[%i].type", m_index));
  int positionLoc =
      shader.GetLocation(TextFormat("lights[%i].position", m_index));
  int targetLoc = shader.GetLocation(TextFormat("lights[%i].target", m_index));
  int colorLoc = shader.GetLocation(TextFormat("lights[%i].color", m_index));

  // Send to shader light enabled state and type
  shader.SetValue(enabledLoc, &m_enabled, SHADER_UNIFORM_INT);
  shader.SetValue(typeLoc, &m_type, SHADER_UNIFORM_INT);

  // Send to shader light position values
  float position[3] = {m_position.x, m_position.y, m_position.z};
  shader.SetValue(positionLoc, position, SHADER_UNIFORM_VEC3);
  // Send to shader light target position values
  float target[3] = {m_target.x, m_target.y, m_target.z};
  shader.SetValue(targetLoc, target, SHADER_UNIFORM_VEC3);

  // Send to shader light color values
  float color[4] = {
      (float)m_color.r / (float)255, (float)m_color.g / (float)255,
      (float)m_color.b / (float)255, (float)m_color.a / (float)255};
  shader.SetValue(colorLoc, color, SHADER_UNIFORM_VEC4);
}

////////////////////////////////////////////////////////////////////////////////
// Model implementation
////////////////////////////////////////////////////////////////////////////////

bool Model::get_world_visible() const {
  return (m_parent ? m_parent->get_world_visible() : true) && m_visible;
}

bool Model::changed() const {
  return m_changed || (m_parent && m_parent->changed());
}

void Model::set_transform(const mat4 &transform) {
  m_local_transform = transform;
  m_changed = true;
}

mat4 Model::get_world_transform() {
  if (changed()) {
    if (m_parent) {
      m_world_transform = (raylib::Matrix)m_local_transform *
                          (raylib::Matrix)m_parent->get_world_transform();
    } else {
      m_world_transform = m_local_transform;
    }
  }

  return m_world_transform;
}

void Model::draw() {
  if (!get_world_visible()) {
    return;
  }
  m_model.SetTransform(get_world_transform());
  DrawModel(m_model, (Vector3){0, 0, 0}, 1.0f, WHITE);
}

std::vector<raylib::Material *> Model::get_materials() {
  std::vector<raylib::Material *> result;
  ::Material *materials = m_model.GetMaterials();
  for (int i = 0; i < m_model.GetMaterialCount(); i++) {
    result.push_back(reinterpret_cast<raylib::Material *>(&materials[i]));
  }
  return result;
}

vec3 Model::get_pos() { return get_world_transform() * vec3(0, 0, 0); }

////////////////////////////////////////////////////////////////////////////////
// Image2D implementation
////////////////////////////////////////////////////////////////////////////////

Image2D::Image2D(int x, int y)
    : m_image(x, y), m_image_changed(true),
      m_position{0, 0, (float)x, (float)y}, m_rotation(0), m_origin(0, 0) {}

Image2D::Image2D(std::string file_name)
    : m_image(file_name), m_image_changed(true),
      m_position{0, 0, (float)m_image.width, (float)m_image.height},
      m_rotation(0), m_origin(0, 0) {}

void Image2D::set_pixel_color(int x, int y, Color color) {
  if (x >= m_image.width || y >= m_image.height) {
    throw std::out_of_range("Pixel coordinates are out of bounds.");
  }

  m_image.DrawPixel(x, y, color);
  m_image_changed = true;
}

Color Image2D::get_pixel_color(int x, int y) {
  if (x >= m_image.width || y >= m_image.height) {
    throw std::out_of_range("Pixel coordinates are out of bounds.");
  }

  return m_image.GetColor(x, y);
}

void Image2D::_draw() {
  if (m_image_changed) {
    m_texture = m_image.LoadTexture();
    m_image_changed = false;
  }

  rect2 source_rect = {0, 0, (float)m_image.width, (float)m_image.height};
  m_texture.Draw(source_rect, m_position, m_origin, m_rotation);
}

////////////////////////////////////////////////////////////////////////////////
// Text2D implementation
////////////////////////////////////////////////////////////////////////////////

Text2D::Text2D(std::string text, int font_size, Color color,
               TextAlignment alignment)
    : m_text(text), m_font_size(font_size), m_color(color), m_position(0, 0),
      m_alignment(alignment) {}

void Text2D::_draw() {
  float text_pos_x = m_position.x;
  if (m_alignment == TextAlignment::CENTER) {
    text_pos_x -= MeasureText(m_text.c_str(), m_font_size) / 2.0f;
  } else if (m_alignment == TextAlignment::RIGHT) {
    text_pos_x -= MeasureText(m_text.c_str(), m_font_size);
  }
  raylib::DrawText(m_text.c_str(), (int)text_pos_x, (int)m_position.y,
                   m_font_size, m_color);
}

////////////////////////////////////////////////////////////////////////////////
// RaylibDevice implementation
////////////////////////////////////////////////////////////////////////////////

RenderTexture2D load_shadowmap_from_texture(int width, int height) {
  RenderTexture2D target = {0};
  target.id = rlLoadFramebuffer();

  if (target.id <= 0) {
    throw std::runtime_error("Couldn't load framebuffer.");
  }
  rlEnableFramebuffer(target.id);

  // 1. Create and attach a 1D color texture, in which the the shader will store
  // the depth values.
  target.texture.id =
      rlLoadTexture(NULL, width, height, RL_PIXELFORMAT_UNCOMPRESSED_R32, 1);
  target.texture.width = width;
  target.texture.height = height;
  target.texture.format = RL_PIXELFORMAT_UNCOMPRESSED_R32;
  target.texture.mipmaps = 1;
  SetTextureWrap(target.texture, RL_TEXTURE_WRAP_CLAMP);
  rlFramebufferAttach(target.id, target.texture.id,
                      RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);

  // 2. Create and attach a depth texture, required for the 3D rendering in the
  // shadow pass.
  //
  // Note: While on Linux, the depth texture is later a valid texture with depth
  // values, and thus can be used directly for the shadowmap, on Web this is not
  // the case, and no matter how hard I tried, the depth texture could not be
  // used directly. Thus, in the shader, I output the depth values to the color
  // texture, and use it as the shadowmap.
  target.depth.id = rlLoadTextureDepth(width, height, true);
  SetTextureWrap(target.depth, RL_TEXTURE_WRAP_CLAMP);
  target.depth.width = width;
  target.depth.height = height;
  target.depth.format = GL_DEPTH_COMPONENT24;
  target.depth.mipmaps = 1;

  rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH,
                      RL_ATTACHMENT_RENDERBUFFER, 0);

  if (rlFramebufferComplete(target.id)) {
    TraceLog(LOG_INFO, "Shadowmap FBO created successfully.");
  } else {
    throw std::runtime_error("Couldn't complete framebuffer.");
  }
  rlDisableFramebuffer();

  return target;
}

RaylibDevice::ShadowGroup::ShadowGroup(
    std::vector<std::shared_ptr<Model>> models, size_t size, float fov,
    int shader_index)
    : m_models(models), m_size(size), m_fov(fov), m_shadow_index(shader_index) {
  m_shadowmap = load_shadowmap_from_texture(m_size, m_size);
}

void RaylibDevice::ShadowGroup::shadow_pass(
    const std::shared_ptr<Light> &shadow_light, raylib::Shader &shader) {

  // Don't calculate the shadowmap if nothing changed.
  bool changed = shadow_light->changed();
  for (const auto &model : m_models) {
    if (model->changed()) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    return;
  }

  // Find the center of the shadow group models.
  vec3 center = std::accumulate(m_models.begin(), m_models.end(), vec3(0, 0, 0),
                                [](vec3 acc, std::shared_ptr<Model> model) {
                                  return acc + model->get_pos();
                                }) /
                m_models.size();

  raylib::Camera3D light_camera(
      /* position */ shadow_light->get_position(),
      /* target */ center,
      /* up */ vec3(0.0f, 1.0f, 0.0f),
      /* fovy */ m_fov);
  light_camera.projection = shadow_light->get_type() == LIGHT_DIRECTIONAL
                                ? CAMERA_ORTHOGRAPHIC
                                : CAMERA_PERSPECTIVE;

  // Make the shader output depth values to the color texture, to be used later
  // for shadowmap calculation.
  int output_depth = 1;
  shader.SetValue(shader.GetLocation("outputDepth"), &output_depth,
                  SHADER_UNIFORM_INT);

  rlActiveTextureSlot(texture_slot());
  rlDisableTexture();
  rlActiveTextureSlot(0);

  BeginTextureMode(m_shadowmap);

  ClearBackground(WHITE);

  BeginMode3D(light_camera);
  mat4 light_view = rlGetMatrixModelview();
  mat4 light_proj = rlGetMatrixProjection();
  for (const auto &model : m_models) {
    model->draw();
  }
  EndMode3D();
  EndTextureMode();

  // Reset shader output to normal rendering.
  output_depth = 0;
  shader.SetValue(shader.GetLocation("outputDepth"), &output_depth,
                  SHADER_UNIFORM_INT);

  mat4 light_view_proj = light_view * light_proj;

  shader.SetValue(
      shader.GetLocation(TextFormat("shadowMaps[%i].lightVP", m_shadow_index)),
      light_view_proj);
}

void RaylibDevice::ShadowGroup::write_shadowmap_to_shader(
    raylib::Shader &shader) {
  int slot = texture_slot();
  rlEnableShader(shader.id);
  rlActiveTextureSlot(slot);
  rlEnableTexture(m_shadowmap.texture.id);

  rlSetUniform(shader.GetLocation(
                   TextFormat("shadowMaps[%i].shadowMap", m_shadow_index)),
               &slot, SHADER_UNIFORM_INT, 1);
  shader.SetValue(shader.GetLocation(
                      TextFormat("shadowMaps[%i].resolution", m_shadow_index)),
                  &m_size, SHADER_UNIFORM_INT);
  int active = 1;
  shader.SetValue(
      shader.GetLocation(TextFormat("shadowMaps[%i].enabled", m_shadow_index)),
      &active, SHADER_UNIFORM_INT);
}

RaylibDevice::RaylibDevice(int screen_width, int screen_height,
                           std::string title)
    : m_camera(raylib::Vector3(2, 4, 6),
               raylib::Vector3(0, 0.5, 0.0)) // Initialize a default camera.
{

  SetConfigFlags(FLAG_MSAA_4X_HINT); // Enable Multi Sampling Anti Aliasing 4x
                                     // (if available)
  InitWindow(screen_width, screen_height,
             title.c_str()); // Create window and OpenGL context

  m_lighting_shader = raylib::Shader(LIGHTING_SHADER_PATH + ".vs",
                                     LIGHTING_SHADER_PATH + ".fs");

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

std::shared_ptr<Light> RaylibDevice::create_light(LightType type,
                                                  raylib::Vector3 position,
                                                  raylib::Vector3 target,
                                                  raylib::Color color) {
  size_t light_index = m_lights.size();
  if (light_index >= MAX_LIGHTS) {
    throw std::runtime_error("Too many lights");
  }

  std::shared_ptr<Light> light(
      new Light(position, target, color, type, light_index));
  m_lights.push_back(light);
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
  std::shared_ptr<Model> model =
      std::shared_ptr<Model>(new Model(mesh, parent));
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
  std::shared_ptr<Model> model =
      std::shared_ptr<Model>(new Model(mesh, parent));
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

  Shader skyboxShader = LoadShader((SKYBOX_SHADER_PATH + ".vs").c_str(),
                                   (SKYBOX_SHADER_PATH + ".fs").c_str());

  // Load skybox shader and set required locations
  // NOTE: Some locations are automatically set at shader loading
  m_skybox_model->materials[0].shader = skyboxShader;

  SetShaderValue(skyboxShader,
                 GetShaderLocation(skyboxShader, "environmentMap"),
                 (int[1]){MATERIAL_MAP_CUBEMAP}, SHADER_UNIFORM_INT);
  SetShaderValue(skyboxShader, GetShaderLocation(skyboxShader, "doGamma"),
                 (int[1]){0}, SHADER_UNIFORM_INT);
  SetShaderValue(skyboxShader, GetShaderLocation(skyboxShader, "vflipped"),
                 (int[1]){0}, SHADER_UNIFORM_INT);

  m_skybox_model->materials[0].maps[MATERIAL_MAP_CUBEMAP].texture =
      LoadTextureCubemap(image,
                         CUBEMAP_LAYOUT_AUTO_DETECT); // CUBEMAP_LAYOUT_PANORAMA
}

std::shared_ptr<RaylibDevice::ShadowGroup> &
RaylibDevice::add_shadow_group(std::vector<std::shared_ptr<Model>> models,
                               size_t size, float fov) {
  m_shadow_groups.push_back(
      std::shared_ptr<RaylibDevice::ShadowGroup>(new RaylibDevice::ShadowGroup(
          models, size, fov, m_shadow_groups.size())));
  return m_shadow_groups.back();
}

std::shared_ptr<Image2D> RaylibDevice::load_image2d(std::string file_name) {
  std::shared_ptr<Image2D> image2d =
      std::shared_ptr<Image2D>(new Image2D(file_name));
  m_2d_drawables.push_back(image2d);
  return image2d;
}

std::shared_ptr<Image2D> RaylibDevice::create_image2d(int width, int height) {
  std::shared_ptr<Image2D> image2d =
      std::shared_ptr<Image2D>(new Image2D(width, height));
  m_2d_drawables.push_back(image2d);
  return image2d;
}

std::shared_ptr<Text2D> RaylibDevice::create_text2d(std::string text,
                                                    int font_size, Color color,
                                                    TextAlignment alignment) {
  std::shared_ptr<Text2D> text2d =
      std::shared_ptr<Text2D>(new Text2D(text, font_size, color, alignment));
  m_2d_drawables.push_back(text2d);
  return text2d;
}

std::shared_ptr<Square2D> RaylibDevice::create_square2d(rect2 position,
                                                        Color color) {
  std::shared_ptr<Square2D> square2d =
      std::shared_ptr<Square2D>(new Square2D());
  square2d->set_position(position);
  square2d->set_color(color);
  m_2d_drawables.push_back(square2d);
  return square2d;
}

void RaylibDevice::delete_drawable2d(std::shared_ptr<Drawable2D> drawable) {
  m_2d_drawables.erase(
      std::remove(m_2d_drawables.begin(), m_2d_drawables.end(), drawable),
      m_2d_drawables.end());
}

void RaylibDevice::draw_frame() {
  m_camera.Update(CAMERA_CUSTOM);

  // Disable all shadow groups at the beginning of the frame, and they will be
  // enabled in the shadowmap class.
  for (int i = 0; i < MAX_SHADOW_GROUPS; i++) {
    int active = 0;
    m_lighting_shader.SetValue(
        m_lighting_shader.GetLocation(TextFormat("shadowMaps[%i].active", i)),
        &active, SHADER_UNIFORM_INT);
  }

  // Update the shader with the camera view vector (points towards { 0.0f,
  // 0.0f, 0.0f })
  std::array<float, 3> cameraPos = {m_camera.position.x, m_camera.position.y,
                                    m_camera.position.z};
  m_lighting_shader.SetValue(m_lighting_shader.locs[SHADER_LOC_VECTOR_VIEW],
                             cameraPos.data(), SHADER_UNIFORM_VEC3);

  // Update light values (actually, only enable/disable them)
  for (const auto &light : m_lights) {
    light->write_to_shader(m_lighting_shader);
  }
  BeginDrawing();

  for (auto shadow_group : m_shadow_groups) {
    shadow_group->shadow_pass(*m_lights.begin(), m_lighting_shader);
  }

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

  for (auto shadow_group : m_shadow_groups) {
    shadow_group->write_shadowmap_to_shader(m_lighting_shader);
  }

  // Draw all models.
  for (const auto &model : m_models) {
    model->draw();
  }

  m_lighting_shader.EndMode();
  EndMode3D();

  // Use to see the shadowmap:
  // DrawTextureEx(m_shadow_groups[0]->m_shadowmap.depth, (Vector2){20, 20},
  // 0.0f, 0.25f, WHITE);
  DrawFPS(10, 10);
  // DrawText("to be put...", 10, 40, 20, DARKGRAY);

  for (const auto &image2d : m_2d_drawables) {
    image2d->draw();
  }
  EndDrawing();

  for (auto model : m_models) {
    model->reset_changed();
  }
  for (auto light : m_lights) {
    light->reset_changed();
  }
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