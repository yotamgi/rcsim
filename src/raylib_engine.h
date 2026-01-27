#ifndef __RAYLIB_ENGINE_H__
#define __RAYLIB_ENGINE_H__

#include "raylib-cpp.hpp"
#include "rlights.h"

#include <memory>
#include <optional>
#include <vector>

static raylib::Vector3 operator*(const raylib::Matrix &m,
                                 const raylib::Vector3 &v) {
  return ::Vector3Transform(v, m);
}

static raylib::Vector3 operator*(const float &scalar,
                                 const raylib::Vector3 &v) {
  return v * scalar;
}

namespace engine {

typedef raylib::Vector3 vec3;
typedef raylib::Matrix mat4;
typedef raylib::Texture Texture;
typedef raylib::Material Material;

static float &mat_get(mat4 &m, int row, int col) {
  size_t index = row * 4 + col;
  // clang-format off
  switch (index) {
  case 0: return m.m0; break; case 1: return m.m1; break;
  case 2: return m.m2; break; case 3: return m.m3; break;
  case 4: return m.m4; break; case 5: return m.m5; break;
  case 6: return m.m6; break; case 7: return m.m7; break;
  case 8: return m.m8; break; case 9: return m.m9; break;
  case 10: return m.m10; break; case 11: return m.m11; break;
  case 12: return m.m12; break; case 13: return m.m13; break;
  case 14: return m.m14; break; case 15: return m.m15; break;
  default:
    throw std::out_of_range("Matrix index out of range");
  }
  // clang-format on
}

using raylib::Keyboard::IsKeyDown;
using raylib::Keyboard::IsKeyPressed;

class Model {
public:
  void set_transform(const mat4 &transform) { m_local_transform = transform; }
  const mat4 &get_transform() const { return m_local_transform; }
  mat4 get_world_transform() const;

  std::vector<raylib::Material *> get_materials();

  void draw();

  friend class RaylibDevice;

private:
  Model(std::string file_name, std::shared_ptr<Model> parent)
      : m_model(file_name), m_parent(parent) {}
  Model(std::shared_ptr<::Mesh> mesh, std::shared_ptr<Model> parent)
      : m_model(*mesh), m_mesh(mesh), m_parent(parent) {}
  Model(std::shared_ptr<Model> parent) : m_parent(parent) {}

  mat4 m_local_transform;
  std::shared_ptr<Model> m_parent;
  raylib::Model m_model;
  std::shared_ptr<::Mesh> m_mesh = nullptr;
};

class RaylibDevice {
public:
  RaylibDevice(int screen_width, int screen_height, std::string title);
  ~RaylibDevice();

  raylib::Camera3D &get_camera() { return m_camera; }

  std::shared_ptr<Model> create_empty(std::shared_ptr<Model> parent = nullptr);
  std::shared_ptr<Model> load_model(const std::string &file_name,
                                    std::shared_ptr<Model> parent = nullptr,
                                    bool enable_lighting = true);
  std::shared_ptr<Model> create_sphere(float radius, int rings, int slices,
                                       std::shared_ptr<Model> parent = nullptr,
                                       bool enable_lighting = true);
  std::shared_ptr<Model> create_cube(float width, float height, float length,
                                     std::shared_ptr<Model> parent = nullptr,
                                     bool enable_lighting = true);
  Light &create_light(int type, raylib::Vector3 position,
                      raylib::Vector3 target, raylib::Color color);

  void add_skybox_from_single_image(std::string image_path);
  void add_skybox_from_6_images(std::string right, std::string left,
                                std::string top, std::string bottom,
                                std::string back, std::string front);

  void draw_frame();

  friend class Model;

private:
  void add_skybox_from_image(const raylib::Image &image);

  raylib::Camera3D m_camera;
  raylib::Shader m_lighting_shader;
  std::vector<Light> m_lights;
  std::optional<raylib::Model> m_skybox_model;
  std::vector<std::shared_ptr<Model>> m_models;
};

class Joystick {
public:
  static std::optional<Joystick> get_available();

  std::vector<float> get_axes() const;

private:
  Joystick(int jid) : m_jid(jid) {}
  int m_jid;
};

} // namespace engine

#endif // __RAYLIB_ENGINE_H__