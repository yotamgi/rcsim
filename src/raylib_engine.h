#ifndef __RAYLIB_ENGINE_H__
#define __RAYLIB_ENGINE_H__

#include "raylib-cpp.hpp"

#include <memory>
#include <optional>
#include <sstream>
#include <vector>

static raylib::Vector3 operator*(const raylib::Matrix &m,
                                 const raylib::Vector3 &v) {
  return ::Vector3Transform(v, m);
}

static raylib::Vector3 operator*(const float &scalar,
                                 const raylib::Vector3 &v) {
  return v * scalar;
}

static raylib::Color operator*(const float &scalar, const raylib::Color &v) {
  return raylib::Color(v.r * scalar, v.g * scalar, v.b * scalar, v.a * scalar);
}
static raylib::Color operator*(const raylib::Color &v, const float &scalar) {
  return raylib::Color(v.r * scalar, v.g * scalar, v.b * scalar, v.a * scalar);
}

namespace engine {

typedef raylib::Vector3 vec3;
typedef raylib::Vector2 vec2;
typedef raylib::Matrix mat4;
typedef raylib::Texture Texture;
typedef raylib::Material Material;
typedef raylib::Color Color;

static raylib::Vector3 mat_to_angles_zyx(const raylib::Matrix &m) {
  ::Quaternion q = ::QuaternionFromMatrix(m);
  return ::QuaternionToEuler(q);
}

static raylib::Matrix angles_zyx_to_mat(const raylib::Vector3 &angles) {
  ::Quaternion q = ::QuaternionFromEuler(angles.x, angles.y, angles.z);
  return ::QuaternionToMatrix(q);
}

struct Coord2D {
  Coord2D() : val{.pixel = 0}, type(Type::PIXEL) {}
  Coord2D(int pixel) : val{.pixel = pixel}, type(Type::PIXEL) {}
  Coord2D(float ratio) : val{.ratio = ratio}, type(Type::RATIO) {}
  union {
    int pixel;
    float ratio;
  } val;
  enum class Type { PIXEL, RATIO } type;

  std::string to_str() {
    std::stringstream ss;
    if (type == Type::PIXEL) {
      ss << val.pixel << " pixels";
    } else {
      ss << val.ratio << " ratio";
    }
    return ss.str();
  }
};

struct Rect2D {
  Coord2D x;
  Coord2D y;
  Coord2D width;
  Coord2D height;
  enum class Origin { MIN, MID, MAX };
  Origin x_orig = Origin::MIN;
  Origin y_orig = Origin::MIN;
};

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

// Light type
enum LightType { LIGHT_DIRECTIONAL = 0, LIGHT_POINT };

class Light {
public:
  const vec3 &get_position() const { return m_position; }
  const void set_position(const vec3 &position) {
    m_position = position;
    m_changed = true;
  }
  const vec3 &get_target() const { return m_target; }
  const void set_target(const vec3 &target) {
    m_target = target;
    m_changed = true;
  }
  const bool enabled() const { return m_enabled == 1; }
  const void set_enabled(bool enabled) {
    m_enabled = enabled ? 1 : 0;
    m_changed = true;
  }
  const Color &get_color() const { return m_color; }
  const void set_color(const Color &color) {
    m_color = color;
    m_changed = true;
  }

  LightType get_type() const { return m_type; }

private:
  Light(vec3 position, vec3 target, Color color, LightType type, int index);

  void write_to_shader(raylib::Shader &shader) const;
  void reset_changed() { m_changed = false; }
  bool changed() const { return m_changed; }

  LightType m_type;
  int m_enabled;
  vec3 m_position;
  vec3 m_target;
  Color m_color;
  int m_index;
  bool m_changed;

  friend class RaylibDevice;
};

class Model {
public:
  void set_transform(const mat4 &transform);
  const mat4 &get_transform() const { return m_local_transform; }
  mat4 get_world_transform();

  std::vector<raylib::Material *> get_materials();
  vec3 get_pos();
  bool get_visible() const { return m_visible; }
  bool get_world_visible() const;
  void set_visible(bool visible) { m_visible = visible; }

  friend class RaylibDevice;

private:
  Model(std::string file_name, std::shared_ptr<Model> parent)
      : m_model(file_name), m_parent(parent), m_changed(true) {}
  Model(std::shared_ptr<::Mesh> mesh, std::shared_ptr<Model> parent)
      : m_model(*mesh), m_mesh(mesh), m_parent(parent), m_changed(true) {}
  Model(std::shared_ptr<Model> parent) : m_parent(parent), m_changed(true) {}

  bool changed() const;
  void reset_changed() { m_changed = false; }

  void draw();

  mat4 m_local_transform;
  mat4 m_world_transform;
  std::shared_ptr<Model> m_parent;
  raylib::Model m_model;
  bool m_changed;
  std::shared_ptr<::Mesh> m_mesh = nullptr;
  bool m_visible = true;
};

class Drawable2D {
public:
  const bool visible() const { return m_visible; }
  void set_visible(bool visible) { m_visible = visible; }
  bool get_screen_visible();
  const Rect2D &get_position() const { return m_rect; }
  void set_position(const Rect2D &rect) { m_rect = rect; }
  void set_position(Coord2D x, Coord2D y) {
    m_rect.x = x;
    m_rect.y = y;
  }
  void set_position(Coord2D x, Coord2D y, Rect2D::Origin x_orig,
                    Rect2D::Origin y_orig) {
    m_rect.x = x;
    m_rect.y = y;
    m_rect.x_orig = x_orig;
    m_rect.y_orig = y_orig;
  }

protected:
  Drawable2D(std::shared_ptr<Drawable2D> parent = nullptr)
      : m_parent(parent), m_visible(true) {}
  virtual void draw() {
    if (get_screen_visible())
      _draw();
  }
  raylib::Rectangle get_screen_rect() const;
  std::shared_ptr<Drawable2D> m_parent = nullptr;
  virtual void _draw() = 0;
  bool m_visible;
  Rect2D m_rect;
  friend class RaylibDevice;
};

class Image2D : public Drawable2D {
public:
  const float &get_rotation() const { return m_rotation; }
  void set_rotation(float rotation) { m_rotation = rotation; }
  const vec2 &get_rotation_axis() const { return m_rotation_axis; }
  void set_rotation_axis(const vec2 &axis) { m_rotation_axis = axis; }
  Color get_pixel_color(int x, int y);
  void set_pixel_color(int x, int y, Color color);

protected:
  Image2D(std::string file_name, std::shared_ptr<Drawable2D> parent = nullptr);
  Image2D(int width, int height, std::shared_ptr<Drawable2D> parent = nullptr);

  virtual void _draw();

  raylib::Image m_image;
  raylib::Texture2D m_texture;
  bool m_image_changed;
  vec2 m_rotation_axis;
  float m_rotation;
  bool m_visible;

  friend class RaylibDevice;
};

class Square2D : public Image2D {
public:
  void set_color(Color color) { set_pixel_color(0, 0, color); }

protected:
  Square2D(std::shared_ptr<Drawable2D> parent = nullptr)
      : Image2D(1, 1, parent) {}
  friend class RaylibDevice;
};

class Text2D : public Drawable2D {
public:
  struct FontOptions {
    int font_size = 20;
    Color color = {0, 0, 0, 255};
    float line_spacing = 1.15;
  };
  const FontOptions &get_font_options() const { return m_font_options; }
  void set_font_options(const FontOptions &options) {
    m_font_options = options;
  }
  std::string get_text() const;
  void set_text(const std::string &text);

private:
  Text2D(std::string text, const FontOptions &options,
         std::shared_ptr<Drawable2D> parent = nullptr);

  virtual void _draw();

  std::vector<std::string> m_lines;
  FontOptions m_font_options;

  friend class RaylibDevice;
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
  std::shared_ptr<Light> create_light(LightType type, raylib::Vector3 position,
                                      raylib::Vector3 target,
                                      raylib::Color color);
  Color get_ambient_light() const { return m_ambient_light; }
  void set_ambient_light(Color color);
  int get_screen_width() const { return ::GetScreenWidth(); }
  int get_screen_height() const { return ::GetScreenHeight(); }

  void add_skybox_from_single_image(std::string image_path);
  void add_skybox_from_6_images(std::string right, std::string left,
                                std::string top, std::string bottom,
                                std::string back, std::string front);

  friend class Model;
  friend class Light;
  friend class Drawable2D;

  class ShadowGroup {
  public:
    std::vector<std::shared_ptr<Model>> &get_models() { return m_models; };

  private:
    ShadowGroup() = delete;
    ShadowGroup(std::vector<std::shared_ptr<Model>> models, size_t size,
                float fov, int shader_index);
    void shadow_pass(const std::shared_ptr<Light> &shadow_light,
                     raylib::Shader &shader);
    void write_shadowmap_to_shader(raylib::Shader &shader);
    int texture_slot() const { return 8 + m_shadow_index; }
    raylib::RenderTexture2D m_shadowmap;
    std::vector<std::shared_ptr<Model>> m_models;
    const size_t m_size;
    float m_fov;
    int m_shadow_index;
    friend class RaylibDevice;
  };

  std::shared_ptr<ShadowGroup> &
  add_shadow_group(std::vector<std::shared_ptr<Model>> models, size_t size,
                   float fov);

  std::shared_ptr<Image2D>
  load_image2d(std::string file_name,
               std::shared_ptr<Drawable2D> parent = nullptr);
  std::shared_ptr<Image2D>
  create_image2d(int width, int height,
                 std::shared_ptr<Drawable2D> parent = nullptr);
  std::shared_ptr<Text2D>
  create_text2d(std::string text, Text2D::FontOptions options,
                std::shared_ptr<Drawable2D> parent = nullptr);
  std::shared_ptr<Square2D>
  create_square2d(Color color, std::shared_ptr<Drawable2D> parent = nullptr);
  void delete_drawable2d(std::shared_ptr<Drawable2D> drawable);

  void draw_frame();

private:
  void add_skybox_from_image(const raylib::Image &image);

  raylib::Camera3D m_camera;
  raylib::Shader m_lighting_shader;
  std::vector<std::shared_ptr<Light>> m_lights;
  std::optional<raylib::Model> m_skybox_model;
  std::vector<std::shared_ptr<Model>> m_models;
  std::vector<std::shared_ptr<ShadowGroup>> m_shadow_groups;
  std::vector<std::shared_ptr<Drawable2D>> m_2d_drawables;
  Color m_ambient_light;
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