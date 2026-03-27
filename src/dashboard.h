#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "controls.h"
#include "flying_object.h"
#include "raylib_engine.h"
#include <vector>

class HorizontalControlsInstrument {
public:
  HorizontalControlsInstrument(
      engine::RaylibDevice *device, engine::Coord2D pos_x,
      engine::Coord2D pos_y, int width, int height,
      std::shared_ptr<engine::Drawable2D> parent = nullptr);
  void update(float pin1, float pin2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_control_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  engine::Coord2D m_pos_x;
  engine::Coord2D m_pos_y;
  int m_width;
  int m_height;
};

class VerticalControlsInstrument {
public:
  VerticalControlsInstrument(
      engine::RaylibDevice *device, engine::Coord2D pos_x,
      engine::Coord2D pos_y, int width, int height,
      std::shared_ptr<engine::Drawable2D> parent = nullptr);
  void update(float pin1, float pin2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_control_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  engine::Coord2D m_pos_x;
  engine::Coord2D m_pos_y;
  int m_width;
  int m_height;
};

class Controls2dInstrument {
public:
  Controls2dInstrument(engine::RaylibDevice *device, engine::Coord2D pos_x,
                       engine::Coord2D pos_y, int size,
                       std::shared_ptr<engine::Drawable2D> parent);
  void update(float pin1_y, float pin1_x, float pin2_y, float pin2_x);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_controls_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  engine::Coord2D m_pos_x;
  engine::Coord2D m_pos_y;
  int m_size;
};

class CurvesInstrument {
public:
  CurvesInstrument(engine::RaylibDevice *device,
                   std::vector<std::vector<ControllerCurve>> curve_groups,
                   std::vector<engine::Color> curve_colors,
                   engine::Coord2D pos_x, engine::Coord2D pos_y, int width,
                   int height, std::vector<std::string> curve_group_names,
                   std::vector<std::string> curve_names,
                   std::shared_ptr<engine::Drawable2D> parent);
  void update(unsigned long active_curve_index, float x_stick_level,
              std::vector<float> y_levels);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Square2D> m_main_node;
  std::vector<std::shared_ptr<engine::Image2D>> m_curves_images;
  std::vector<std::shared_ptr<engine::Image2D>> m_pin_images;
  std::shared_ptr<engine::Image2D> m_curves_vertical_line;
  std::vector<std::vector<ControllerCurve>> m_curve_groups;
  std::vector<std::shared_ptr<engine::Text2D>> m_headline_text;
  std::vector<std::shared_ptr<engine::Text2D>> m_curve_texts;
  std::vector<std::string> m_curve_names;
  engine::Coord2D m_pos_x;
  engine::Coord2D m_pos_y;
  int m_width;
  int m_height;
};

class SpeedometerInstrument {
public:
  SpeedometerInstrument(engine::RaylibDevice *device, engine::Coord2D pos_x,
                        engine::Coord2D pos_y, float max_value,
                        std::string headline, std::string value_template,
                        std::string unit,
                        std::shared_ptr<engine::Drawable2D> parent);
  void update(float value1, float value2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_speedometer_image;
  std::shared_ptr<engine::Image2D> m_speedometer_hand_image;
  std::shared_ptr<engine::Image2D> m_speedometer_hand2_image;
  std::shared_ptr<engine::Text2D> m_text_headline;
  std::shared_ptr<engine::Text2D> m_text_value;
  std::shared_ptr<engine::Text2D> m_text_unit;
  engine::Coord2D m_pos_x;
  engine::Coord2D m_pos_y;
  float m_max_value;
  std::string m_text_value_template;
};

class Dashboard {
public:
  virtual void update_ui(const Controls::Telemetry &controls_telemetry,
                         const FlyingObject::Telemetry &telemetry) = 0;
};

class HeliDashboard : public Dashboard {
public:
  HeliDashboard(engine::RaylibDevice *device,
                std::vector<ControllerCurve> throttle_curves,
                std::vector<ControllerCurve> lift_curves, float max_rps);

  void update_ui(const Controls::Telemetry &controls_telemetry,
                 const FlyingObject::Telemetry &telemetry);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Square2D> m_dashboard;
  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  CurvesInstrument m_curves_instrument;
  SpeedometerInstrument m_main_rotor_instrument;
  std::shared_ptr<engine::Text2D> m_input_text;
};

class PlaneDashboard : public Dashboard {
public:
  PlaneDashboard(engine::RaylibDevice *device, float max_airspeed);

  void update_ui(const Controls::Telemetry &controls_telemetry,
                 const FlyingObject::Telemetry &telemetry);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Square2D> m_dashboard;
  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  VerticalControlsInstrument m_throttle_instrument;
  SpeedometerInstrument m_airspeed_instrument;
};

#endif // __DASHBOARD_H__
