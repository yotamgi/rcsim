#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "controls.h"
#include "flying_object.h"
#include "raylib_engine.h"
#include <vector>

class HorizontalControlsInstrument {
public:
  HorizontalControlsInstrument(engine::RaylibDevice *device, float pos_x,
                               int pos_y, int width, int height);
  void update(float pin1, float pin2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_yaw_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class VerticalControlsInstrument {
public:
  VerticalControlsInstrument(engine::RaylibDevice *device, float pos_x,
                             int pos_y, int width, int height);
  void update(float pin1, float pin2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_yaw_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class Controls2dInstrument {
public:
  Controls2dInstrument(engine::RaylibDevice *device, float pos_x, int pos_y,
                       int size);
  void update(float pin1_y, float pin1_x, float pin2_y, float pin2_x);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_controls_image;
  std::shared_ptr<engine::Image2D> m_pin_image;
  std::shared_ptr<engine::Image2D> m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_size;
};

class CurvesInstrument {
public:
  CurvesInstrument(engine::RaylibDevice *device,
                   std::vector<std::vector<ControllerCurve>> curve_groups,
                   std::vector<engine::Color> curve_colors, float pos_x,
                   int pos_y, int width, int height);
  void update(unsigned long active_curve_index, float x_stick_level,
              std::vector<float> y_levels);

private:
  engine::RaylibDevice *m_device;
  std::vector<std::shared_ptr<engine::Image2D>> m_curves_images;
  std::vector<std::shared_ptr<engine::Image2D>> m_pin_images;
  std::shared_ptr<engine::Image2D> m_curves_vertical_line;
  std::vector<std::vector<ControllerCurve>> m_curve_groups;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class SpeedometerInstrument {
public:
  SpeedometerInstrument(engine::RaylibDevice *device, float pos_x, int pos_y,
                        float max_value, std::string headline,
                        std::string value_template, std::string unit);
  void update(float value1, float value2);

private:
  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Image2D> m_speedometer_image;
  std::shared_ptr<engine::Image2D> m_speedometer_hand_image;
  std::shared_ptr<engine::Image2D> m_speedometer_hand2_image;
  std::shared_ptr<engine::Text2D> m_text_headline;
  std::shared_ptr<engine::Text2D> m_text_value;
  std::shared_ptr<engine::Text2D> m_text_unit;
  int m_pos_x;
  int m_pos_y;
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
  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  CurvesInstrument m_curves_instrument;
  SpeedometerInstrument m_main_rotor_instrument;

  // // Texts.
  // std::shared_ptr<engine::Image2D> m_throttle_text;
  // std::shared_ptr<engine::Image2D> m_throttle_hold_text;
  // std::shared_ptr<engine::Image2D> m_curves_text;
  // std::shared_ptr<engine::Image2D> m_switch_curves_text;
  // std::shared_ptr<engine::Image2D> m_controls_and_gyro_view_text;

  std::shared_ptr<engine::Image2D> m_dashboard_background;
};

class PlaneDashboard : public Dashboard {
public:
  PlaneDashboard(engine::RaylibDevice *device, float max_airspeed);

  void update_ui(const Controls::Telemetry &controls_telemetry,
                 const FlyingObject::Telemetry &telemetry);

private:
  engine::RaylibDevice *m_device;
  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  VerticalControlsInstrument m_throttle_instrument;
  SpeedometerInstrument m_airspeed_instrument;
  std::shared_ptr<engine::Image2D> m_dashboard_background;
};

#endif // __DASHBOARD_H__
