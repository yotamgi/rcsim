#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "controls.h"
#include "flying_object.h"
#include <irrlicht/irrlicht.h>
#include <vector>

class HorizontalControlsInstrument {
public:
  HorizontalControlsInstrument(irr::video::IVideoDriver *driver, float pos_x,
                               int pos_y, int width, int height);
  void update(float pin1, float pin2);

private:
  irr::video::IVideoDriver *m_driver;
  irr::video::ITexture *m_yaw_image;
  irr::video::ITexture *m_pin_image;
  irr::video::ITexture *m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class VerticalControlsInstrument {
public:
  VerticalControlsInstrument(irr::video::IVideoDriver *driver, float pos_x,
                             int pos_y, int width, int height);
  void update(float pin1, float pin2);

private:
  irr::video::IVideoDriver *m_driver;
  irr::video::ITexture *m_yaw_image;
  irr::video::ITexture *m_pin_image;
  irr::video::ITexture *m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class Controls2dInstrument {
public:
  Controls2dInstrument(irr::video::IVideoDriver *driver, float pos_x, int pos_y,
                       int size);
  void update(float pin1_y, float pin1_x, float pin2_y, float pin2_x);

private:
  irr::video::IVideoDriver *m_driver;
  irr::video::ITexture *m_controls_image;
  irr::video::ITexture *m_pin_image;
  irr::video::ITexture *m_second_pin_image;
  int m_pos_x;
  int m_pos_y;
  int m_size;
};

class CurvesInstrument {
public:
  CurvesInstrument(irr::video::IVideoDriver *driver,
                   std::vector<std::vector<ControllerCurve>> curve_groups,
                   std::vector<irr::video::SColor> curve_colors, float pos_x,
                   int pos_y, int width, int height);
  void update(unsigned long active_curve_index, float x_stick_level,
              std::vector<float> y_levels);

private:
  irr::video::IVideoDriver *m_driver;
  std::vector<irr::video::ITexture *> m_curves_images;
  irr::video::ITexture *m_pin_image;
  irr::video::ITexture *m_curves_vertical_line;
  std::vector<std::vector<ControllerCurve>> m_curve_groups;
  int m_pos_x;
  int m_pos_y;
  int m_width;
  int m_height;
};

class SpeedometerInstrument {
public:
  SpeedometerInstrument(irr::video::IVideoDriver *driver, float pos_x,
                        int pos_y, float max_value);
  void update(float value1, float value2);

private:
  irr::video::IVideoDriver *m_driver;
  irr::video::ITexture *m_speedometer_image;
  irr::video::ITexture *m_speedometer_hand_image;
  irr::video::ITexture *m_speedometer_hand2_image;
  int m_pos_x;
  int m_pos_y;
  float m_max_value;
};

class Dashboard {
public:
  virtual void update_ui(const Controls::Telemetry &controls_telemetry,
                         const FlyingObject::Telemetry &telemetry) = 0;
};

class HeliDashboard : public Dashboard {
public:
  HeliDashboard(irr::video::IVideoDriver *driver,
                std::vector<ControllerCurve> throttle_curves,
                std::vector<ControllerCurve> lift_curves, float max_rps);

  void update_ui(const Controls::Telemetry &controls_telemetry,
                 const FlyingObject::Telemetry &telemetry);

private:
  irr::video::IVideoDriver *m_driver;

  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  CurvesInstrument m_curves_instrument;
  SpeedometerInstrument m_main_rotor_instrument;

  // Texts.
  irr::video::ITexture *m_throttle_text;
  irr::video::ITexture *m_throttle_hold_text;
  irr::video::ITexture *m_curves_text;
  irr::video::ITexture *m_switch_curves_text;
  irr::video::ITexture *m_controls_and_gyro_view_text;

  irr::video::ITexture *m_dashboard_background;
};

class PlaneDashboard : public Dashboard {
public:
  PlaneDashboard(irr::video::IVideoDriver *driver, float max_airspeed);

  void update_ui(const Controls::Telemetry &controls_telemetry,
                 const FlyingObject::Telemetry &telemetry);

private:
  irr::video::IVideoDriver *m_driver;
  HorizontalControlsInstrument m_yaw_instrument;
  Controls2dInstrument m_pitch_roll_instrument;
  VerticalControlsInstrument m_throttle_instrument;
  SpeedometerInstrument m_airspeed_instrument;
  irr::video::ITexture *m_dashboard_background;
};

#endif // __DASHBOARD_H__
