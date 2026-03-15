#include "dashboard.h"
#include <cmath>

#include <format>
#include <iostream>
#include <sstream>

const int BACKGROUND_HEIGHT = 220;

const int CONTROLS_SIZE = 160;
const int YAW_VIEW_HEIGHT = 25;
const int PIN_SIZE = 15;
const float CONTROLS_POS_X = 0.15;
const int CONTROLS_POS_Y = 200;
const int YAW_VIEW_POS_Y = CONTROLS_POS_Y - CONTROLS_SIZE - 2;
const float YAW_VIEW_POS_X = CONTROLS_POS_X;

const float CURVES_IMAGE_POS_X = 0.7;
const int CURVES_IMAGE_POS_Y = 180;
const int CURVES_IMAGE_HEIGHT = 150;
const int CURVES_IMAGE_WIDTH = 275;
const int CURVES_GRID_SIZE = 25;
const int CURVE_THICKNESS = 3.5;

const float MAIN_ROTOR_INDICATOR_X = 0.4;
const int MAIN_ROTOR_INDICATOR_Y = 200;
const int MAIN_ROTOR_INDICATOR_WIDTH = 300;
const int MAIN_ROTOR_INDICATOR_HEIGHT = 170;
const int MAIN_ROTOR_INDICATOR_HAND_WIDTH = 39;
const int MAIN_ROTOR_INDICATOR_HAND_HEIGHT = 140;

const int TEXT_HEIGHT = 60;
const int TEXT_WIDTH = 60 * 8;

static float curve_x_to_value(int x) {
  return (float(x) / CURVES_IMAGE_WIDTH) * 2. - 1.;
}
static float curve_value_to_y(float value) {
  return float(CURVES_IMAGE_HEIGHT) * ((value + 1.) / 2.);
}

static float pos_y_(engine::RaylibDevice *device, int pos_y) {
  return float(device->get_screen_height() - pos_y);
}

static float pos_x_ratio(engine::RaylibDevice *device, float pos_x_ratio) {
  return float(device->get_screen_width() * pos_x_ratio);
}

#define POS_Y(y) pos_y_(m_device, (y))
#define POS_X(x) pos_x_ratio(m_device, (x))

static void plot_curve(const ControllerCurve &curve,
                       std::shared_ptr<engine::Image2D> image,
                       const engine::Color &color,
                       int image_width = CURVES_IMAGE_WIDTH,
                       int image_height = CURVES_IMAGE_HEIGHT) {
  float curve_y_value;

  for (int x = 0; x < image_width; x++) {
    float in_value = curve_x_to_value(x);
    float new_curve_y_value = curve_value_to_y(-curve.translate(in_value));
    float grad = (x == 0) ? 0 : (new_curve_y_value - curve_y_value);
    curve_y_value = new_curve_y_value;

    float grad_angle = PI / 2 - std::atan(grad);
    float thickness_squared =
        std::pow(CURVE_THICKNESS / std::sin(grad_angle), 3);

    for (int y = 0; y < image_height; y++) {
      float distance = std::abs(float(y) - curve_y_value);

      float value =
          (thickness_squared - std::pow(distance, 3)) / thickness_squared;
      value = (value > 0) ? value : 0;
      engine::Color prev_color = image->get_pixel_color(x, y);
      int new_alpha =
          std::max((unsigned int)prev_color.a, (unsigned int)(value * color.a));
      int new_red = value * color.r + (1 - value) * prev_color.r;
      int new_green = value * color.g + (1 - value) * prev_color.g;
      int new_blue = value * color.b + (1 - value) * prev_color.b;
      image->set_pixel_color(
          x, y, engine::Color(new_red, new_green, new_blue, new_alpha));
    }
  }
}

static std::shared_ptr<engine::Image2D>
create_circular_pin_image(engine::RaylibDevice *device, int size,
                          const engine::Color &color) {
  std::shared_ptr<engine::Image2D> pin_image =
      device->create_image2d(size, size);

  for (int x = 0; x < size; x++) {
    for (int y = 0; y < size; y++) {
      int half = size / 2;
      float distance =
          std::pow(float((x - half) * (x - half) + (y - half) * (y - half)) /
                       (half * half),
                   3);
      float intensity = distance > 1 ? 0 : 1 - distance;
      engine::Color color = engine::Color(0xff, 0xff, 0, 150 * intensity);
      pin_image->set_pixel_color(x, y, color);
    }
  }
  return pin_image;
}

static std::shared_ptr<engine::Image2D>
create_plus_pin_image(engine::RaylibDevice *device, int size,
                      const engine::Color &color) {

  std::shared_ptr<engine::Image2D> pin_image =
      device->create_image2d(size, size);

  for (int x = 0; x < size; x++) {
    for (int y = 0; y < size; y++) {
      int half = size / 2;
      if ((x < half + 2 && x > half - 2) || (y < half + 2 && y > half - 2)) {
        pin_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
      } else {
        pin_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0));
      }
    }
  }
  return pin_image;
}

////////////////////////////////////////////////////////////////////////////////
// HorizontalControlsInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

HorizontalControlsInstrument::HorizontalControlsInstrument(
    engine::RaylibDevice *device, float pos_x, int pos_y, int width, int height)
    : m_device(device), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_width(width), m_height(height) {

  // Create the yaw control image.
  m_yaw_image = m_device->create_image2d(width, height);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int half_x = width / 2;
      int half_y = height / 2;
      int x_fade = 0.25 * width;
      // Draw the axes;
      if ((x > half_x - 1 && x < half_x + 1) ||
          (y > half_y - 1 && y < half_y + 1)) {
        m_yaw_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
      }
      // Draw the gradient;
      else {
        float distance = float(std::abs(y - half_y)) / half_y;
        float intensity = 1 - distance;
        if (x < x_fade) {
          intensity *= 1 - std::pow(float(x_fade - x) / x_fade, 2);
        } else if (x > width - x_fade) {
          intensity *= 1 - std::pow(float(x - (width - x_fade)) / x_fade, 2);
        }
        m_yaw_image->set_pixel_color(
            x, y, engine::Color(0xA0, 0, 0, 150 * intensity));
      }
    }
  }

  m_pin_image = create_circular_pin_image(m_device, PIN_SIZE,
                                          engine::Color(0xff, 0xff, 0, 0));
  m_second_pin_image =
      create_plus_pin_image(m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0));

  // The yaw image is static, so it's position is set in the constructor.
  m_yaw_image->set_position(
      {(float)m_pos_x, (float)m_pos_y, (float)m_width, (float)m_height});
}

void HorizontalControlsInstrument::update(float pin1, float pin2) {
  // Draw the yaw controls view.
  float pin1_x = m_pos_x + m_width / 2 + pin1 * m_width / 2 - PIN_SIZE / 2;
  float pin1_y = m_pos_y + m_height / 2 - PIN_SIZE / 2;
  float pin2_x = m_pos_x + m_width / 2 + pin2 * m_width / 2 - PIN_SIZE / 2;
  float pin2_y = m_pos_y + m_height / 2 - PIN_SIZE;
  m_pin_image->set_position(
      {(float)pin2_x, (float)pin2_y, (float)PIN_SIZE, (float)PIN_SIZE * 2});
  m_second_pin_image->set_position(
      {(float)pin1_x, (float)pin1_y, (float)PIN_SIZE, (float)PIN_SIZE});
}

////////////////////////////////////////////////////////////////////////////////
// VerticalControlsInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

VerticalControlsInstrument::VerticalControlsInstrument(
    engine::RaylibDevice *device, float pos_x, int pos_y, int width, int height)
    : m_device(device), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_width(width), m_height(height) {

  // Create the yaw control image.
  m_yaw_image = device->create_image2d(width, height);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int half_x = width / 2;
      int half_y = height / 2;
      int x_fade = 0.25 * width;
      // Draw the axes;
      if ((x > half_x - 1 && x < half_x + 1) ||
          (y > half_y - 1 && y < half_y + 1)) {
        m_yaw_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
      }
      // Draw the gradient;
      else {
        float distance = float(std::abs(y - half_y)) / half_y;
        float intensity = 1 - distance;
        if (x < x_fade) {
          intensity *= 1 - std::pow(float(x_fade - x) / x_fade, 2);
        } else if (x > width - x_fade) {
          intensity *= 1 - std::pow(float(x - (width - x_fade)) / x_fade, 2);
        }
        m_yaw_image->set_pixel_color(
            x, y, engine::Color(0xA0, 0, 0, 150 * intensity));
      }
    }
  }

  m_pin_image = create_circular_pin_image(m_device, PIN_SIZE,
                                          engine::Color(0xff, 0xff, 0, 0));
  m_second_pin_image =
      create_plus_pin_image(m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0));
}

void VerticalControlsInstrument::update(float pin1, float pin2) {
  // Draw the yaw controls view.
  int pin1_x = m_pos_x + m_width / 2 - PIN_SIZE / 2;
  int pin1_y = m_pos_y + m_height / 2 - pin1 * m_height / 2 - PIN_SIZE / 2;
  int pin2_x = m_pos_x + m_width / 2 - PIN_SIZE;
  int pin2_y = m_pos_y + m_height / 2 - pin2 * m_height / 2 - PIN_SIZE / 2;
  m_yaw_image->set_position(engine::rect2{(float)m_pos_x, (float)m_pos_y,
                                          (float)m_width, (float)m_height});

  m_pin_image->set_position(engine::rect2{(float)pin2_x, (float)pin2_y,
                                          (float)PIN_SIZE, (float)PIN_SIZE});
  m_second_pin_image->set_position(engine::rect2{
      (float)pin1_x, (float)pin1_y, (float)PIN_SIZE, (float)PIN_SIZE});
}

////////////////////////////////////////////////////////////////////////////////
// Controls2dInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

Controls2dInstrument::Controls2dInstrument(engine::RaylibDevice *device,
                                           float pos_x, int pos_y, int size)
    : m_device(device), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_size(size) {
  // Create the background image.
  m_controls_image = device->create_image2d(m_size, m_size);

  for (int x = 0; x < size; x++) {
    for (int y = 0; y < size; y++) {
      int half = size / 2;
      // Draw the axes;
      if ((x > half - 1 && x < half + 1) || (y > half - 1 && y < half + 1)) {
        m_controls_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
      }
      // Draw the gradient;
      else {
        float distance =
            float((x - half) * (x - half) + (y - half) * (y - half)) /
            (half * half);
        float intensity = distance > 1 ? 0 : 1 - distance;
        m_controls_image->set_pixel_color(
            x, y, engine::Color(0xA0, 0, 0, 150 * intensity));
      }
    }
  }

  m_pin_image = create_circular_pin_image(m_device, PIN_SIZE,
                                          engine::Color(0xff, 0xff, 0, 0));
  m_second_pin_image =
      create_plus_pin_image(m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0));
}

void Controls2dInstrument::update(float pin1_y, float pin1_x, float pin2_y,
                                  float pin2_x) {
  // Calculate the pin positions.
  int before_pin_x = m_pos_x + m_size / 2 + pin1_x * m_size / 2 - PIN_SIZE / 2;
  int before_pin_y = m_pos_y + m_size / 2 + pin1_y * m_size / 2 - PIN_SIZE / 2;
  int after_pin_x = m_pos_x + m_size / 2 + pin2_x * m_size / 2 - PIN_SIZE / 2;
  int after_pin_y = m_pos_y + m_size / 2 + pin2_y * m_size / 2 - PIN_SIZE / 2;

  m_controls_image->set_position(engine::rect2{(float)m_pos_x, (float)m_pos_y,
                                               (float)m_size, (float)m_size});

  m_pin_image->set_position(engine::rect2{(float)after_pin_x,
                                          (float)after_pin_y, (float)PIN_SIZE,
                                          (float)PIN_SIZE});

  m_second_pin_image->set_position(
      engine::rect2{(float)before_pin_x, (float)before_pin_y, (float)PIN_SIZE,
                    (float)PIN_SIZE});
}

////////////////////////////////////////////////////////////////////////////////
// CurvesInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

CurvesInstrument::CurvesInstrument(
    engine::RaylibDevice *device,
    std::vector<std::vector<ControllerCurve>> curve_groups,
    std::vector<engine::Color> curve_colors, float pos_x, int pos_y, int width,
    int height)
    : m_device(device), m_curve_groups(curve_groups), m_pos_x(POS_X(pos_x)),
      m_pos_y(POS_Y(pos_y)), m_width(width), m_height(height) {

  size_t num_curves_in_group = curve_groups[0].size();

  // Create the throttle-lift curves canvas.
  for (size_t curve_index = 0; curve_index < num_curves_in_group;
       curve_index++) {
    std::shared_ptr<engine::Image2D> curves_image =
        m_device->create_image2d(width, height);

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        // The grid:
        if (((x % CURVES_GRID_SIZE == 0) || (y % CURVES_GRID_SIZE == 0)) &&
            (x != 0) && (y != 0)) {
          curves_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0x40));
        } else {
          curves_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0x20));
        }
      }
    }
    for (size_t curve_group_index = 0; curve_group_index < curve_groups.size();
         curve_group_index++) {
      plot_curve(curve_groups[curve_group_index][curve_index], curves_image,
                 curve_colors[curve_group_index]);
    }
    m_curves_images.push_back(curves_image);
    m_pin_images.push_back(create_circular_pin_image(
        device, PIN_SIZE * 2, engine::Color(0xff, 0, 0, 0xff)));
  }

  // Create the curves vertical line.
  m_curves_vertical_line = m_device->create_image2d(3, height);
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < height; y++) {
      m_curves_vertical_line->set_pixel_color(x, y,
                                              engine::Color(0, 0, 0, 0x80));
    }
  }
}

void CurvesInstrument::update(unsigned long active_curve_index,
                              float x_stick_level,
                              std::vector<float> y_levels) {

  for (size_t curve_index = 0; curve_index < m_curves_images.size();
       curve_index++) {
    m_curves_images[curve_index]->set_visible(curve_index ==
                                              active_curve_index);
    m_curves_images[curve_index]->set_position(engine::rect2{
        (float)m_pos_x, (float)m_pos_y, (float)m_width, (float)m_height});
  }

  int vertical_line_x_offset = float(m_width) * (x_stick_level + 1) / 2;
  m_curves_vertical_line->set_position(
      engine::rect2{(float)(m_pos_x + vertical_line_x_offset), (float)m_pos_y,
                    3.0f, (float)m_height});
  float pin_size = PIN_SIZE * 0.8;
  for (size_t i = 0; i < y_levels.size(); i++) {
    float y_level = y_levels[i];
    int throttle_pin_x = vertical_line_x_offset - pin_size / 2;
    int throttle_pin_y = float(m_height) * (-y_level + 1) / 2 - pin_size / 2;
    m_pin_images[i]->set_position(engine::rect2{
        (float)(throttle_pin_x + m_pos_x), (float)(throttle_pin_y + m_pos_y),
        (float)pin_size, (float)pin_size});
  }
}

////////////////////////////////////////////////////////////////////////////////
//  SpeedometerInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

SpeedometerInstrument::SpeedometerInstrument(
    engine::RaylibDevice *device, float pos_x, int pos_y, float max_value,
    std::string headline, std::string value_template, std::string unit)
    : m_device(device), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_max_value(max_value), m_text_value_template(value_template) {

  // Create the main rotor indicator.
  m_speedometer_image =
      m_device->load_image2d("resources/media/speedometer3_meter.png");
  m_speedometer_hand2_image =
      m_device->load_image2d("resources/media/speedometer3_hand_yellow.png");
  m_speedometer_hand_image =
      m_device->load_image2d("resources/media/speedometer3_hand.png");

  float origin_x = float(MAIN_ROTOR_INDICATOR_HAND_WIDTH) * (45.7 / 91);
  float origin_y = float(MAIN_ROTOR_INDICATOR_HAND_HEIGHT) * (286.6 / 329);
  m_speedometer_hand_image->set_origin({origin_x, origin_y});
  m_speedometer_hand2_image->set_origin({origin_x, origin_y});
  m_text_headline =
      m_device->create_text2d(headline, 30, engine::Color(0, 0, 0, 0xff),
                              engine::TextAlignment::CENTER);
  m_text_value = m_device->create_text2d("", 70, engine::Color(0, 0, 0, 0xff),
                                         engine::TextAlignment::CENTER);
  m_text_unit = m_device->create_text2d(unit, 20, engine::Color(0, 0, 0, 0xff),
                                        engine::TextAlignment::CENTER);
}

void SpeedometerInstrument::update(float value1, float value2) {
  m_speedometer_image->set_position(engine::rect2{
      (float)m_pos_x, (float)m_pos_y, (float)(MAIN_ROTOR_INDICATOR_WIDTH),
      (float)(MAIN_ROTOR_INDICATOR_HEIGHT)});

  int hand_x = m_pos_x + MAIN_ROTOR_INDICATOR_WIDTH / 2;
  int hand_y = m_pos_y + MAIN_ROTOR_INDICATOR_HEIGHT -
               MAIN_ROTOR_INDICATOR_HAND_WIDTH / 4;
  float size_x = float(MAIN_ROTOR_INDICATOR_HAND_WIDTH) / 91;
  float size_y = float(MAIN_ROTOR_INDICATOR_HAND_HEIGHT) / 329;
  m_speedometer_hand2_image->set_position(engine::rect2{
      (float)hand_x, (float)hand_y, (float)(MAIN_ROTOR_INDICATOR_HAND_WIDTH),
      (float)(MAIN_ROTOR_INDICATOR_HAND_HEIGHT)});
  m_speedometer_hand2_image->set_rotation(180. * value2 / m_max_value - 90.);
  m_speedometer_hand_image->set_position(engine::rect2{
      (float)hand_x, (float)hand_y, (float)(MAIN_ROTOR_INDICATOR_HAND_WIDTH),
      (float)(MAIN_ROTOR_INDICATOR_HAND_HEIGHT)});
  m_speedometer_hand_image->set_rotation(180. * value1 / m_max_value - 90.);

  // The texts.
  float text_pos_x = m_pos_x + MAIN_ROTOR_INDICATOR_WIDTH / 2;
  m_text_headline->set_position(engine::vec2{(float)text_pos_x, (float)m_pos_y - 10});
  char *output_text = new char[m_text_value_template.size() + 100];
  sprintf(output_text, m_text_value_template.c_str(), value1, value2);
  m_text_value->set_text(std::string(output_text));
  m_text_value->set_position(engine::vec2{(float)text_pos_x, (float)m_pos_y + 70});
  m_text_unit->set_position(engine::vec2{(float)text_pos_x, (float)m_pos_y + MAIN_ROTOR_INDICATOR_HEIGHT});
}

////////////////////////////////////////////////////////////////////////////////
//  HeliDashboard class implementation
////////////////////////////////////////////////////////////////////////////////

HeliDashboard::HeliDashboard(engine::RaylibDevice *device,
                             std::vector<ControllerCurve> throttle_curves,
                             std::vector<ControllerCurve> lift_curves,
                             float max_rps)
    : m_device(device), m_dashboard_background(device->create_image2d(10, 10)),
      m_yaw_instrument(device, YAW_VIEW_POS_X, YAW_VIEW_POS_Y, CONTROLS_SIZE,
                       YAW_VIEW_HEIGHT),
      m_pitch_roll_instrument(device, CONTROLS_POS_X, CONTROLS_POS_Y,
                              CONTROLS_SIZE),
      m_curves_instrument(device, {throttle_curves, lift_curves},
                          {engine::Color(0x8c, 0x2a, 0xde, 0xb0),
                           engine::Color(0x42, 0x9e, 0xff, 0xb0)},
                          CURVES_IMAGE_POS_X, CURVES_IMAGE_POS_Y,
                          CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT),
      m_main_rotor_instrument(device, MAIN_ROTOR_INDICATOR_X,
                              MAIN_ROTOR_INDICATOR_Y, max_rps, "Main Rotor",
                              "%1.1f", "RPS") {

  // Create the dashboard background
  for (int x = 0; x < 10; x++) {
    for (int y = 0; y < 10; y++) {
      m_dashboard_background->set_pixel_color(x, y,
                                              engine::Color(0, 0, 0, 0x30));
    }
  }
  m_dashboard_background->set_position({0, POS_Y(BACKGROUND_HEIGHT),
                                        (float)device->get_screen_width(),
                                        (float)device->get_screen_height()});
}

void HeliDashboard::update_ui(const Controls::Telemetry &controls_telemetry,
                              const BaseHeli::Telemetry &telemetry) {
  ControlsInput user_input = controls_telemetry.user_input;
  ServoData before_controller = controls_telemetry.before_controller;
  ServoData after_controller = controls_telemetry.after_controller;

  m_pitch_roll_instrument.update(before_controller[1], before_controller[2],
                                 after_controller[1], after_controller[2]);

  m_yaw_instrument.update(before_controller[3], after_controller[3]);

  m_curves_instrument.update(user_input.active_curve_index,
                             user_input.throttle_stick,
                             {after_controller[0], after_controller[4]});

  // Draw the main rotor indicator.
  m_main_rotor_instrument.update(telemetry.rps, telemetry.target_rps);
}

////////////////////////////////////////////////////////////////////////////////
//  PlaneDashboard class implementation
////////////////////////////////////////////////////////////////////////////////

PlaneDashboard::PlaneDashboard(engine::RaylibDevice *device, float max_speed)
    : m_device(device), m_dashboard_background(device->create_image2d(10, 10)),
      m_yaw_instrument(device, YAW_VIEW_POS_X, YAW_VIEW_POS_Y, CONTROLS_SIZE,
                       YAW_VIEW_HEIGHT),
      m_pitch_roll_instrument(device, CONTROLS_POS_X, CONTROLS_POS_Y,
                              CONTROLS_SIZE),
      m_throttle_instrument(device, CONTROLS_POS_X * 0.8, CONTROLS_POS_Y,
                            CONTROLS_SIZE / 4, CONTROLS_SIZE),
      m_airspeed_instrument(device, MAIN_ROTOR_INDICATOR_X,
                            MAIN_ROTOR_INDICATOR_Y, max_speed, "Airspeed",
                            "%.1f", "m/s") {

  // Create the dashboard background
  for (int x = 0; x < 10; x++) {
    for (int y = 0; y < 10; y++) {
      m_dashboard_background->set_pixel_color(x, y,
                                              engine::Color(0, 0, 0, 0x30));
    }
  }
  m_dashboard_background->set_position({0, POS_Y(BACKGROUND_HEIGHT),
                                        (float)m_device->get_screen_width(),
                                        (float)m_device->get_screen_height()});
}

void PlaneDashboard::update_ui(const Controls::Telemetry &controls_telemetry,
                               const BaseHeli::Telemetry &telemetry) {
  // ControlsInput user_input = controls_telemetry.user_input;
  ServoData before_controller = controls_telemetry.before_controller;
  ServoData after_controller = controls_telemetry.after_controller;

  m_pitch_roll_instrument.update(before_controller[1], before_controller[2],
                                 after_controller[1], after_controller[2]);

  m_yaw_instrument.update(before_controller[3], after_controller[3]);
  m_throttle_instrument.update(before_controller[0], after_controller[0]);
  m_airspeed_instrument.update(telemetry.airspeed,
                               telemetry.velocity_magnitude);
}
