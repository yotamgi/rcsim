#include "dashboard.h"
#include <cmath>

#include <format>
#include <iomanip>
#include <iostream>
#include <sstream>

using Origin = engine::Rect2D::Origin;

const int BACKGROUND_HEIGHT = 220;
const engine::Color BACKGROUND_COLOR = engine::Color(0, 0, 0, 0x30);

const int CONTROLS_SIZE = 160;
const int YAW_VIEW_HEIGHT = 25;
const int PIN_SIZE = 15;
const float CONTROLS_POS_X = 0.15f;
const float CONTROLS_POS_Y = 0.5f;
const float YAW_VIEW_POS_Y = 0.92f;
const float YAW_VIEW_POS_X = CONTROLS_POS_X;

const float CURVES_IMAGE_POS_X = 0.8;
const int CURVES_IMAGE_HEIGHT = 150;
const int CURVES_IMAGE_WIDTH = 275;
const int CURVES_GRID_SIZE = 25;
const int CURVE_THICKNESS = 3.5;

const float SPEEDOMETER_X = 0.4;
const int SPEEDOMETER_Y = 200;
const int SPEEDOMETER_WIDTH = 300;
const int SPEEDOMETER_HEIGHT = 170;
const int SPEEDOMETER_HAND_WIDTH = 91 * SPEEDOMETER_WIDTH / 651;
const int SPEEDOMETER_HAND_HEIGHT = 329 * SPEEDOMETER_HEIGHT / 394;

const int TEXT_HEIGHT = 60;
const int TEXT_WIDTH = 60 * 8;

static float curve_x_to_value(int x) {
  return (float(x) / CURVES_IMAGE_WIDTH) * 2. - 1.;
}
static float curve_value_to_y(float value) {
  return float(CURVES_IMAGE_HEIGHT) * ((value + 1.) / 2.);
}

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
                          const engine::Color &color,
                          std::shared_ptr<engine::Drawable2D> parent) {
  std::shared_ptr<engine::Image2D> pin_image =
      device->create_image2d(size, size, parent);

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
                      const engine::Color &color,
                      std::shared_ptr<engine::Drawable2D> parent) {

  std::shared_ptr<engine::Image2D> pin_image =
      device->create_image2d(size, size, parent);

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
    engine::RaylibDevice *device, engine::Coord2D pos_x, engine::Coord2D pos_y,
    int width, int height, std::shared_ptr<engine::Drawable2D> parent)
    : m_device(device), m_pos_x(pos_x), m_pos_y(pos_y), m_width(width),
      m_height(height) {

  // Create the yaw control image.
  m_control_image = m_device->create_image2d(width, height, parent);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int half_x = width / 2;
      int half_y = height / 2;
      int x_fade = 0.25 * width;
      // Draw the axes;
      if ((x > half_x - 1 && x < half_x + 1) ||
          (y > half_y - 1 && y < half_y + 1)) {
        m_control_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
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
        m_control_image->set_pixel_color(
            x, y, engine::Color(0xA0, 0, 0, 150 * intensity));
      }
    }
  }

  m_pin_image = create_circular_pin_image(
      m_device, 100, engine::Color(0xff, 0xff, 0, 0), m_control_image);
  m_second_pin_image = create_plus_pin_image(
      m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0), m_control_image);

  // The yaw image is static, so it's position is set in the constructor.
  m_control_image->set_position(
      {m_pos_x, m_pos_y, m_width, m_height, Origin::MID, Origin::MID});
}

void HorizontalControlsInstrument::update(float pin1, float pin2) {
  // Draw the yaw controls view.
  m_pin_image->set_position(
      {pin2 + 0.5f, 0.5f, int(PIN_SIZE), 1.0f, Origin::MID, Origin::MID});
  m_second_pin_image->set_position(
      {pin1 + 0.5f, 0.5f, int(PIN_SIZE), 1.0f, Origin::MID, Origin::MID});
}

////////////////////////////////////////////////////////////////////////////////
// VerticalControlsInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

VerticalControlsInstrument::VerticalControlsInstrument(
    engine::RaylibDevice *device, engine::Coord2D pos_x, engine::Coord2D pos_y,
    int width, int height, std::shared_ptr<engine::Drawable2D> parent)
    : m_device(device), m_pos_x(pos_x), m_pos_y(pos_y), m_width(width),
      m_height(height) {

  // Create the yaw control image.
  m_control_image = device->create_image2d(width, height, parent);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int half_x = width / 2;
      int half_y = height / 2;
      int x_fade = 0.25 * width;
      // Draw the axes;
      if ((x > half_x - 1 && x < half_x + 1) ||
          (y > half_y - 1 && y < half_y + 1)) {
        m_control_image->set_pixel_color(x, y, engine::Color(0, 0, 0, 0xff));
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
        m_control_image->set_pixel_color(
            x, y, engine::Color(0xA0, 0, 0, 150 * intensity));
      }
    }
  }

  m_pin_image = create_circular_pin_image(
      m_device, 100, engine::Color(0xff, 0xff, 0, 0), m_control_image);
  m_second_pin_image = create_plus_pin_image(
      m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0), m_control_image);

  m_control_image->set_position(engine::Rect2D{
      m_pos_x, m_pos_y, m_width, m_height, Origin::MID, Origin::MID});
}

void VerticalControlsInstrument::update(float pin1, float pin2) {
  // Draw the yaw controls view.
  m_pin_image->set_position(engine::Rect2D{0.5f, 0.5f - pin2 / 2, 1.0f,
                                           PIN_SIZE, Origin::MID, Origin::MID});
  m_second_pin_image->set_position(engine::Rect2D{
      0.5f, 0.5f - pin1 / 2.0f, 1.0f, PIN_SIZE, Origin::MID, Origin::MID});
}

////////////////////////////////////////////////////////////////////////////////
// Controls2dInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

Controls2dInstrument::Controls2dInstrument(
    engine::RaylibDevice *device, engine::Coord2D pos_x, engine::Coord2D pos_y,
    int size, std::shared_ptr<engine::Drawable2D> parent)
    : m_device(device), m_pos_x(pos_x), m_pos_y(pos_y), m_size(size) {
  // Create the background image.
  m_controls_image = device->create_image2d(m_size, m_size, parent);

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

  m_controls_image->set_position(engine::Rect2D{
      m_pos_x, m_pos_y, m_size, m_size, Origin::MID, Origin::MID});

  m_pin_image = create_circular_pin_image(
      m_device, 100, engine::Color(0xff, 0xff, 0, 0), m_controls_image);
  m_second_pin_image = create_plus_pin_image(
      m_device, PIN_SIZE, engine::Color(0xff, 0, 0, 0), m_controls_image);
}

void Controls2dInstrument::update(float pin1_y, float pin1_x, float pin2_y,
                                  float pin2_x) {
  m_pin_image->set_position(engine::Rect2D{0.5f + pin1_x, 0.5f + pin1_y,
                                           PIN_SIZE, PIN_SIZE, Origin::MID,
                                           Origin::MID});

  m_second_pin_image->set_position(engine::Rect2D{0.5f + pin2_x, 0.5f + pin2_y,
                                                  PIN_SIZE, PIN_SIZE,
                                                  Origin::MID, Origin::MID});
}

////////////////////////////////////////////////////////////////////////////////
// CurvesInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

CurvesInstrument::CurvesInstrument(
    engine::RaylibDevice *device,
    std::vector<std::vector<ControllerCurve>> curve_groups,
    std::vector<engine::Color> curve_colors, engine::Coord2D pos_x,
    engine::Coord2D pos_y, int width, int height,
    std::vector<std::string> curve_group_names,
    std::vector<std::string> curve_names,
    std::shared_ptr<engine::Drawable2D> parent)
    : m_device(device), m_curve_groups(curve_groups),
      m_curve_names(curve_names), m_pos_x(pos_x), m_pos_y(pos_y),
      m_width(width), m_height(height) {

  size_t num_curves_in_group = curve_groups[0].size();
  m_main_node = m_device->create_square2d(engine::Color(0, 0, 0, 0), parent);
  m_main_node->set_position(
      engine::Rect2D{m_pos_x, pos_y, width, height, Origin::MID, Origin::MID});

  // Create the throttle-lift curves canvas.
  for (size_t curve_index = 0; curve_index < num_curves_in_group;
       curve_index++) {
    std::shared_ptr<engine::Image2D> curves_image =
        m_device->create_image2d(width, height, m_main_node);
    curves_image->set_position(engine::Rect2D{0.0f, 0.0f, 1.0f, 1.0f});

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
        device, PIN_SIZE * 2, engine::Color(0xff, 0, 0, 0xff), m_main_node));
  }

  // Create the curves vertical line.
  m_curves_vertical_line =
      m_device->create_square2d(engine::Color(0, 0, 0, 0x80), m_main_node);

  // Create the texts.
  for (std::string curve_group_name : curve_group_names) {
    m_headline_text.push_back(m_device->create_text2d(
        "Throttle-Lift Curve - " + curve_group_name,
        engine::Text2D::FontOptions{20, engine::Color(0, 0, 0, 0xff)},
        m_main_node));
    m_headline_text.back()->set_position(0.5f, -20, Origin::MID, Origin::MID);
  }

  for (int curve_index = 0; curve_index < num_curves_in_group; curve_index++) {
    m_curve_texts.push_back(m_device->create_text2d(
        curve_names[curve_index],
        engine::Text2D::FontOptions{20, curve_colors[curve_index]},
        m_main_node));
    m_curve_texts.back()->set_position(0.5f * curve_index, 1.1f, Origin::MIN,
                                       Origin::MIN);
  }
}

void CurvesInstrument::update(unsigned long active_curve_index,
                              float x_stick_level,
                              std::vector<float> y_levels) {

  for (size_t curve_index = 0; curve_index < m_curves_images.size();
       curve_index++) {
    m_curves_images[curve_index]->set_visible(curve_index ==
                                              active_curve_index);
  }

  m_curves_vertical_line->set_position(engine::Rect2D{
      0.5f + x_stick_level / 2.0f, 0, 1, 1.0f, Origin::MID, Origin::MIN});

  int pin_size = PIN_SIZE * 0.8;
  for (size_t i = 0; i < y_levels.size(); i++) {
    m_pin_images[i]->set_position(
        engine::Rect2D{0.5f + x_stick_level / 2, 0.5f - y_levels[i] / 2,
                       PIN_SIZE, PIN_SIZE, Origin::MID, Origin::MID});
  }

  // Update the texts.
  for (size_t i = 0; i < m_headline_text.size(); i++) {
    m_headline_text[i]->set_visible(i == active_curve_index);
  }

  float delta_x = m_width / m_curve_texts.size();
  for (size_t i = 0; i < m_curve_texts.size(); i++) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0) << m_curve_names[i] << ": "
       << y_levels[i] * 100 << "%";
    m_curve_texts[i]->set_text(ss.str());
  }
}

////////////////////////////////////////////////////////////////////////////////
//  SpeedometerInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

SpeedometerInstrument::SpeedometerInstrument(
    engine::RaylibDevice *device, engine::Coord2D pos_x, engine::Coord2D pos_y,
    float max_value, std::string headline, std::string value_template,
    std::string unit, std::shared_ptr<engine::Drawable2D> parent)
    : m_device(device), m_pos_x(pos_x), m_pos_y(pos_y), m_max_value(max_value),
      m_text_value_template(value_template) {

  // Create the main rotor indicator.
  m_speedometer_image =
      m_device->load_image2d("resources/media/speedometer3_meter.png", parent);
  m_speedometer_hand2_image = m_device->load_image2d(
      "resources/media/speedometer3_hand_yellow.png", m_speedometer_image);
  m_speedometer_hand_image = m_device->load_image2d(
      "resources/media/speedometer3_hand.png", m_speedometer_image);

  float origin_x = float(SPEEDOMETER_HAND_WIDTH) * (45.7 / 91);
  float origin_y = float(SPEEDOMETER_HAND_HEIGHT) * (286.6 / 329);
  m_speedometer_hand_image->set_rotation_axis({origin_x, origin_y});
  m_speedometer_hand2_image->set_rotation_axis({origin_x, origin_y});
  m_text_headline = m_device->create_text2d(
      headline, engine::Text2D::FontOptions{30}, m_speedometer_image);
  m_text_value = m_device->create_text2d("", engine::Text2D::FontOptions{70},
                                         m_speedometer_image);
  m_text_unit = m_device->create_text2d(unit, engine::Text2D::FontOptions{20},
                                        m_speedometer_image);

  m_speedometer_image->set_position(
      engine::Rect2D{m_pos_x, m_pos_y, SPEEDOMETER_WIDTH, SPEEDOMETER_HEIGHT,
                     Origin::MID, Origin::MID});
  float ratio = (float)SPEEDOMETER_WIDTH / 651;
  m_speedometer_hand_image->set_position(
      engine::Rect2D{0.5f, 1.0f, SPEEDOMETER_HAND_WIDTH,
                     SPEEDOMETER_HAND_HEIGHT, Origin::MID, Origin::MAX});
  m_speedometer_hand2_image->set_position(
      engine::Rect2D{0.5f, 1.0f, SPEEDOMETER_HAND_WIDTH,
                     SPEEDOMETER_HAND_HEIGHT, Origin::MID, Origin::MAX});
  m_text_headline->set_position(0.5f, 15, Origin::MID, Origin::MAX);
  m_text_value->set_position(0.5f, 0.65f, Origin::MID, Origin::MID);
  m_text_unit->set_position(0.72f, 0.92f, Origin::MID, Origin::MID);
}

void SpeedometerInstrument::update(float value1, float value2) {
  m_speedometer_hand_image->set_rotation(180. * value1 / m_max_value - 90.);
  m_speedometer_hand2_image->set_rotation(180. * value2 / m_max_value - 90.);

  // The texts.
  char *output_text = new char[m_text_value_template.size() + 100];
  sprintf(output_text, m_text_value_template.c_str(), value1, value2);
  m_text_value->set_text(std::string(output_text));
}

////////////////////////////////////////////////////////////////////////////////
//  HeliDashboard class implementation
////////////////////////////////////////////////////////////////////////////////

HeliDashboard::HeliDashboard(engine::RaylibDevice *device,
                             std::vector<ControllerCurve> throttle_curves,
                             std::vector<ControllerCurve> lift_curves,
                             float max_rps)
    : m_device(device), m_dashboard(device->create_square2d(BACKGROUND_COLOR)),
      m_yaw_instrument(device, YAW_VIEW_POS_X, YAW_VIEW_POS_Y, CONTROLS_SIZE,
                       YAW_VIEW_HEIGHT, m_dashboard),
      m_pitch_roll_instrument(device, CONTROLS_POS_X, CONTROLS_POS_Y,
                              CONTROLS_SIZE, m_dashboard),
      m_curves_instrument(device, {throttle_curves, lift_curves},
                          {engine::Color(0x8c, 0x2a, 0xde, 0xb0),
                           engine::Color(0x42 / 2, 0x9e / 2, 0xff / 2, 0xb0)},
                          CURVES_IMAGE_POS_X, 0.5f, CURVES_IMAGE_WIDTH,
                          CURVES_IMAGE_HEIGHT, {"Normal", "Idle-up"},
                          {"Thr", "Lift"}, m_dashboard),
      m_main_rotor_instrument(device, 0.5f, 0.5f, max_rps, "Main Rotor",
                              "%2.1f", "RPS", m_dashboard),
      m_input_text(device->create_text2d(
          "Stick Input",
          engine::Text2D::FontOptions{20, engine::Color(0, 0, 0, 0xff)},
          m_dashboard)) {
  m_dashboard->set_position(engine::Rect2D{0, 0.75f, 1.0f, 0.25f});
  m_input_text->set_position(20, 0.1f);
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
    : m_device(device),
      m_dashboard(device->create_square2d(engine::Color(0, 0, 0, 0x30))),
      m_yaw_instrument(device, YAW_VIEW_POS_X, YAW_VIEW_POS_Y, CONTROLS_SIZE,
                       YAW_VIEW_HEIGHT, m_dashboard),
      m_pitch_roll_instrument(device, CONTROLS_POS_X, 0.5f, CONTROLS_SIZE,
                              m_dashboard),
      m_throttle_instrument(device, CONTROLS_POS_X - 0.07f, 0.5f,
                            YAW_VIEW_HEIGHT, CONTROLS_SIZE, m_dashboard),
      m_airspeed_instrument(device, 0.5f, 0.5f, max_speed, "Airspeed", "%2.1f",
                            "m/s", m_dashboard) {
  m_dashboard->set_position(engine::Rect2D{0, 0.75f, 1.0f, 0.25f});
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
