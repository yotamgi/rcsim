#include "dashboard.h"
#include <cmath>

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

static void draw2DImageWithRotation(
    irr::video::IVideoDriver *driver, irr::video::ITexture *texture,
    irr::core::rect<irr::s32> sourceRect,
    irr::core::position2d<irr::s32> position,
    irr::core::position2d<irr::s32> rotationPoint, irr::f32 rotation,
    irr::core::vector2df scale, irr::video::SColor color) {
  irr::video::SMaterial material;

  // Store and clear the projection matrix
  irr::core::matrix4 oldProjMat =
      driver->getTransform(irr::video::ETS_PROJECTION);
  driver->setTransform(irr::video::ETS_PROJECTION, irr::core::matrix4());

  // Store and clear the view matrix
  irr::core::matrix4 oldViewMat = driver->getTransform(irr::video::ETS_VIEW);
  driver->setTransform(irr::video::ETS_VIEW, irr::core::matrix4());

  // Store and clear the world matrix
  irr::core::matrix4 oldWorldMat = driver->getTransform(irr::video::ETS_WORLD);
  driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

  // Find the positions of corners
  irr::core::vector2df corner[4];

  corner[0] = irr::core::vector2df(position.X, position.Y);
  corner[1] = irr::core::vector2df(position.X + sourceRect.getWidth() * scale.X,
                                   position.Y);
  corner[2] = irr::core::vector2df(
      position.X, position.Y + sourceRect.getHeight() * scale.Y);
  corner[3] =
      irr::core::vector2df(position.X + sourceRect.getWidth() * scale.X,
                           position.Y + sourceRect.getHeight() * scale.Y);

  // Rotate corners
  if (rotation != 0.0f) {
    for (int x = 0; x < 4; x++) {
      corner[x].rotateBy(
          rotation, irr::core::vector2df(rotationPoint.X, rotationPoint.Y));
    }
  }

  // Find the uv coordinates of the sourceRect
  irr::core::vector2df uvCorner[4];
  uvCorner[0] = irr::core::vector2df(sourceRect.UpperLeftCorner.X,
                                     sourceRect.UpperLeftCorner.Y);
  uvCorner[1] = irr::core::vector2df(sourceRect.LowerRightCorner.X,
                                     sourceRect.UpperLeftCorner.Y);
  uvCorner[2] = irr::core::vector2df(sourceRect.UpperLeftCorner.X,
                                     sourceRect.LowerRightCorner.Y);
  uvCorner[3] = irr::core::vector2df(sourceRect.LowerRightCorner.X,
                                     sourceRect.LowerRightCorner.Y);
  for (int x = 0; x < 4; x++) {
    float uvX = uvCorner[x].X / (float)texture->getSize().Width;
    float uvY = uvCorner[x].Y / (float)texture->getSize().Height;
    uvCorner[x] = irr::core::vector2df(uvX, uvY);
  }

  // Vertices for the image
  irr::video::S3DVertex vertices[4];
  irr::u16 indices[6] = {0, 1, 2, 3, 2, 1};

  // Convert pixels to world coordinates
  float screenWidth = driver->getScreenSize().Width;
  float screenHeight = driver->getScreenSize().Height;
  for (int x = 0; x < 4; x++) {
    float screenPosX = ((corner[x].X / screenWidth) - 0.5f) * 2.0f;
    float screenPosY = ((corner[x].Y / screenHeight) - 0.5f) * -2.0f;
    vertices[x].Pos = irr::core::vector3df(screenPosX, screenPosY, 1);
    vertices[x].TCoords = uvCorner[x];
    vertices[x].Color = color;
  }

  material.Lighting = false;
  material.ZWriteEnable = false;
  material.ZBuffer = false;
  material.TextureLayer[0].Texture = texture;
  material.MaterialTypeParam = irr::video::pack_textureBlendFunc(
      irr::video::EBF_SRC_ALPHA, irr::video::EBF_ONE_MINUS_SRC_ALPHA,
      irr::video::EMFN_MODULATE_1X,
      irr::video::EAS_TEXTURE | irr::video::EAS_VERTEX_COLOR);

  material.MaterialType = irr::video::EMT_ONETEXTURE_BLEND;

  driver->setMaterial(material);
  driver->drawIndexedTriangleList(&vertices[0], 4, &indices[0], 2);

  // Restore projection and view matrices
  driver->setTransform(irr::video::ETS_PROJECTION, oldProjMat);
  driver->setTransform(irr::video::ETS_VIEW, oldViewMat);
  driver->setTransform(irr::video::ETS_WORLD, oldWorldMat);
}

static void plot_curve(const ControllerCurve &curve, irr::video::IImage *image,
                       const irr::video::SColor &color,
                       int image_width = CURVES_IMAGE_WIDTH,
                       int image_height = CURVES_IMAGE_HEIGHT) {
  float curve_y_value;

  for (int x = 0; x < image_width; x++) {
    float in_value = curve_x_to_value(x);
    float new_curve_y_value = curve_value_to_y(-curve.translate(in_value));
    float grad = (x == 0) ? 0 : (new_curve_y_value - curve_y_value);
    curve_y_value = new_curve_y_value;

    float grad_angle = irr::core::PI / 2 - std::atan(grad);
    float thickness_squared =
        std::pow(CURVE_THICKNESS / std::sin(grad_angle), 3);

    for (int y = 0; y < image_height; y++) {
      float distance = std::abs(float(y) - curve_y_value);

      float value =
          (thickness_squared - std::pow(distance, 3)) / thickness_squared;
      value = (value > 0) ? value : 0;
      irr::video::SColor prev_color = image->getPixel(x, y);
      int new_alpha = std::max(prev_color.getAlpha(),
                               (unsigned int)(value * color.getAlpha()));
      int new_red = value * color.getRed() + (1 - value) * prev_color.getRed();
      int new_green =
          value * color.getGreen() + (1 - value) * prev_color.getGreen();
      int new_blue =
          value * color.getBlue() + (1 - value) * prev_color.getBlue();
      image->setPixel(
          x, y, irr::video::SColor(new_alpha, new_red, new_green, new_blue));
    }
  }
}

Dashboard::Dashboard(irr::video::IVideoDriver *driver,
                     std::vector<ControllerCurve> throttle_curves,
                     std::vector<ControllerCurve> lift_curves, float max_rps)
    : m_throttle_curves(throttle_curves), m_lift_curves(lift_curves),
      m_max_rps(max_rps) {

  // Create the dashboard background
  irr::video::IImage *background_image = driver->createImage(
      irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(10, 10));
  for (int x = 0; x < 10; x++) {
    for (int y = 0; y < 10; y++) {
      background_image->setPixel(x, y, irr::video::SColor(0x30, 0, 0, 0));
    }
  }
  m_dashboard_background = driver->addTexture(
      irr::io::path("dashboard_background"), background_image);

  // Create the pitch-roll controls image.
  irr::video::IImage *controls_image = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(CONTROLS_SIZE, CONTROLS_SIZE));

  for (int x = 0; x < CONTROLS_SIZE; x++) {
    for (int y = 0; y < CONTROLS_SIZE; y++) {
      int half = CONTROLS_SIZE / 2;
      // Draw the axes;
      if ((x > half - 1 && x < half + 1) || (y > half - 1 && y < half + 1)) {
        controls_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
      }
      // Draw the gradient;
      else {
        float distance =
            float((x - half) * (x - half) + (y - half) * (y - half)) /
            (half * half);
        float intensity = distance > 1 ? 0 : 1 - distance;
        controls_image->setPixel(
            x, y, irr::video::SColor(150 * intensity, 0xA0, 0, 0));
      }
    }
  }

  m_controls_image =
      driver->addTexture(irr::io::path("controls_image"), controls_image);

  // Create the yaw control image.
  irr::video::IImage *yaw_image = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(CONTROLS_SIZE, YAW_VIEW_HEIGHT));

  for (int x = 0; x < CONTROLS_SIZE; x++) {
    for (int y = 0; y < YAW_VIEW_HEIGHT; y++) {
      int half_x = CONTROLS_SIZE / 2;
      int half_y = YAW_VIEW_HEIGHT / 2;
      int x_fade = 0.25 * CONTROLS_SIZE;
      // Draw the axes;
      if ((x > half_x - 1 && x < half_x + 1) ||
          (y > half_y - 1 && y < half_y + 1)) {
        yaw_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
      }
      // Draw the gradient;
      else {
        float distance = float(std::abs(y - half_y)) / half_y;
        float intensity = 1 - distance;
        if (x < x_fade) {
          intensity *= 1 - std::pow(float(x_fade - x) / x_fade, 2);
        } else if (x > CONTROLS_SIZE - x_fade) {
          intensity *=
              1 - std::pow(float(x - (CONTROLS_SIZE - x_fade)) / x_fade, 2);
        }
        yaw_image->setPixel(x, y,
                            irr::video::SColor(150 * intensity, 0xA0, 0, 0));
      }
    }
  }

  m_yaw_image = driver->addTexture(irr::io::path("yaw_image"), yaw_image);

  // Create the pin image;
  irr::video::IImage *pin_image = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(PIN_SIZE, PIN_SIZE));

  for (int x = 0; x < PIN_SIZE; x++) {
    for (int y = 0; y < PIN_SIZE; y++) {
      int half = PIN_SIZE / 2;
      float distance =
          std::pow(float((x - half) * (x - half) + (y - half) * (y - half)) /
                       (half * half),
                   3);
      float intensity = distance > 1 ? 0 : 1 - distance;
      pin_image->setPixel(x, y,
                          irr::video::SColor(150 * intensity, 0xff, 0xff, 0));
    }
  }
  m_pin_image = driver->addTexture(irr::io::path("pin_image"), pin_image);

  // Create the second pin image.
  irr::video::IImage *second_pin_image = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(PIN_SIZE, PIN_SIZE));

  for (int x = 0; x < PIN_SIZE; x++) {
    for (int y = 0; y < PIN_SIZE; y++) {
      int half = PIN_SIZE / 2;
      if ((x < half + 2 && x > half - 2) || (y < half + 2 && y > half - 2)) {
        second_pin_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
      } else {
        second_pin_image->setPixel(x, y, irr::video::SColor(0, 0, 0, 0));
      }
    }
  }
  m_second_pin_image =
      driver->addTexture(irr::io::path("second_pin_image"), second_pin_image);

  m_driver = driver;

  // Create the throttle-lift curves canvas.
  for (size_t curve_index = 0; curve_index < m_throttle_curves.size();
       curve_index++) {
    irr::video::IImage *curves_image = driver->createImage(
        irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(
                                      CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT));

    for (int x = 0; x < CURVES_IMAGE_WIDTH; x++) {
      for (int y = 0; y < CURVES_IMAGE_HEIGHT; y++) {
        // The grid:
        if (((x % CURVES_GRID_SIZE == 0) || (y % CURVES_GRID_SIZE == 0)) &&
            (x != 0) && (y != 0)) {
          curves_image->setPixel(x, y, irr::video::SColor(0x40, 0, 0, 0));
        } else {
          curves_image->setPixel(x, y, irr::video::SColor(0x20, 0, 0, 0));
        }
      }
    }
    plot_curve(m_throttle_curves[curve_index], curves_image,
               irr::video::SColor(0xb0, 0x8c, 0x2a, 0xde));
    plot_curve(m_lift_curves[curve_index], curves_image,
               irr::video::SColor(0xb0, 0x42, 0x9e, 0xff));

    std::stringstream ss;
    ss << "curve_images_" << curve_index;
    m_curves_images.push_back(
        driver->addTexture(ss.str().c_str(), curves_image));
  }

  // Create the curves vertical line.
  irr::video::IImage *curves_vertical_line = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(3, CURVES_IMAGE_HEIGHT));
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < CURVES_IMAGE_HEIGHT; y++) {
      curves_vertical_line->setPixel(x, y, irr::video::SColor(0x80, 0, 0, 0));
    }
  }
  m_curves_vertical_line = driver->addTexture(
      irr::io::path("curves_vertical_line"), curves_vertical_line);

  // Create the main rotor indicator.
  m_main_rotor_indicator_image =
      driver->getTexture("media/speedometer3_meter.png");
  m_main_rotor_indicator_hand_image =
      driver->getTexture("media/speedometer3_hand.png");
  m_main_rotor_indicator_target_hand_image =
      driver->getTexture("media/speedometer3_hand_yellow.png");
  m_throttle_text = driver->getTexture("media/throttle_text.png");
  m_throttle_hold_text = driver->getTexture("media/throttle_hold_text.png");
  m_curves_text = driver->getTexture("media/curves_text.png");
  m_switch_curves_text = driver->getTexture("media/switch_curves_text.png");
  m_controls_and_gyro_view_text =
      driver->getTexture("media/controls_and_gyro_view.png");

  m_driver = driver;
}

static int pos_y(irr::video::IVideoDriver *driver, int pos_y) {
  return driver->getScreenSize().Height - pos_y;
}

static int pos_x_ratio(irr::video::IVideoDriver *driver, float pos_x_ratio) {
  return driver->getScreenSize().Width * pos_x_ratio;
}

#define POS_Y(y) pos_y(m_driver, (y))
#define POS_X(x) pos_x_ratio(m_driver, (x))

void Dashboard::update_ui(const Controls::Telemetry &controls_telemetry,
                          const BaseHeli::Telemetry &telemetry) {
  ControlsInput user_input = controls_telemetry.user_input;
  ServoData before_controller = controls_telemetry.before_controller;
  ServoData after_controller = controls_telemetry.after_controller;
  int active_curve_index = controls_telemetry.active_curve_index;

  // Draw the background
  m_driver->draw2DImage(m_dashboard_background,
                        irr::core::rect<int>(POS_X(0), POS_Y(BACKGROUND_HEIGHT),
                                             POS_X(1), POS_Y(0)),
                        irr::core::rect<int>(0, 0, 10, 10), NULL, NULL, true);

  // Draw the controls test
  m_driver->draw2DImage(
      m_controls_and_gyro_view_text,
      irr::core::rect<int>(POS_X(CONTROLS_POS_X) - TEXT_WIDTH / 2,
                           POS_Y(CONTROLS_POS_Y) - 10,
                           POS_X(CONTROLS_POS_X) + TEXT_WIDTH / 2,
                           POS_Y(CONTROLS_POS_Y) + TEXT_HEIGHT - 10),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);

  // Draw the pitch-roll control view.
  float roll_before = before_controller[2];
  float roll_after = after_controller[2];
  float pitch_before = before_controller[1];
  float pitch_after = after_controller[1];
  int before_pin_x = POS_X(CONTROLS_POS_X) + CONTROLS_SIZE / 2 +
                     roll_before * CONTROLS_SIZE / 2 - PIN_SIZE / 2;
  int before_pin_y = POS_Y(CONTROLS_POS_Y) + CONTROLS_SIZE / 2 +
                     pitch_before * CONTROLS_SIZE / 2 - PIN_SIZE / 2;
  int after_pin_x = POS_X(CONTROLS_POS_X) + CONTROLS_SIZE / 2 +
                    roll_after * CONTROLS_SIZE / 2 - PIN_SIZE / 2;
  int after_pin_y = POS_Y(CONTROLS_POS_Y) + CONTROLS_SIZE / 2 +
                    pitch_after * CONTROLS_SIZE / 2 - PIN_SIZE / 2;

  m_driver->draw2DImage(
      m_controls_image,
      irr::core::position2d<int>(POS_X(CONTROLS_POS_X), POS_Y(CONTROLS_POS_Y)),
      irr::core::rect<int>(0, 0, CONTROLS_SIZE, CONTROLS_SIZE), NULL,
      irr::video::SColor(255, 255, 255, 255), true);

  m_driver->draw2DImage(m_pin_image,
                        irr::core::position2d<int>(after_pin_x, after_pin_y),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        irr::video::SColor(255, 255, 255, 255), true);

  m_driver->draw2DImage(m_second_pin_image,
                        irr::core::position2d<int>(before_pin_x, before_pin_y),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        irr::video::SColor(255, 255, 255, 255), true);

  // Draw the yaw controls view.
  float yaw_before = before_controller[3];
  float yaw_after = after_controller[3];
  int yaw_before_pin_x = POS_X(YAW_VIEW_POS_X) + CONTROLS_SIZE / 2 +
                         yaw_before * CONTROLS_SIZE / 2 - PIN_SIZE / 2;
  int yaw_before_pin_y =
      POS_Y(YAW_VIEW_POS_Y) + YAW_VIEW_HEIGHT / 2 - PIN_SIZE / 2;
  int yaw_after_pin_x = POS_X(YAW_VIEW_POS_X) + CONTROLS_SIZE / 2 +
                        yaw_after * CONTROLS_SIZE / 2 - PIN_SIZE / 2;
  int yaw_after_pin_y = POS_Y(YAW_VIEW_POS_Y) + YAW_VIEW_HEIGHT / 2 - PIN_SIZE;
  m_driver->draw2DImage(
      m_yaw_image,
      irr::core::position2d<int>(POS_X(YAW_VIEW_POS_X), POS_Y(YAW_VIEW_POS_Y)),
      irr::core::rect<int>(0, 0, CONTROLS_SIZE, YAW_VIEW_HEIGHT), NULL,
      irr::video::SColor(255, 255, 255, 255), true);

  m_driver->draw2DImage(m_pin_image,
                        irr::core::rect<int>(yaw_after_pin_x, yaw_after_pin_y,
                                             yaw_after_pin_x + PIN_SIZE,
                                             yaw_after_pin_y + PIN_SIZE * 2),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        NULL, true);

  m_driver->draw2DImage(m_second_pin_image,
                        irr::core::rect<int>(yaw_before_pin_x, yaw_before_pin_y,
                                             yaw_before_pin_x + PIN_SIZE,
                                             yaw_before_pin_y + PIN_SIZE),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        NULL, true);

  // Draw the lift/throttle curves text.
  m_driver->draw2DImage(
      m_curves_text,
      irr::core::rect<int>(POS_X(CURVES_IMAGE_POS_X),
                           POS_Y(CURVES_IMAGE_POS_Y) - 30,
                           POS_X(CURVES_IMAGE_POS_X) + TEXT_WIDTH,
                           POS_Y(CURVES_IMAGE_POS_Y) + TEXT_HEIGHT - 30),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);
  m_driver->draw2DImage(
      m_switch_curves_text,
      irr::core::rect<int>(POS_X(CURVES_IMAGE_POS_X),
                           POS_Y(CURVES_IMAGE_POS_Y) + CURVES_IMAGE_HEIGHT,
                           POS_X(CURVES_IMAGE_POS_X) + TEXT_WIDTH,
                           POS_Y(CURVES_IMAGE_POS_Y) + CURVES_IMAGE_HEIGHT +
                               TEXT_HEIGHT),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);

  // Draw the lift/throttle curves.
  m_driver->draw2DImage(
      m_curves_images[active_curve_index],
      irr::core::position2d<int>(POS_X(CURVES_IMAGE_POS_X),
                                 POS_Y(CURVES_IMAGE_POS_Y)),
      irr::core::rect<int>(0, 0, CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT), NULL,
      irr::video::SColor(255, 255, 255, 255), true);

  int vertical_line_x_offset =
      float(CURVES_IMAGE_WIDTH) * (user_input.throttle_stick + 1) / 2;
  m_driver->draw2DImage(
      m_curves_vertical_line,
      irr::core::rect<int>(POS_X(CURVES_IMAGE_POS_X) + vertical_line_x_offset,
                           POS_Y(CURVES_IMAGE_POS_Y),
                           POS_X(CURVES_IMAGE_POS_X) + vertical_line_x_offset +
                               3,
                           POS_Y(CURVES_IMAGE_POS_Y) + CURVES_IMAGE_HEIGHT),
      irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL, NULL, true);

  float pin_size = 1.2 * PIN_SIZE;
  int throttle_pin_x = vertical_line_x_offset - pin_size / 2;
  int throttle_pin_y =
      float(CURVES_IMAGE_HEIGHT) * (-after_controller[0] + 1) / 2 -
      pin_size / 2;
  m_driver->draw2DImage(
      m_pin_image,
      irr::core::rect<int>(
          throttle_pin_x + POS_X(CURVES_IMAGE_POS_X),
          throttle_pin_y + POS_Y(CURVES_IMAGE_POS_Y),
          throttle_pin_x + POS_X(CURVES_IMAGE_POS_X) + pin_size,
          throttle_pin_y + POS_Y(CURVES_IMAGE_POS_Y) + pin_size),
      irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL, NULL, true);

  int lift_pin_y =
      float(CURVES_IMAGE_HEIGHT) * (-after_controller[4] + 1) / 2 -
      pin_size / 2;
  m_driver->draw2DImage(
      m_pin_image,
      irr::core::rect<int>(throttle_pin_x + POS_X(CURVES_IMAGE_POS_X),
                           lift_pin_y + POS_Y(CURVES_IMAGE_POS_Y),
                           throttle_pin_x + POS_X(CURVES_IMAGE_POS_X) +
                               pin_size,
                           lift_pin_y + POS_Y(CURVES_IMAGE_POS_Y) + pin_size),
      irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL, NULL, true);

  // Draw the trottle text.
  m_driver->draw2DImage(
      m_throttle_text,
      irr::core::rect<int>(POS_X(MAIN_ROTOR_INDICATOR_X) - 20,
                           POS_Y(MAIN_ROTOR_INDICATOR_Y) - TEXT_HEIGHT / 6,
                           POS_X(MAIN_ROTOR_INDICATOR_X) + TEXT_WIDTH - 20,
                           POS_Y(MAIN_ROTOR_INDICATOR_Y) + TEXT_HEIGHT / 6 * 5),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);
  m_driver->draw2DImage(
      m_throttle_hold_text,
      irr::core::rect<int>(POS_X(MAIN_ROTOR_INDICATOR_X),
                           POS_Y(MAIN_ROTOR_INDICATOR_Y) +
                               MAIN_ROTOR_INDICATOR_HEIGHT,
                           POS_X(MAIN_ROTOR_INDICATOR_X) + TEXT_WIDTH,
                           POS_Y(MAIN_ROTOR_INDICATOR_Y) +
                               MAIN_ROTOR_INDICATOR_HEIGHT + TEXT_HEIGHT),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);

  // Draw the main rotor indicator.
  m_driver->draw2DImage(
      m_main_rotor_indicator_image,
      irr::core::rect<int>(
          POS_X(MAIN_ROTOR_INDICATOR_X), POS_Y(MAIN_ROTOR_INDICATOR_Y),
          POS_X(MAIN_ROTOR_INDICATOR_X) + MAIN_ROTOR_INDICATOR_WIDTH,
          POS_Y(MAIN_ROTOR_INDICATOR_Y) + MAIN_ROTOR_INDICATOR_HEIGHT),
      irr::core::rect<int>(0, 0, 651, 394), NULL, NULL, true);

  int hand_x = POS_X(MAIN_ROTOR_INDICATOR_X) + MAIN_ROTOR_INDICATOR_WIDTH / 2 -
               MAIN_ROTOR_INDICATOR_HAND_WIDTH / 2;
  int hand_y = POS_Y(MAIN_ROTOR_INDICATOR_Y) + MAIN_ROTOR_INDICATOR_HEIGHT -
               MAIN_ROTOR_INDICATOR_HAND_HEIGHT;
  int hand_rotation_point_x =
      hand_x + MAIN_ROTOR_INDICATOR_HAND_WIDTH * (45.7 / 91);
  int hand_rotation_point_y =
      hand_y + MAIN_ROTOR_INDICATOR_HAND_HEIGHT * (286.6 / 329);
  float size_x = float(MAIN_ROTOR_INDICATOR_HAND_WIDTH) / 91;
  float size_y = float(MAIN_ROTOR_INDICATOR_HAND_HEIGHT) / 329;
  draw2DImageWithRotation(
      m_driver,
      /*texture*/ m_main_rotor_indicator_target_hand_image,
      /*sourceRect*/ irr::core::rect<int>(0, 0, 91, 329),
      /*position*/ irr::core::position2d<irr::s32>(hand_x, hand_y),
      /*rotationPoint*/
      irr::core::position2d<irr::s32>(hand_rotation_point_x,
                                      hand_rotation_point_y),
      /*rotation*/ 180. * telemetry.target_rps / m_max_rps - 90.,
      /*scale*/ irr::core::vector2df(size_x, size_y),
      /*color*/ irr::video::SColor(255, 255, 255, 255));
  draw2DImageWithRotation(
      m_driver,
      /*texture*/ m_main_rotor_indicator_hand_image,
      /*sourceRect*/ irr::core::rect<int>(0, 0, 91, 329),
      /*position*/ irr::core::position2d<irr::s32>(hand_x, hand_y),
      /*rotationPoint*/
      irr::core::position2d<irr::s32>(hand_rotation_point_x,
                                      hand_rotation_point_y),
      /*rotation*/ 180. * telemetry.rps / m_max_rps - 90.,
      /*scale*/ irr::core::vector2df(size_x, size_y),
      /*color*/ irr::video::SColor(255, 255, 255, 255));
}
