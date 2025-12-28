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

static int pos_y_(irr::video::IVideoDriver *driver, int pos_y) {
  return driver->getScreenSize().Height - pos_y;
}

static int pos_x_ratio(irr::video::IVideoDriver *driver, float pos_x_ratio) {
  return driver->getScreenSize().Width * pos_x_ratio;
}

#define POS_Y(y) pos_y_(m_driver, (y))
#define POS_X(x) pos_x_ratio(m_driver, (x))

static void draw_image_with_rotation(
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

static irr::video::ITexture *
create_circular_pin_image(irr::video::IVideoDriver *driver, int size,
                          const irr::video::SColor &color) {
  irr::video::IImage *pin_image =
      driver->createImage(irr::video::ECF_A8R8G8B8,
                          irr::core::dimension2d<unsigned int>(size, size));

  for (int x = 0; x < size; x++) {
    for (int y = 0; y < size; y++) {
      int half = size / 2;
      float distance =
          std::pow(float((x - half) * (x - half) + (y - half) * (y - half)) /
                       (half * half),
                   3);
      float intensity = distance > 1 ? 0 : 1 - distance;
      pin_image->setPixel(x, y,
                          irr::video::SColor(150 * intensity, 0xff, 0xff, 0));
    }
  }
  return driver->addTexture(irr::io::path("pin_image"), pin_image);
}

static irr::video::ITexture *
create_plus_pin_image(irr::video::IVideoDriver *driver, int size,
                      const irr::video::SColor &color) {

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
  return driver->addTexture(irr::io::path("second_pin_image"),
                            second_pin_image);
}

////////////////////////////////////////////////////////////////////////////////
// HorizontalControlsInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

HorizontalControlsInstrument::HorizontalControlsInstrument(
    irr::video::IVideoDriver *driver, float pos_x, int pos_y, int width,
    int height)
    : m_driver(driver), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_width(width), m_height(height) {

  // Create the yaw control image.
  irr::video::IImage *yaw_image =
      driver->createImage(irr::video::ECF_A8R8G8B8,
                          irr::core::dimension2d<unsigned int>(width, height));

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      int half_x = width / 2;
      int half_y = height / 2;
      int x_fade = 0.25 * width;
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
        } else if (x > width - x_fade) {
          intensity *= 1 - std::pow(float(x - (width - x_fade)) / x_fade, 2);
        }
        yaw_image->setPixel(x, y,
                            irr::video::SColor(150 * intensity, 0xA0, 0, 0));
      }
    }
  }

  m_yaw_image = driver->addTexture(irr::io::path("yaw_image"), yaw_image);
  m_pin_image = create_circular_pin_image(driver, PIN_SIZE,
                                          irr::video::SColor(0xff, 0xff, 0, 0));
  m_second_pin_image = create_plus_pin_image(driver, PIN_SIZE,
                                             irr::video::SColor(0xff, 0, 0, 0));
}

void HorizontalControlsInstrument::update(float pin1, float pin2) {
  // Draw the yaw controls view.
  int yaw_before_pin_x =
      m_pos_x + m_width / 2 + pin1 * m_width / 2 - PIN_SIZE / 2;
  int yaw_before_pin_y = m_pos_y + m_height / 2 - PIN_SIZE / 2;
  int yaw_after_pin_x =
      m_pos_x + m_width / 2 + pin2 * m_width / 2 - PIN_SIZE / 2;
  int yaw_after_pin_y = m_pos_y + m_height / 2 - PIN_SIZE;
  m_driver->draw2DImage(m_yaw_image,
                        irr::core::position2d<int>(m_pos_x, m_pos_y),
                        irr::core::rect<int>(0, 0, m_width, m_height), NULL,
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
}

////////////////////////////////////////////////////////////////////////////////
// Controls2dInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

Controls2dInstrument::Controls2dInstrument(irr::video::IVideoDriver *driver,
                                           float pos_x, int pos_y, int size)
    : m_driver(driver), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_size(size) {
  // Create the background image.
  irr::video::IImage *controls_image = driver->createImage(
      irr::video::ECF_A8R8G8B8,
      irr::core::dimension2d<unsigned int>(CONTROLS_SIZE, CONTROLS_SIZE));

  for (int x = 0; x < size; x++) {
    for (int y = 0; y < size; y++) {
      int half = size / 2;
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

  m_pin_image = create_circular_pin_image(driver, PIN_SIZE,
                                          irr::video::SColor(0xff, 0xff, 0, 0));
  m_second_pin_image = create_plus_pin_image(driver, PIN_SIZE,
                                             irr::video::SColor(0xff, 0, 0, 0));
}

void Controls2dInstrument::update(float pin1_y, float pin1_x, float pin2_y,
                                  float pin2_x) {
  // Calculate the pin positions.
  int before_pin_x = m_pos_x + m_size / 2 + pin1_x * m_size / 2 - PIN_SIZE / 2;
  int before_pin_y = m_pos_y + m_size / 2 + pin1_y * m_size / 2 - PIN_SIZE / 2;
  int after_pin_x = m_pos_x + m_size / 2 + pin2_x * m_size / 2 - PIN_SIZE / 2;
  int after_pin_y = m_pos_y + m_size / 2 + pin2_y * m_size / 2 - PIN_SIZE / 2;

  m_driver->draw2DImage(m_controls_image,
                        irr::core::position2d<int>(m_pos_x, m_pos_y),
                        irr::core::rect<int>(0, 0, m_size, m_size), NULL,
                        irr::video::SColor(255, 255, 255, 255), true);

  m_driver->draw2DImage(m_pin_image,
                        irr::core::position2d<int>(after_pin_x, after_pin_y),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        irr::video::SColor(255, 255, 255, 255), true);

  m_driver->draw2DImage(m_second_pin_image,
                        irr::core::position2d<int>(before_pin_x, before_pin_y),
                        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL,
                        irr::video::SColor(255, 255, 255, 255), true);
}

////////////////////////////////////////////////////////////////////////////////
// CurvesInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

CurvesInstrument::CurvesInstrument(
    irr::video::IVideoDriver *driver,
    std::vector<std::vector<ControllerCurve>> curve_groups,
    std::vector<irr::video::SColor> curve_colors, float pos_x, int pos_y,
    int width, int height)
    : m_driver(driver), m_curve_groups(curve_groups), m_pos_x(POS_X(pos_x)),
      m_pos_y(POS_Y(pos_y)), m_width(width), m_height(height) {

  size_t num_curves_in_group = curve_groups[0].size();

  // Create the throttle-lift curves canvas.
  for (size_t curve_index = 0; curve_index < num_curves_in_group;
       curve_index++) {
    irr::video::IImage *curves_image = driver->createImage(
        irr::video::ECF_A8R8G8B8,
        irr::core::dimension2d<unsigned int>(width, height));

    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        // The grid:
        if (((x % CURVES_GRID_SIZE == 0) || (y % CURVES_GRID_SIZE == 0)) &&
            (x != 0) && (y != 0)) {
          curves_image->setPixel(x, y, irr::video::SColor(0x40, 0, 0, 0));
        } else {
          curves_image->setPixel(x, y, irr::video::SColor(0x20, 0, 0, 0));
        }
      }
    }
    for (size_t curve_group_index = 0; curve_group_index < curve_groups.size();
         curve_group_index++) {
      plot_curve(curve_groups[curve_group_index][curve_index], curves_image,
                 curve_colors[curve_group_index]);
    }

    std::stringstream ss;
    ss << "curve_images_" << curve_index;
    m_curves_images.push_back(
        driver->addTexture(ss.str().c_str(), curves_image));
  }

  // Create the curves vertical line.
  irr::video::IImage *curves_vertical_line =
      driver->createImage(irr::video::ECF_A8R8G8B8,
                          irr::core::dimension2d<unsigned int>(3, height));
  for (int x = 0; x < 3; x++) {
    for (int y = 0; y < height; y++) {
      curves_vertical_line->setPixel(x, y, irr::video::SColor(0x80, 0, 0, 0));
    }
  }
  m_curves_vertical_line = driver->addTexture(
      irr::io::path("curves_vertical_line"), curves_vertical_line);
  m_pin_image = create_circular_pin_image(driver, PIN_SIZE,
                                          irr::video::SColor(0xff, 0xff, 0, 0));
}

void CurvesInstrument::update(unsigned long active_curve_index,
                              float x_stick_level,
                              std::vector<float> y_levels) {
  // Draw the lift/throttle curves text.
  if (m_curves_images.size() != 0) {
    // Draw the lift/throttle curves.
    m_driver->draw2DImage(m_curves_images[active_curve_index],
                          irr::core::position2d<int>(m_pos_x, m_pos_y),
                          irr::core::rect<int>(0, 0, m_width, m_height), NULL,
                          irr::video::SColor(255, 255, 255, 255), true);
  }

  int vertical_line_x_offset = float(m_width) * (x_stick_level + 1) / 2;
  m_driver->draw2DImage(
      m_curves_vertical_line,
      irr::core::rect<int>(m_pos_x + vertical_line_x_offset, m_pos_y,
                           m_pos_x + vertical_line_x_offset + 3,
                           m_pos_y + m_height),
      irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL, NULL, true);

  float pin_size = 1.2 * PIN_SIZE;
  for (float y_level : y_levels) {
    int throttle_pin_x = vertical_line_x_offset - pin_size / 2;
    int throttle_pin_y = float(m_height) * (-y_level + 1) / 2 - pin_size / 2;
    m_driver->draw2DImage(
        m_pin_image,
        irr::core::rect<int>(throttle_pin_x + m_pos_x, throttle_pin_y + m_pos_y,
                             throttle_pin_x + m_pos_x + pin_size,
                             throttle_pin_y + m_pos_y + pin_size),
        irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE), NULL, NULL, true);
  }
}

////////////////////////////////////////////////////////////////////////////////
//  SpeedometerInstrument class implementation
////////////////////////////////////////////////////////////////////////////////

SpeedometerInstrument::SpeedometerInstrument(irr::video::IVideoDriver *driver,
                                             float pos_x, int pos_y,
                                             float max_value)
    : m_driver(driver), m_pos_x(POS_X(pos_x)), m_pos_y(POS_Y(pos_y)),
      m_max_value(max_value) {

  // Create the main rotor indicator.
  m_speedometer_image = driver->getTexture("media/speedometer3_meter.png");
  m_speedometer_hand_image = driver->getTexture("media/speedometer3_hand.png");
  m_speedometer_hand2_image =
      driver->getTexture("media/speedometer3_hand_yellow.png");
}

void SpeedometerInstrument::update(float value1, float value2) {
  m_driver->draw2DImage(
      m_speedometer_image,
      irr::core::rect<int>(m_pos_x, m_pos_y,
                           m_pos_x + MAIN_ROTOR_INDICATOR_WIDTH,
                           m_pos_y + MAIN_ROTOR_INDICATOR_HEIGHT),
      irr::core::rect<int>(0, 0, 651, 394), NULL, NULL, true);

  int hand_x = m_pos_x + MAIN_ROTOR_INDICATOR_WIDTH / 2 -
               MAIN_ROTOR_INDICATOR_HAND_WIDTH / 2;
  int hand_y =
      m_pos_y + MAIN_ROTOR_INDICATOR_HEIGHT - MAIN_ROTOR_INDICATOR_HAND_HEIGHT;
  int hand_rotation_point_x =
      hand_x + MAIN_ROTOR_INDICATOR_HAND_WIDTH * (45.7 / 91);
  int hand_rotation_point_y =
      hand_y + MAIN_ROTOR_INDICATOR_HAND_HEIGHT * (286.6 / 329);
  float size_x = float(MAIN_ROTOR_INDICATOR_HAND_WIDTH) / 91;
  float size_y = float(MAIN_ROTOR_INDICATOR_HAND_HEIGHT) / 329;
  draw_image_with_rotation(
      m_driver,
      /*texture*/ m_speedometer_hand2_image,
      /*sourceRect*/ irr::core::rect<int>(0, 0, 91, 329),
      /*position*/ irr::core::position2d<irr::s32>(hand_x, hand_y),
      /*rotationPoint*/
      irr::core::position2d<irr::s32>(hand_rotation_point_x,
                                      hand_rotation_point_y),
      /*rotation*/ 180. * value2 / m_max_value - 90.,
      /*scale*/ irr::core::vector2df(size_x, size_y),
      /*color*/ irr::video::SColor(255, 255, 255, 255));
  draw_image_with_rotation(
      m_driver,
      /*texture*/ m_speedometer_hand_image,
      /*sourceRect*/ irr::core::rect<int>(0, 0, 91, 329),
      /*position*/ irr::core::position2d<irr::s32>(hand_x, hand_y),
      /*rotationPoint*/
      irr::core::position2d<irr::s32>(hand_rotation_point_x,
                                      hand_rotation_point_y),
      /*rotation*/ 180. * value1 / m_max_value - 90.,
      /*scale*/ irr::core::vector2df(size_x, size_y),
      /*color*/ irr::video::SColor(255, 255, 255, 255));
}

////////////////////////////////////////////////////////////////////////////////
//  Dashboard class implementation
////////////////////////////////////////////////////////////////////////////////

Dashboard::Dashboard(irr::video::IVideoDriver *driver,
                     std::vector<ControllerCurve> throttle_curves,
                     std::vector<ControllerCurve> lift_curves, float max_rps)
    : m_driver(driver), m_yaw_instrument(driver, YAW_VIEW_POS_X, YAW_VIEW_POS_Y,
                                         CONTROLS_SIZE, YAW_VIEW_HEIGHT),
      m_pitch_roll_instrument(driver, CONTROLS_POS_X, CONTROLS_POS_Y,
                              CONTROLS_SIZE),
      m_curves_instrument(driver, {throttle_curves, lift_curves},
                          {irr::video::SColor(0xb0, 0x8c, 0x2a, 0xde),
                           irr::video::SColor(0xb0, 0x42, 0x9e, 0xff)},
                          CURVES_IMAGE_POS_X, CURVES_IMAGE_POS_Y,
                          CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT),
      m_main_rotor_instrument(driver, MAIN_ROTOR_INDICATOR_X,
                              MAIN_ROTOR_INDICATOR_Y, max_rps) {

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

  // The text images.
  m_throttle_text = driver->getTexture("media/throttle_text.png");
  m_throttle_hold_text = driver->getTexture("media/throttle_hold_text.png");
  m_curves_text = driver->getTexture("media/curves_text.png");
  m_switch_curves_text = driver->getTexture("media/switch_curves_text.png");
  m_controls_and_gyro_view_text =
      driver->getTexture("media/controls_and_gyro_view.png");
}

void Dashboard::update_ui(const Controls::Telemetry &controls_telemetry,
                          const BaseHeli::Telemetry &telemetry) {
  ControlsInput user_input = controls_telemetry.user_input;
  ServoData before_controller = controls_telemetry.before_controller;
  ServoData after_controller = controls_telemetry.after_controller;

  // Draw the background
  m_driver->draw2DImage(m_dashboard_background,
                        irr::core::rect<int>(POS_X(0), POS_Y(BACKGROUND_HEIGHT),
                                             POS_X(1), POS_Y(0)),
                        irr::core::rect<int>(0, 0, 10, 10), NULL, NULL, true);

  // Draw the controls text.
  m_driver->draw2DImage(
      m_controls_and_gyro_view_text,
      irr::core::rect<int>(POS_X(CONTROLS_POS_X) - TEXT_WIDTH / 2,
                           POS_Y(CONTROLS_POS_Y) - 10,
                           POS_X(CONTROLS_POS_X) + TEXT_WIDTH / 2,
                           POS_Y(CONTROLS_POS_Y) + TEXT_HEIGHT - 10),
      irr::core::rect<int>(0, 0, 800, 100), NULL, NULL, true);

  m_pitch_roll_instrument.update(before_controller[1], before_controller[2],
                                 after_controller[1], after_controller[2]);

  m_yaw_instrument.update(before_controller[3], after_controller[3]);

  // Draw the curves text.
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

  m_curves_instrument.update(user_input.active_curve_index,
                             user_input.throttle_stick,
                             {after_controller[0], after_controller[4]});

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
  m_main_rotor_instrument.update(telemetry.rps, telemetry.target_rps);
}
