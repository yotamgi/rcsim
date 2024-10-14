#include "controls.h"
#include <cmath>

#include <iostream>
#include <sstream>

const int CONTROLS_SIZE = 140;
const int YAW_VIEW_HEIGHT = 30;
const int PIN_SIZE = 12;
const int LOCATION_X = 20;
const int LOCATION_Y = 50;
const int YAW_VIEW_LOCATION_Y = LOCATION_Y + CONTROLS_SIZE + 20;
const int YAW_VIEW_LOCATION_X = LOCATION_X;
const int LIFT_VIEW_WIDTH = 30;
const int LIFT_VIEW_LOCATION_Y = LOCATION_Y;
const int LIFT_VIEW_LOCATION_X = LOCATION_X - LIFT_VIEW_WIDTH - 10;

const int CURVES_IMAGE_LOCATION_X = 200;
const int CURVES_IMAGE_LOCATION_Y = 45;
const int CURVES_IMAGE_HEIGHT = 140;
const int CURVES_IMAGE_WIDTH = 250;
const int CURVES_GRID_SIZE = 25;
const int CURVE_THICKNESS = 2.5;


static float curve_x_to_value(int x) {
    return (float(x) / CURVES_IMAGE_WIDTH) * 2. - 1.;
}
static float curve_value_to_y(float value) {
    return float(CURVES_IMAGE_HEIGHT) * ((value + 1.) / 2.);
}


static void plot_curve(const ControllerCurve &curve, 
                       irr::video::IImage *image,
                       const irr::video::SColor &color,
                       int image_width = CURVES_IMAGE_WIDTH,
                       int image_height = CURVES_IMAGE_HEIGHT)
{
    float curve_y_value;

    for (int x=0; x<image_width; x++) {
        float in_value = curve_x_to_value(x);
        float new_curve_y_value = curve_value_to_y(-curve.translate(in_value));
        float grad = (x == 0) ? 0 : (new_curve_y_value - curve_y_value);
        curve_y_value = new_curve_y_value;

        float grad_angle = irr::core::PI/2 - std::atan(grad);
        float thickness_squared = std::pow(CURVE_THICKNESS / std::sin(grad_angle), 3);

        for (int y=0; y<image_height; y++) {
            float distance = std::abs(float(y) - curve_y_value);

            float value = (thickness_squared - std::pow(distance, 3)) / thickness_squared;
            value = (value > 0) ? value : 0;
            irr::video::SColor prev_color = image->getPixel(x, y);
            int new_alpha = std::max(prev_color.getAlpha(), (unsigned int)(value*color.getAlpha()));
            int new_red = value * color.getRed() + (1-value) * prev_color.getRed();
            int new_green = value * color.getGreen() + (1-value) * prev_color.getGreen();
            int new_blue = value * color.getBlue() + (1-value) * prev_color.getBlue();
            image->setPixel(x, y, irr::video::SColor(new_alpha, new_red, new_green, new_blue));
        }
    }
}


ControlsView::ControlsView(
            irr::video::IVideoDriver *driver,
            std::vector<ControllerCurve> throttle_curves,
            std::vector<ControllerCurve> lift_curves):
    m_throttle_curves(throttle_curves),
    m_lift_curves(lift_curves)
{

    // Create the pitch-roll controls image.
    irr::video::IImage * controls_image = driver->createImage(
            irr::video::ECF_A8R8G8B8,
            irr::core::dimension2d<unsigned int>(CONTROLS_SIZE, CONTROLS_SIZE)
    );

    for (int x=0; x<CONTROLS_SIZE; x++) {
        for (int y=0; y<CONTROLS_SIZE; y++) {
            int half = CONTROLS_SIZE / 2;
            // Draw the axes;
            if ((x > half-1 && x < half+1) || (y > half-1 && y < half+1)) {
                controls_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
            }
            // Draw the gradient;
            else {
                float distance = float((x-half)*(x-half) + (y-half)*(y-half)) / (half*half);
                float intensity = distance > 1? 0: 1-distance;
                controls_image->setPixel(x, y, irr::video::SColor(150*intensity, 0xA0, 0, 0));
            }
        }
    }

    m_controls_image = driver->addTexture(irr::io::path("controls_image"), controls_image);

    // Create the yaw control image.
    irr::video::IImage * yaw_image = driver->createImage(
            irr::video::ECF_A8R8G8B8,
            irr::core::dimension2d<unsigned int>(CONTROLS_SIZE, YAW_VIEW_HEIGHT)
    );

    for (int x=0; x<CONTROLS_SIZE; x++) {
        for (int y=0; y<YAW_VIEW_HEIGHT; y++) {
            int half_x = CONTROLS_SIZE / 2;
            int half_y = YAW_VIEW_HEIGHT / 2;
            int x_fade = 0.25 * CONTROLS_SIZE;
            // Draw the axes;
            if ((x > half_x-1 && x < half_x+1) || (y > half_y-1 && y < half_y+1)) {
                yaw_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
            }
            // Draw the gradient;
            else {
                float distance = float(std::abs(y - half_y)) / half_y;
                float intensity = 1-distance;
                if (x < x_fade) {
                    intensity *= 1 - std::pow(float(x_fade - x) / x_fade, 2);
                } else if (x > CONTROLS_SIZE - x_fade) {
                    intensity *= 1 - std::pow(float(x - (CONTROLS_SIZE - x_fade)) / x_fade, 2);
                }
                yaw_image->setPixel(x, y, irr::video::SColor(150*intensity, 0xA0, 0, 0));
            }
        }
    }

    m_yaw_image = driver->addTexture(irr::io::path("yaw_image"), yaw_image);

    // Create the pin image;
    irr::video::IImage *pin_image = driver->createImage(
            irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(PIN_SIZE, PIN_SIZE));

    for (int x=0; x<PIN_SIZE; x++) {
        for (int y=0; y<PIN_SIZE; y++) {
            int half = PIN_SIZE / 2;
            float distance = std::pow(float((x-half)*(x-half) + (y-half)*(y-half)) / (half*half), 3);
            float intensity = distance > 1? 0: 1-distance;
            pin_image->setPixel(x, y, irr::video::SColor(150*intensity, 0xff, 0xff, 0));
        }
    }
    m_pin_image = driver->addTexture(irr::io::path("pin_image"), pin_image);

    // Create the second pin image.
    irr::video::IImage *second_pin_image = driver->createImage(
            irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(PIN_SIZE, PIN_SIZE));

    for (int x=0; x<PIN_SIZE; x++) {
        for (int y=0; y<PIN_SIZE; y++) {
            int half = PIN_SIZE / 2;
            if ((x < half+2 && x > half-2) || (y < half+2 && y > half-2)) {
                second_pin_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
            } else {
                second_pin_image->setPixel(x, y, irr::video::SColor(0, 0, 0, 0));
            }
        }
    }
    m_second_pin_image = driver->addTexture(irr::io::path("second_pin_image"), second_pin_image);

    m_driver = driver;

    // Create the throttle-lift curves canvas.
    for (size_t curve_index = 0; curve_index < m_throttle_curves.size(); curve_index++) {
        irr::video::IImage *curves_image = driver->createImage(
                irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(
                CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT));

        for (int x=0; x<CURVES_IMAGE_WIDTH; x++) {
            for (int y=0; y<CURVES_IMAGE_HEIGHT; y++) {
                // The grid:
                if (((x % CURVES_GRID_SIZE == 0) || (y % CURVES_GRID_SIZE == 0)) && (x != 0) && (y != 0)) {
                    curves_image->setPixel(x, y, irr::video::SColor(0x40, 0, 0, 0));
                }
                else {
                    curves_image->setPixel(x, y, irr::video::SColor(0x20, 0, 0, 0));
                }
            }
        }
        plot_curve(m_throttle_curves[curve_index], 
                   curves_image,
                   irr::video::SColor(0xb0, 0x8c, 0x2a, 0xde)
        );
        plot_curve(m_lift_curves[curve_index],
                   curves_image,
                   irr::video::SColor(0xb0, 0x42, 0x9e, 0xff)
        );

        std::stringstream ss;
        ss << "curve_images_" << curve_index;
        m_curves_images.push_back(driver->addTexture(ss.str().c_str(), curves_image));
    }


    // Create the curves vertical line.
    irr::video::IImage *curves_vertical_line = driver->createImage(
            irr::video::ECF_A8R8G8B8, irr::core::dimension2d<unsigned int>(
            3, CURVES_IMAGE_HEIGHT));
    for (int x=0; x<3; x++) {
        for (int y=0; y<CURVES_IMAGE_HEIGHT; y++) {
            curves_vertical_line->setPixel(x, y, irr::video::SColor(0x80, 0, 0, 0));
        }
    }
    m_curves_vertical_line = driver->addTexture(irr::io::path("curves_vertical_line"), curves_vertical_line);

    m_driver = driver;
}

void ControlsView::update_ui(
            const ControlsInput &user_input,
            const ServoData &before_controller,
            const ServoData &after_controller,
            int active_curve_index)
{
    // Draw the pitch-roll control view.
    int before_pin_x = LOCATION_X + CONTROLS_SIZE/2 + before_controller.roll*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int before_pin_y = LOCATION_Y + CONTROLS_SIZE/2 + before_controller.pitch*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int after_pin_x = LOCATION_X + CONTROLS_SIZE/2 + after_controller.roll*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int after_pin_y = LOCATION_Y + CONTROLS_SIZE/2 + after_controller.pitch*CONTROLS_SIZE/2 - PIN_SIZE/2;

    m_driver->draw2DImage(
            m_controls_image,
            irr::core::position2d<int>(LOCATION_X, LOCATION_Y),
            irr::core::rect<int>(0, 0, CONTROLS_SIZE, CONTROLS_SIZE),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    m_driver->draw2DImage(
            m_pin_image,
            irr::core::position2d<int>(after_pin_x, after_pin_y),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    m_driver->draw2DImage(
            m_second_pin_image,
            irr::core::position2d<int>(before_pin_x, before_pin_y),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    // Draw the yaw controls view.
    int yaw_before_pin_x = YAW_VIEW_LOCATION_X + CONTROLS_SIZE/2 + before_controller.yaw*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int yaw_before_pin_y = YAW_VIEW_LOCATION_Y + YAW_VIEW_HEIGHT/2 - PIN_SIZE/2;
    int yaw_after_pin_x = YAW_VIEW_LOCATION_X + CONTROLS_SIZE/2 + after_controller.yaw*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int yaw_after_pin_y = YAW_VIEW_LOCATION_Y + YAW_VIEW_HEIGHT/2 - PIN_SIZE;
    m_driver->draw2DImage(
            m_yaw_image,
            irr::core::position2d<int>(YAW_VIEW_LOCATION_X, YAW_VIEW_LOCATION_Y),
            irr::core::rect<int>(0, 0, CONTROLS_SIZE, YAW_VIEW_HEIGHT),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    m_driver->draw2DImage(
            m_pin_image,
            irr::core::rect<int>(yaw_after_pin_x,
                                 yaw_after_pin_y,
                                 yaw_after_pin_x+PIN_SIZE,
                                 yaw_after_pin_y+PIN_SIZE*2),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);

    m_driver->draw2DImage(
            m_second_pin_image,
            irr::core::rect<int>(yaw_before_pin_x,
                                 yaw_before_pin_y,
                                 yaw_before_pin_x+PIN_SIZE,
                                 yaw_before_pin_y+PIN_SIZE),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);

    // Draw the lift/throttle curves.
    m_driver->draw2DImage(
            m_curves_images[active_curve_index],
            irr::core::position2d<int>(CURVES_IMAGE_LOCATION_X, CURVES_IMAGE_LOCATION_Y),
            irr::core::rect<int>(0, 0, CURVES_IMAGE_WIDTH, CURVES_IMAGE_HEIGHT),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    int vertical_line_x_offset = float(CURVES_IMAGE_WIDTH) * (user_input.throttle_stick + 1) / 2;
    m_driver->draw2DImage(
            m_curves_vertical_line,
            irr::core::rect<int>(CURVES_IMAGE_LOCATION_X + vertical_line_x_offset,
                                 CURVES_IMAGE_LOCATION_Y,
                                 CURVES_IMAGE_LOCATION_X + vertical_line_x_offset + 3,
                                 CURVES_IMAGE_LOCATION_Y + CURVES_IMAGE_HEIGHT),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);

    float pin_size = 1.2 * PIN_SIZE;
    int throttle_pin_x = vertical_line_x_offset - pin_size / 2;
    int throttle_pin_y = float(CURVES_IMAGE_HEIGHT)
                * (-after_controller.throttle + 1) / 2
                - pin_size / 2;
    m_driver->draw2DImage(
            m_pin_image,
            irr::core::rect<int>(throttle_pin_x + CURVES_IMAGE_LOCATION_X,
                                 throttle_pin_y + CURVES_IMAGE_LOCATION_Y,
                                 throttle_pin_x + CURVES_IMAGE_LOCATION_X + pin_size,
                                 throttle_pin_y + CURVES_IMAGE_LOCATION_Y + pin_size),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);

    int lift_pin_y = float(CURVES_IMAGE_HEIGHT)
                * (-after_controller.lift + 1) / 2
                - pin_size / 2;
    m_driver->draw2DImage(
            m_pin_image,
            irr::core::rect<int>(throttle_pin_x + CURVES_IMAGE_LOCATION_X,
                                 lift_pin_y + CURVES_IMAGE_LOCATION_Y,
                                 throttle_pin_x + CURVES_IMAGE_LOCATION_X + pin_size,
                                 lift_pin_y + CURVES_IMAGE_LOCATION_Y + pin_size),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);
}
