#include "controls_view.h"
#include <cmath>

const int CONTROLS_SIZE = 140;
const int YAW_VIEW_HEIGHT = 30;
const int PIN_SIZE = 12;
const int LOCATION_X = 50;
const int LOCATION_Y = 50;
const int YAW_VIEW_LOCATION_Y = LOCATION_Y + CONTROLS_SIZE + 20;
const int YAW_VIEW_LOCATION_X = LOCATION_X;
const int LIFT_VIEW_WIDTH = 30;
const int LIFT_VIEW_LOCATION_Y = LOCATION_Y;
const int LIFT_VIEW_LOCATION_X = LOCATION_X - LIFT_VIEW_WIDTH - 10;


ControlsView::ControlsView(irr::video::IVideoDriver *driver) {

    // Create the controls image.
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

    // Create the lift control image.
    irr::video::IImage * lift_image = driver->createImage(
            irr::video::ECF_A8R8G8B8,
            irr::core::dimension2d<unsigned int>(LIFT_VIEW_WIDTH, CONTROLS_SIZE)
    );

    for (int x=0; x<LIFT_VIEW_WIDTH; x++) {
        for (int y=0; y<CONTROLS_SIZE; y++) {
            int half_x = LIFT_VIEW_WIDTH / 2;
            int half_y = CONTROLS_SIZE / 2;
            int y_fade = 0.25 * CONTROLS_SIZE;
            // Draw the axes;
            if ((x > half_x-1 && x < half_x+1) || (y > half_y-1 && y < half_y+1)) {
                lift_image->setPixel(x, y, irr::video::SColor(0xff, 0, 0, 0));
            }
            // Draw the gradient;
            else {
                float distance = float(std::abs(x - half_x)) / half_x;
                float intensity = 1-distance;
                if (y < y_fade) {
                    intensity *= 1 - std::pow(float(y_fade - y) / y_fade, 2);
                } else if (y > CONTROLS_SIZE - y_fade) {
                    intensity *= 1 - std::pow(float(y - (CONTROLS_SIZE - y_fade)) / y_fade, 2);
                }
                lift_image->setPixel(x, y, irr::video::SColor(150*intensity, 0xA0, 0, 0));
            }
        }
    }

    m_lift_image = driver->addTexture(irr::io::path("lift_image"), lift_image);

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
}

void ControlsView::update_ui(const ServoData &before_controller, const ServoData &after_controller) {

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

    // Draw the lift controls view.
    int lift_before_pin_x = LIFT_VIEW_LOCATION_X + LIFT_VIEW_WIDTH/2 - PIN_SIZE/2;
    int lift_before_pin_y = LIFT_VIEW_LOCATION_Y + CONTROLS_SIZE/2 - before_controller.lift*CONTROLS_SIZE/2 - PIN_SIZE/2;
    int lift_after_pin_x = LIFT_VIEW_LOCATION_X + LIFT_VIEW_WIDTH/2 - PIN_SIZE;
    int lift_after_pin_y = LIFT_VIEW_LOCATION_Y + CONTROLS_SIZE/2 - after_controller.lift*CONTROLS_SIZE/2 - PIN_SIZE/2;

    m_driver->draw2DImage(
            m_lift_image,
            irr::core::position2d<int>(LIFT_VIEW_LOCATION_X, LIFT_VIEW_LOCATION_Y),
            irr::core::rect<int>(0, 0, LIFT_VIEW_WIDTH, CONTROLS_SIZE),
            NULL, 
            irr::video::SColor(255, 255, 255, 255),
            true);

    
    m_driver->draw2DImage(
            m_pin_image,
            irr::core::rect<int>(lift_after_pin_x,
                                 lift_after_pin_y,
                                 lift_after_pin_x+PIN_SIZE*2,
                                 lift_after_pin_y+PIN_SIZE),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);

    m_driver->draw2DImage(
            m_second_pin_image,
            irr::core::rect<int>(lift_before_pin_x,
                                 lift_before_pin_y,
                                 lift_before_pin_x+PIN_SIZE,
                                 lift_before_pin_y+PIN_SIZE),
            irr::core::rect<int>(0, 0, PIN_SIZE, PIN_SIZE),
            NULL, NULL, true);
            
}
