#ifndef __CONTROLS_H__
#define __CONTROLS_H__

#include "flight_controller.h"
#include "heli.h"
#include <irrlicht/irrlicht.h>
#include <vector>


const int CURVE_CACHE_SIZE = 250;


struct ControlsInput {
    float pitch_stick;
    float roll_stick;
    float yaw_stick;
    float throttle_stick;
};


class ControllerCurve {
public:
    struct Point {
        Point(float _in_level, float _out_level):in_level(_in_level), out_level(_out_level) {}
        float in_level;
        float out_level;
    };

    ControllerCurve(std::vector<Point> points);

    float translate(float level) const;
private:

    void add_line_to_cahce(const Point &point_from, const Point &point_to);
    float m_cache[CURVE_CACHE_SIZE];
};


class ControlsView {
public:
    ControlsView(
            irr::video::IVideoDriver *driver,
            std::vector<ControllerCurve> throttle_curves,
            std::vector<ControllerCurve> lift_curves
    );

    void update_ui(
            const ControlsInput &user_input,
            const ServoData &before_controller,
            const ServoData &after_controller,
            int active_curve_index
    );

private:
    irr::video::ITexture *m_controls_image;
    irr::video::ITexture *m_pin_image;
    irr::video::ITexture *m_second_pin_image;
    irr::video::ITexture *m_yaw_image;
    std::vector<irr::video::ITexture *> m_curves_images;
    irr::video::ITexture *m_curves_vertical_line;
    irr::video::IVideoDriver* m_driver;

    std::vector<ControllerCurve> m_throttle_curves;
    std::vector<ControllerCurve>  m_lift_curves;
};


class Controls {
public:
    Controls(FlightController *flight_controller,
             std::vector<ControllerCurve> throttle_curves,
             std::vector<ControllerCurve> lift,
             irr::video::IVideoDriver *driver);

    ServoData get_servo_data(const ControlsInput &input, float time_delta);

    void update_ui();

private:
    FlightController *m_flight_controller;
    std::vector<ControllerCurve> m_throttle_curves;
    std::vector<ControllerCurve> m_lift_curves;
    int m_active_curve;

    // Saved for UI.
    ControlsInput m_user_input;
    ServoData m_before_flight_controller;
    ServoData m_after_flight_controller;

    // UI stuff.
    irr::video::IVideoDriver *m_driver;
    ControlsView m_controls_view;
};


#endif // __CONTROLS_H__
