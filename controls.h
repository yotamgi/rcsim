#ifndef __CONTROLS_H__
#define __CONTROLS_H__

#include "flight_controller.h"
#include "heli.h"
#include <vector>


const int CURVE_CACHE_SIZE = 250;


struct ControlsInput {
    float pitch_stick;
    float roll_stick;
    float yaw_stick;
    float throttle_stick;
    size_t active_curve_index;
    bool gyro_6dof;
    bool throttle_hold;
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


class Controls {
public:
    Controls(FlightController *flight_controller,
             std::vector<ControllerCurve> throttle_curves,
             std::vector<ControllerCurve> lift_curves);

    ServoData get_servo_data(const ControlsInput &input, float time_delta);

    struct Telemetry {
        ControlsInput user_input;
        ServoData before_controller;
        ServoData after_controller;
        int active_curve_index;
    };
    Telemetry get_telemetry();

    std::vector<ControllerCurve> get_throttle_curves() { return m_throttle_curves; }
    std::vector<ControllerCurve> get_lift_curves() { return m_lift_curves; }

private:
    FlightController *m_flight_controller;
    std::vector<ControllerCurve> m_throttle_curves;
    std::vector<ControllerCurve> m_lift_curves;

    // Saved for Telemetry.
    ControlsInput m_user_input;
    ServoData m_before_flight_controller;
    ServoData m_after_flight_controller;
};


#endif // __CONTROLS_H__
