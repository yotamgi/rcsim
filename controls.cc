#include "controls.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////
// ControllerCurve methods.
////////////////////////////////////////////////////////////////////////////////

static inline int in_level_to_cache_index(float level) {
  return int((level + 1.0) / 2. * (CURVE_CACHE_SIZE - 1));
}

void ControllerCurve::add_line_to_cahce(const Point &point_from,
                                        const Point &point_to) {
  float grad = (point_from.out_level - point_to.out_level) /
               (point_from.in_level - point_to.in_level);
  int cache_from = in_level_to_cache_index(point_from.in_level);
  int cache_to = in_level_to_cache_index(point_to.in_level);
  for (int i = cache_from; i < cache_to + 1; i++) {
    float curr_ratio = float(i - cache_from) / (cache_to - cache_from);
    m_cache[i] = point_from.out_level +
                 curr_ratio * (point_to.in_level - point_from.in_level) * grad;
  }
}

ControllerCurve::ControllerCurve(std::vector<Point> points) {
  Point prev_point(-1.0, points.front().out_level);

  for (Point point : points) {
    add_line_to_cahce(prev_point, point);
    prev_point = point;
  }
  if (points.back().in_level != 1.0) {
    add_line_to_cahce(points.back(), Point(1.0, points.back().out_level));
  }
}

float ControllerCurve::translate(float level) const {
  level = (level > 1) ? 1 : level;
  level = (level < -1) ? -1 : level;
  return m_cache[in_level_to_cache_index(level)];
}

////////////////////////////////////////////////////////////////////////////////
// Controls methods.
////////////////////////////////////////////////////////////////////////////////

HeliControls::HeliControls(
    std::shared_ptr<HeliFlightController> flight_controller,
    std::vector<ControllerCurve> throttle_curves,
    std::vector<ControllerCurve> lift_curves)
    : m_flight_controller(flight_controller),
      m_throttle_curves(throttle_curves), m_lift_curves(lift_curves),
      m_before_flight_controller(5) {}

ServoData HeliControls::get_servo_data(const ControlsInput &input,
                                                float time_delta) {
  m_user_input = input;
  m_before_flight_controller[HELI_CHANNEL_PITCH] = input.pitch_stick;
  m_before_flight_controller[HELI_CHANNEL_ROLL] = input.roll_stick;
  m_before_flight_controller[HELI_CHANNEL_YAW] = input.yaw_stick;
  m_before_flight_controller[HELI_CHANNEL_LIFT] =
      m_lift_curves[input.active_curve_index].translate(input.throttle_stick);
  m_before_flight_controller[HELI_CHANNEL_THROTTLE] =
      m_throttle_curves[input.active_curve_index].translate(
          input.throttle_stick);

  m_flight_controller->set_six_axis(input.gyro_6dof);
  m_after_flight_controller =
      m_flight_controller->translate(m_before_flight_controller, time_delta);
  if (input.throttle_hold) {
    m_after_flight_controller[HELI_CHANNEL_THROTTLE] = -1.;
  }
  return m_after_flight_controller;
}

Controls::Telemetry HeliControls::get_telemetry() {
  Controls::Telemetry telemetry;
  telemetry.user_input = m_user_input;
  telemetry.before_controller = m_before_flight_controller;
  telemetry.after_controller = m_after_flight_controller;
  telemetry.active_curve_index = m_user_input.active_curve_index;
  return telemetry;
}
