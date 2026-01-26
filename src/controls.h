#ifndef __CONTROLS_H__
#define __CONTROLS_H__

// #include "flight_controller.h"
// #include "heli.h"
#include "airplane.h"
#include <memory>
#include <vector>

const int CURVE_CACHE_SIZE = 250;

typedef std::vector<float> ServoData;

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
    Point(float _in_level, float _out_level)
        : in_level(_in_level), out_level(_out_level) {}
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
  virtual ServoData get_servo_data(const ControlsInput &input,
                                   float time_delta) = 0;

  struct Telemetry {
    ControlsInput user_input;
    ServoData before_controller;
    ServoData after_controller;
    int active_curve_index;
  };
  virtual Telemetry get_telemetry() = 0;
};

// class HeliControls : public Controls {
// public:
//   HeliControls(std::shared_ptr<HeliFlightController> flight_controller,
//                std::vector<ControllerCurve> throttle_curves,
//                std::vector<ControllerCurve> lift_curves);

//   // Implementation of the Controls interface.
//   virtual ServoData get_servo_data(const ControlsInput &input,
//                                    float time_delta);
//   virtual Controls::Telemetry get_telemetry();

// private:
//   std::shared_ptr<HeliFlightController> m_flight_controller;
//   std::vector<ControllerCurve> m_throttle_curves;
//   std::vector<ControllerCurve> m_lift_curves;

//   // Saved for Telemetry.
//   ControlsInput m_user_input;
//   ServoData m_before_flight_controller;
//   ServoData m_after_flight_controller;
// };

class AirplaneControls : public Controls {
public:
  AirplaneControls(bool use_flapron): m_use_flapron(use_flapron) {}

  virtual ServoData get_servo_data(const ControlsInput &input,
                                   float time_delta) override;
  virtual Controls::Telemetry get_telemetry() override;

private:
  bool m_use_flapron;

  // Used for telemetry.
  ControlsInput m_user_input;
  ServoData m_output_servo_data;
};

#endif // __CONTROLS_H__
