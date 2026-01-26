#include "flight_controller.h"
#include "raylib_engine.h"
#include <iostream>

std::vector<float>
HeliFlightController::translate(const std::vector<float> &servo_data,
                                float time_delta) {

  // If the throttle is low, set all the servos to 0.
  if (servo_data[HELI_CHANNEL_THROTTLE] < -0.7) {
    std::vector<float> zero_servo_data = servo_data;
    for (float &x : zero_servo_data)
      x = 0;
    zero_servo_data[HELI_CHANNEL_THROTTLE] = servo_data[HELI_CHANNEL_THROTTLE];
    return zero_servo_data;
  }

  // Otherwise, apply PID.
  float pitch = servo_data[HELI_CHANNEL_PITCH];
  float roll = servo_data[HELI_CHANNEL_ROLL];
  float yaw = servo_data[HELI_CHANNEL_YAW];
  m_heli_angles += m_heli->get_gyro_angularv() * time_delta;
  m_wanted_angles +=
      engine::vec3(pitch, yaw, roll) * time_delta * engine::vec3(8, 8, 6);
  engine::vec3 error = (m_wanted_angles - m_heli_angles);
  engine::vec3 derror = (error - m_prev_error) / time_delta;
  m_prev_error = error;
  m_error_integral = 0.8 * m_error_integral + error * time_delta;

  engine::vec3 controls = error * engine::vec3(1, 1e-2, 1) +
                          derror * engine::vec3(5e-2, 3e-1, 5e-2) +
                          m_error_integral * engine::vec3(2.5, 30, 10.);
  std::vector<float> new_servo_data = servo_data;
  new_servo_data[HELI_CHANNEL_YAW] = controls.y;
  if (m_six_axis) {
    new_servo_data[HELI_CHANNEL_PITCH] = controls.x;
    new_servo_data[HELI_CHANNEL_ROLL] = controls.z;
  } else {
    m_wanted_angles.x = m_heli_angles.x;
    m_wanted_angles.z = m_heli_angles.z;
  }
  return new_servo_data;
}
