#include "flight_controller.h"
#include <iostream>

typedef irr::core::vector3df irrvec;

std::vector<float>
HeliFlightController::translate(const std::vector<float> &servo_data,
                                float time_delta) {

  // If the throttle is low, set all the servos to 0.
  if (servo_data[HELI_CHANNEL_THROTTLE] < -0.7) {
    std::vector<float> zero_servo_data = servo_data;
    for (float &x : zero_servo_data) x = 0;
    zero_servo_data[HELI_CHANNEL_THROTTLE] =
        servo_data[HELI_CHANNEL_THROTTLE];
    return zero_servo_data;
  }

  // Otherwise, apply PID.
  float pitch = servo_data[HELI_CHANNEL_PITCH];
  float roll = servo_data[HELI_CHANNEL_ROLL];
  float yaw = servo_data[HELI_CHANNEL_YAW];
  m_heli_angles += m_heli->get_gyro_angularv() * time_delta;
  m_wanted_angles += irrvec3(pitch, yaw, roll) * time_delta * irrvec3(8, 8, 6);
  irrvec3 error = (m_wanted_angles - m_heli_angles);
  irrvec3 derror = (error - m_prev_error) / time_delta;
  m_prev_error = error;
  m_error_integral = 0.8 * m_error_integral + error * time_delta;

  irrvec3 controls = error * irrvec3(1, 1e-2, 1) +
                     derror * irrvec3(5e-2, 3e-1, 5e-2) +
                     m_error_integral * irrvec3(2.5, 30, 10.);

  std::vector<float> new_servo_data = servo_data;
  new_servo_data[HELI_CHANNEL_YAW] = controls.Y;
  if (m_six_axis) {
    new_servo_data[HELI_CHANNEL_PITCH] = controls.X;
    new_servo_data[HELI_CHANNEL_ROLL] = controls.Z;
  } else {
    m_wanted_angles.X = m_heli_angles.X;
    m_wanted_angles.Z = m_heli_angles.Z;
  }
  return new_servo_data;
}
