#include "flight_controller.h"
#include <iostream>

typedef irr::core::vector3df irrvec;


ServoData GyroFlightController::translate(const ServoData& servo_data, float time_delta) {

    // If the throttle is low, set all the servos to 0.
    if (servo_data.throttle < -0.7) {
        ServoData zero_servo_data;
        memset((void *)&zero_servo_data, 0, sizeof(ServoData));
        zero_servo_data.throttle = servo_data.throttle;
        return zero_servo_data;
    }

    // Otherwise, apply PID.
    m_heli_angles += m_heli->get_gyro_angularv() * time_delta;
    m_wanted_angles += irrvec3(servo_data.pitch, servo_data.yaw, servo_data.roll) * time_delta * irrvec3(8, 8, 6);
    irrvec3 error = (m_wanted_angles - m_heli_angles);
    irrvec3 derror = (error - m_prev_error) / time_delta;
    m_prev_error = error;
    m_error_integral = 0.8*m_error_integral + error * time_delta;

    irrvec3 controls = error*irrvec3(1, 1e-2, 1)
                       + derror*irrvec3(5e-2, 3e-1, 5e-2)
                       + m_error_integral*irrvec3(2.5, 30, 10.);

    ServoData new_servo_data = servo_data;
    new_servo_data.yaw = controls.Y;
    if (m_six_axis) {
        new_servo_data.pitch = controls.X;
        new_servo_data.roll = controls.Z;
    } else {
        m_wanted_angles.X = m_heli_angles.X;
        m_wanted_angles.Z = m_heli_angles.Z;
    }
    return new_servo_data;
}
