#include "controller.h"
#include <iostream>

typedef irr::core::vector3df irrvec;


ServoData GyroController::updateServoData(const ServoData& servo_data, float time_delta) {
    m_heli_angles += m_heli->get_gyro_angularv() * time_delta;
    m_wanted_angles += irrvec3(servo_data.pitch, servo_data.yaw, servo_data.roll) * time_delta * irrvec3(3, 4, 3);
    irrvec3 error = (m_wanted_angles - m_heli_angles);
    irrvec3 derror = (error - m_prev_error) / time_delta;
    m_prev_error = error;

    irrvec3 controls = error*irrvec3(5, 5, 5) + derror*irrvec3(1e-3, 1e-1, 1e-3);

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
