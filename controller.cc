#include "controller.h"
#include <iostream>


ServoData TailGyroController::updateServoData(const ServoData& servo_data, float time_delta) {
    m_heli_yaw += m_heli->get_yaw_angularv() * time_delta;
    m_wanted_yaw += servo_data.yaw * time_delta * 6;
    ServoData new_servo_data = servo_data;
    float error = (m_wanted_yaw - m_heli_yaw);
    float derror = (error - m_prev_error) / time_delta;
    m_prev_error = error;

    new_servo_data.yaw = error*5 + derror/5;
    return new_servo_data;
}
