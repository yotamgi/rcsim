#include "controller.h"
#include <iostream>


ServoData TailGyroController::updateServoData(const ServoData& servo_data, float time_delta) {
    m_heli_yaw += m_heli->get_gyro_angularv().Y * time_delta;
    m_wanted_yaw += servo_data.yaw * time_delta * 6;
    float error = (m_wanted_yaw - m_heli_yaw);
    float derror = (error - m_prev_error) / time_delta;
    m_prev_error = error;

    ServoData new_servo_data = servo_data;
    new_servo_data.yaw = error*5 + derror/25;
    return new_servo_data;
}
