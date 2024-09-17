#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "heli.h"

class Controller {
public:
    Controller(const BaseHeli *heli):m_heli(heli) {}
    virtual ServoData updateServoData(const ServoData& servo_data, float time_delta) = 0;
protected:
    const BaseHeli *m_heli;
};


class TailGyroController : public Controller {
public:
    TailGyroController(const BaseHeli *heli):Controller(heli),m_heli_yaw(0),m_wanted_yaw(0),m_prev_error(0) {}
    virtual ServoData updateServoData(const ServoData& servo_data, float time_delta);
private:
    float m_heli_yaw;
    float m_wanted_yaw;
    float m_prev_error;
};

#endif // __CONTROLLER_H__
