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


class GyroController : public Controller {
public:
    GyroController(const BaseHeli *heli):Controller(heli),m_six_axis(true) {}
    virtual ServoData updateServoData(const ServoData& servo_data, float time_delta);

    void set_six_axis(bool set) { m_six_axis = set; }
private:
    irr::core::vector3df m_heli_angles;
    irr::core::vector3df m_wanted_angles;
    irr::core::vector3df m_prev_error;

    bool m_six_axis;
};

#endif // __CONTROLLER_H__
