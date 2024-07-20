#ifndef __HELI_H__
#define __HELI_H__

#include "smooth_rand.h"
#include "rotor_blur.h"
#include <string>
#include <irrlicht/irrlicht.h>
#include <memory>


typedef irr::core::vector3df irrvec3;


struct HeliParams {
    irrvec3 init_pos;
    irrvec3 init_rotation;
    double swash_sensitivity;  //  [degree / sec]
    double yaw_sensitivity;  //  [degree / sec]
    double mass;  //  [Kg]
    double max_lift;  //  [N]
    irrvec3 drag;  // [N / (M / SEC)]
    float torbulant_airspeed;  // [M / SEC]
};

/**
 * Servo data, from -1 to 1.
 */
struct ServoData {
    double roll;
    double pitch;
    double yaw;
    double lift;
};


class BaseHeli {
public:

    BaseHeli(const HeliParams &params);

    void update(double time_delta, 
                const irrvec3 &wind_speed,
                const ServoData &servo_data);
    irrvec3 get_position() const {return m_pos;}
    void set_position(const irrvec3 new_pos) { m_pos = new_pos; }
    irrvec3 get_velocity() const {return m_v;}
    void set_velocity(const irrvec3 new_v) { m_v = new_v; }

protected:
    virtual void update_ui() = 0;

    irrvec3 m_v;
    irrvec3 m_pos;
    irr::core::matrix4 m_rotation;
    SmoothRandFloat torbulant_rand;

    // From params.
    double m_swash_sensitivity;
    double m_yaw_sensitivity;
    double m_mass;
    double m_max_lift;
    double m_torbulant_airspeed;
    irrvec3 m_drag_vec;
};


class BellHeli : public BaseHeli {
public:
    BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver);
private:
    virtual void update_ui();

    irr::core::matrix4 m_shape_rotation;
	irr::scene::IMeshSceneNode* m_node;
    std::shared_ptr<RotorBlur> m_main_rotor_blur;
    std::shared_ptr<RotorBlur> m_flybar_blur;
    std::shared_ptr<RotorBlur> m_tail_prop_blur;
};


#endif
