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

    double mass;  //  [Kg]
    double max_lift;  //  [N]
    irrvec3 drag;  // [N / (M / SEC)]
    float torbulant_airspeed;  // [M / SEC]

    float main_rotor_max_vel;  // [Rotations / SEC]
    double main_rotor_acc;  // [Rotations / SEC / SEC]

    float tail_length;  // [Meters]
    float tail_drag;  // Tail drag to apply yaw torque on airspeed [Newton / [M / SEC]]

    double swash_torque;  //  The swash moment strength [N * M]
    double yaw_torque;  //  The yaw moment strength [N * M]
    float rotor_moment_of_inertia;  // [Kg * M] around rotor rotating axis.
    irrvec3 body_moment_of_inertia;  //  Moment of inertia [Kg * M] coefficients around center of Mass.
};

/**
 * Servo data, from -1 to 1.
 */
struct ServoData {
    double roll;
    double pitch;
    double yaw;
    double lift;
    double throttle;
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
    float get_yaw_angularv() const {return m_yaw_angularv; }

protected:
    virtual void update_ui(float time_delta) = 0;
    irrvec3 get_torques_in_body(float time_delta,
                                const irrvec3 &wind_speed,
                                const ServoData &servo_data);
    void update_moments(float time_delta,
                        const irrvec3 &wind_speed,
                        const ServoData &servo_data);

    irrvec3 m_v;
    irrvec3 m_pos;
    irrvec3 m_angular_momentum;
    float m_yaw_angularv;
    irr::core::matrix4 m_rotation;
    SmoothRandFloat torbulant_rand;
    float m_main_rotor_vel;

    HeliParams m_params;
};


class BellHeli : public BaseHeli {
public:
    BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver);
private:
    virtual void update_ui(float time_delta);

    irr::core::matrix4 m_shape_rotation;
	irr::scene::IMeshSceneNode* m_body_node;
	irr::scene::IMeshSceneNode* m_rotor_node;
	irr::scene::IMeshSceneNode* m_tail_rotor_node;
    std::shared_ptr<RotorBlur> m_main_rotor_blur;
    std::shared_ptr<RotorBlur> m_flybar_blur;
    std::shared_ptr<RotorBlur> m_tail_prop_blur;

    float m_main_rotor_angle;
    float m_tail_rotor_angle;
};


#endif
