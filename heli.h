#ifndef __HELI_H__
#define __HELI_H__

#include "smooth_rand.h"
#include "rotor_blur.h"
#include <string>
#include <irrlicht/irrlicht.h>
#include <memory>
#include <vector>


typedef irr::core::vector3df irrvec3;


struct HeliParams {
    irrvec3 init_pos;
    irrvec3 init_rotation;

    double mass;  //  [Kg]
    double max_lift;  //  [N]
    irrvec3 drag;  // [N / (M / SEC)]
    float torbulant_airspeed;  // [M / SEC]

    float main_rotor_max_vel;  // [Rotations / SEC]
    float main_rotor_torque;  //  [N * M^2]
    float main_rotor_length;  // [M]
    float main_rotor_max_angle_of_attack;  // [degrees]

    float tail_length;  // [M]
    float tail_drag;  // Tail drag to apply yaw torque on airspeed [Newton / [M / SEC]]

    double swash_torque;  //  The swash moment strength [N * M]
    double yaw_torque;  //  The yaw moment strength [N * M]
    float rotor_moment_of_inertia;  // [Kg * M] around rotor rotating axis.
    irrvec3 body_moment_of_inertia;  //  Moment of inertia [Kg * M] coefficients around center of Mass.

    float rigidness;  // [rad / [torque]] A number describing torque applied to keep the heli
                      //                   rotor connected
    float anti_wobliness;  // [[torque] / [M/Sec]] A number describing the helicopter deformation
                           // friction, e.g. the eneregy loss due to non-rigid movements.
    std::vector<irrvec3> touchpoints_in_heli;

    float external_torque_limit;  // [torque] Maximum external torque allowed on the helicopter.
                                  //          High value here may cause numerical instability when
                                  //          the heli touches the ground.
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
    irrvec3 get_gyro_angularv() const {return m_body_angularv_in_body_coords; }

    void add_force(unsigned int touchpoints_index, const irrvec3 &force);
    void reset_force() { m_external_force = irrvec3();  m_external_torque = irrvec3(); }

    struct TouchPoint {
        irrvec3 pos_in_world;
        irrvec3 vel_in_world;
    };

    std::vector<TouchPoint> get_touchpoints_in_world();

protected:
    virtual void update_ui(float time_delta) = 0;
    void update_body_moments(float time_delta,
                             const irrvec3 &moment_in_world);
    void update_rotor_moments(float time_delta,
                              const irrvec3 &moment_in_world);
    void update_moments(float time_delta,
                        const irrvec3 &wind_speed);

    class ServoFilter {
    public:
        ServoFilter(float max_rps):m_max_rps(max_rps),m_current_status(0) {}
        float update(float value, float time_delta);
        float get() const { return m_current_status; }
    private:
        float m_max_rps;
        float m_current_status;
    };

    irrvec3 m_v;
    irrvec3 m_pos;
    SmoothRandFloat torbulant_rand;
    float m_main_rotor_vel;
    irrvec3 m_external_torque;
    irrvec3 m_external_force;

    // Body and rotor orientation properties.
    irrvec3 m_rotor_angular_momentum_in_world;
    irr::core::matrix4 m_rotor_rotation;
    irr::core::matrix4 m_body_rotation;
    irrvec3 m_body_angularv_in_body_coords;
    irrvec3 m_prev_reaction_in_body;

    // To avoid recalculation, some forces are stored:
    float m_lift_force;
    float m_tail_rotor_force;

    HeliParams m_params;

    // Servos
    ServoFilter m_pitch_servo;
    ServoFilter m_roll_servo;
    ServoFilter m_yaw_servo;
    ServoFilter m_lift_servo;
    ServoFilter m_throttle_servo;
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
