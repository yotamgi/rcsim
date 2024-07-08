#ifndef __HELI_H__
#define __HELI_H__

#include <string>
#include <irrlicht/irrlicht.h>
#include <memory>

typedef irr::core::vector3df irrvec3;


struct HeliParams {
    std::string shape_path;
    std::string texture_path;
    irrvec3 shape_rotation;
    irrvec3 shape_scale;
    irrvec3 init_pos;
    irrvec3 init_rotation;

    double swash_sensitivity;  //  [degree / sec]
    double yaw_sensitivity;  //  [degree / sec]
    double mass;  //  [Kg]
    double max_lift;  //  [N]
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


class Heli {
public:

    Heli(const HeliParams &params,
         irr::scene::ISceneManager* smgr,
	     irr::video::IVideoDriver* driver);

    void update(double time_delta, 
                const irrvec3 &wind_speed,
                const ServoData &servo_data);
    void update_ui();

    irrvec3 get_position() const {return m_pos;}
    void set_position(const irrvec3 new_pos) { m_pos = new_pos; }
    irrvec3 get_velocity() const {return m_v;}
    void set_velocity(const irrvec3 new_v) { m_v = new_v; }

private:
    irrvec3 m_v;
    irrvec3 m_pos;
    irr::core::matrix4 m_rotation;
    irr::core::matrix4 m_shape_rotation;
	irr::scene::IMeshSceneNode* m_node;

    // From params.
    double m_swash_sensitivity;
    double m_yaw_sensitivity;
    double m_mass;
    double m_max_lift;
};


#endif
