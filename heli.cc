#include "heli.h"
#include <iostream>

double GRAVITY_CONSTANT = 9.8;

Heli::Heli(const HeliParams &params, irr::scene::ISceneManager* smgr,
	     irr::video::IVideoDriver* driver)
{
	irr::scene::IMesh* heli_mesh = smgr->getMesh(params.shape_path.c_str());
	m_node = smgr->addMeshSceneNode(heli_mesh);
    m_node->setScale(params.shape_scale);
    m_node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
    m_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, false);
    m_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_node->setMaterialTexture(0, driver->getTexture(params.texture_path.c_str()));

    m_pos = params.init_pos;
    m_v = irrvec3(0, 0, 0);
    m_rotation.setRotationDegrees(params.init_rotation);
    m_shape_rotation.setRotationDegrees(params.shape_rotation);
    m_swash_sensitivity = params.swash_sensitivity;
    m_yaw_sensitivity = params.yaw_sensitivity;
    m_mass = params.mass;
    m_max_lift = params.max_lift;
    m_drag_vec = params.drag;

    update_ui();
}

void Heli::update_ui() {
    m_node->setPosition(m_pos);
    m_node->setRotation((m_rotation * m_shape_rotation).getRotationDegrees());
}

void Heli::update(double time_delta, 
                  const irrvec3 &wind_speed,
                  const ServoData &servo_data)
{
    // Update angular velocity according to servos.
    irrvec3 angular_v(
        servo_data.pitch * m_swash_sensitivity,
        servo_data.yaw * m_yaw_sensitivity,
        servo_data.roll * m_swash_sensitivity
    );
    
    irr::core::matrix4 d_rotation;
    d_rotation.setRotationDegrees(angular_v * time_delta);
    m_rotation *= d_rotation;

    // Gravity is easy - always down.
    irrvec3 gravity(0, -GRAVITY_CONSTANT, 0);

    // Lift is up in the heli axis;
    irrvec3 heli_up(0, 1, 0);
    m_rotation.rotateVect(heli_up);
    irrvec3 lift = heli_up * servo_data.lift * m_max_lift;
    std::cout << "Lift: (" << lift.X << ", " << lift.Y << ", " << lift.Z << ")" << std::endl;

    // Aerodynamic force.
    irrvec3 airspeed = m_v - wind_speed;
    irr::core::matrix4 world_to_heli = m_rotation.getTransposed();
    world_to_heli.rotateVect(airspeed);  // In heli coord system.
    irrvec3 aerodynamic_drag = m_drag_vec * airspeed;
    m_rotation.rotateVect(aerodynamic_drag);  // Back in world coord system.
    std::cout << "airspeed: (" << airspeed.X << ", " << airspeed.Y << ", " << airspeed.Z << ")" << std::endl;
    std::cout << "Aerodynamic drag: (" << aerodynamic_drag.X << ", " << aerodynamic_drag.Y << ", " << aerodynamic_drag.Z << ")" << std::endl;

    irrvec3 total_force = gravity + lift - aerodynamic_drag;
    irrvec3 acc = total_force / m_mass;
    m_v += time_delta * acc;
    m_pos += time_delta * m_v;
}
