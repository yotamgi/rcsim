#include "heli.h"
#include "smooth_rand.h"
#include <iostream>
#include <cmath>

double GRAVITY_CONSTANT = 9.8;

float norm(irrvec3 vec) {
    return std::sqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
}


class MainRotorBlur : public RotorBlur {
public:
    MainRotorBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            375,  // radius.
            irrvec3(-45, 180, 0),  // position.
            irrvec3(0, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        if (along_radius < 20) return 20;
        return 300;
    }
    virtual irr::video::SColor get_color(float along_radious) const {
        if (along_radious < 20) return irr::video::SColor(255, 128, 128, 128);
        if (along_radious > 352 && along_radious < 360)
            return irr::video::SColor(255, 255, 255, 255);
        else return irr::video::SColor(255, 0, 0, 0);
    }
};


class FlybarBlur : public RotorBlur {
public:
    FlybarBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            62,  // radius.
            irrvec3(-45, 200, 0),  // position.
            irrvec3(0, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        if (along_radius > 57)
            return 70;
        else if (along_radius < 30)
            return 70;
        else return 50;
    }
    virtual irr::video::SColor get_color(float along_radious) const {
        return irr::video::SColor(255, 128, 128, 128);
    }
};


class TailRotorBlur : public RotorBlur {
public:
    TailRotorBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            66,  // radius.
            irrvec3(-462, 154, -14),  // position.
            irrvec3(90, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        return 40;
    }
    virtual irr::video::SColor get_color(float along_radious) const {
        if (along_radious > 53 && along_radious < 64)
            return irr::video::SColor(255, 255, 255, 255);
        return irr::video::SColor(255, 0, 0, 0);
    }
};


Heli::Heli(const HeliParams &params, irr::scene::ISceneManager* smgr,
	     irr::video::IVideoDriver* driver):
         torbulant_rand(3., 1)
{
	irr::scene::IMesh* heli_mesh = smgr->getMesh(params.shape_path.c_str());
	m_node = smgr->addMeshSceneNode(heli_mesh);
    m_node->setScale(params.shape_scale);
    m_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    m_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_node->setMaterialTexture(0, driver->getTexture(params.texture_path.c_str()));
    m_node->addShadowVolumeSceneNode();
    for (unsigned int i=0; i < m_node->getMaterialCount(); i++) {
        m_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    m_main_rotor_blur = std::make_shared<MainRotorBlur>(smgr, m_node);
    m_flybar_blur = std::make_shared<FlybarBlur>(smgr, m_node);
    m_tail_prop_blur = std::make_shared<TailRotorBlur>(smgr, m_node);

    m_pos = params.init_pos;
    m_v = irrvec3(0, 0, 0);
    m_rotation.setRotationDegrees(params.init_rotation);
    m_shape_rotation.setRotationDegrees(params.shape_rotation);
    m_swash_sensitivity = params.swash_sensitivity;
    m_yaw_sensitivity = params.yaw_sensitivity;
    m_mass = params.mass;
    m_max_lift = params.max_lift;
    m_drag_vec = params.drag;
    m_torbulant_airspeed = params.torbulant_airspeed;

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

    // Account for torbulation in low airspeed.
    float torbulant_coeff = m_torbulant_airspeed - norm(airspeed);
    torbulant_coeff = torbulant_coeff > 1 ? 1 : torbulant_coeff;
    torbulant_coeff = torbulant_coeff < 0 ? 0 : torbulant_coeff;
    torbulant_coeff *= torbulant_rand.update(time_delta);
    float lift_torbulant_effect = 1 - torbulant_coeff / 2;
    std::cout << "Lift torbulant effect: " << lift_torbulant_effect << std::endl;
    lift *= lift_torbulant_effect;

    irrvec3 total_force = gravity + lift - aerodynamic_drag;
    irrvec3 acc = total_force / m_mass;
    m_v += time_delta * acc;
    m_pos += time_delta * m_v;
}
