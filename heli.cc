#include "heli.h"
#include "smooth_rand.h"
#include <iostream>
#include <cmath>

double GRAVITY_CONSTANT = 9.8;

float norm(irrvec3 vec) {
    return std::sqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
}


BaseHeli::BaseHeli(const HeliParams &params):
         torbulant_rand(3., 1)
{
    m_pos = params.init_pos;
    m_v = irrvec3(0, 0, 0);
    m_rotation.setRotationDegrees(params.init_rotation);
    m_swash_sensitivity = params.swash_sensitivity;
    m_yaw_sensitivity = params.yaw_sensitivity;
    m_mass = params.mass;
    m_max_lift = params.max_lift;
    m_drag_vec = params.drag;
    m_torbulant_airspeed = params.torbulant_airspeed;
}

void BaseHeli::update(double time_delta, 
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

    update_ui(time_delta);
}

class MainRotorBlur : public RotorBlur {
public:
    MainRotorBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            3.75,  // radius.
            irrvec3(-0.45, 1.8, 0),  // position.
            irrvec3(0, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        if (along_radius < 0.2) return 0.2;
        return 3.;
    }
    virtual irr::video::SColor get_color(float along_radious) const {
        if (along_radious < 0.2) return irr::video::SColor(255, 128, 128, 128);
        if (along_radious > 3.52 && along_radious < 3.6)
            return irr::video::SColor(255, 255, 255, 255);
        else return irr::video::SColor(255, 0, 0, 0);
    }
};


class FlybarBlur : public RotorBlur {
public:
    FlybarBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            0.62,  // radius.
            irrvec3(-0.45, 2, 0),  // position.
            irrvec3(0, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        if (along_radius > 0.57)
            return 0.7;
        else if (along_radius < 0.3)
            return 0.7;
        else return 0.5;
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
            0.66,  // radius.
            irrvec3(-4.62, 1.54, -0.14),  // position.
            irrvec3(90, 0, 0),  // rotation.
            0.03,  // thichkess.
            parent  // parent_node
        );
    }

private:
    virtual float get_width(float along_radius) const  {
        return 0.4;
    }
    virtual irr::video::SColor get_color(float along_radious) const {
        if (along_radious > 0.53 && along_radious < 0.64)
            return irr::video::SColor(255, 255, 255, 255);
        return irr::video::SColor(255, 0, 0, 0);
    }
};



const struct HeliParams BELL_AERODYNAMICS = {
    .init_pos = irrvec3(0, 3, 0),
    .init_rotation = irrvec3(0, 0, 0),
    .swash_sensitivity = 150.,
    .yaw_sensitivity = 150.,
    .mass = 0.5,
    .max_lift = 0.5 * 10 * 5,
    .drag = irrvec3(0.5, 4, 0.1),
    .torbulant_airspeed = 7,
};


BellHeli::BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver):
    BaseHeli(BELL_AERODYNAMICS)
{
    // Create the body mesh.
	irr::scene::IMesh* heli_mesh = smgr->getMesh("media/Bell/source/bell_body.obj");
	m_body_node = smgr->addMeshSceneNode(heli_mesh);
    m_body_node->setScale(irrvec3(0.5, 0.5, 0.5));
    m_body_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    m_body_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_body_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_body_node->setMaterialTexture(0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
    m_body_node->addShadowVolumeSceneNode();
    for (unsigned int i=0; i < m_body_node->getMaterialCount(); i++) {
        m_body_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    // Create the main rotor mesh.
	irr::scene::IMesh* main_rotor_mesh = smgr->getMesh("media/Bell/source/bell_main_rotor.obj");
    irr::core::matrix4 main_rotor_translation;
    main_rotor_translation.setTranslation(irrvec3(0.45, 0, 0));
    smgr->getMeshManipulator()->apply(
        irr::scene::SVertexPositionTransformManipulator(main_rotor_translation), main_rotor_mesh);
	m_rotor_node = smgr->addMeshSceneNode(main_rotor_mesh);
    m_rotor_node->setScale(irrvec3(0.5, 0.5, 0.5));
    m_rotor_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    m_rotor_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_rotor_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_rotor_node->setMaterialTexture(0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
    m_rotor_node->addShadowVolumeSceneNode();
    for (unsigned int i=0; i < m_rotor_node->getMaterialCount(); i++) {
        m_rotor_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    // Create the tail rotor mesh.
	irr::scene::IMesh* tail_rotor_mesh = smgr->getMesh("media/Bell/source/bell_tail_rotor.obj");
    irr::core::matrix4 tail_rotor_translation;
    tail_rotor_translation.setTranslation(irrvec3(4.62, -1.54, 0));
    smgr->getMeshManipulator()->apply(
        irr::scene::SVertexPositionTransformManipulator(tail_rotor_translation), tail_rotor_mesh);
	m_tail_rotor_node = smgr->addMeshSceneNode(tail_rotor_mesh);
    m_tail_rotor_node->setScale(irrvec3(0.5, 0.5, 0.5));
    m_tail_rotor_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    m_tail_rotor_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_tail_rotor_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_tail_rotor_node->setMaterialTexture(0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
    m_tail_rotor_node->addShadowVolumeSceneNode();
    for (unsigned int i=0; i < m_tail_rotor_node->getMaterialCount(); i++) {
        m_tail_rotor_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    m_shape_rotation.setRotationDegrees(irrvec3(0, -90, 0));

    m_main_rotor_blur = std::make_shared<MainRotorBlur>(smgr, m_body_node);
    m_flybar_blur = std::make_shared<FlybarBlur>(smgr, m_body_node);
    m_tail_prop_blur = std::make_shared<TailRotorBlur>(smgr, m_body_node);

    update_ui(0);
}

void BellHeli::update_ui(float time_delta) {
    m_body_node->setPosition(m_pos);
    m_tail_rotor_node->setPosition(m_pos);

    m_body_node->setRotation((m_rotation * m_shape_rotation).getRotationDegrees());
    m_tail_rotor_node->setRotation((m_rotation * m_shape_rotation).getRotationDegrees());

    // Set the main rotor rotation.
    irr::core::matrix4 main_rotor_rotation;
    main_rotor_rotation.setRotationDegrees(irrvec3(0, m_main_rotor_angle, 0));
    m_main_rotor_angle += 500 * time_delta;
    irrvec3 main_rotor_offset(0, 0, 0.225);
    (m_rotation).rotateVect(main_rotor_offset);
    m_rotor_node->setPosition(m_pos - main_rotor_offset);
    m_rotor_node->setRotation((m_rotation * m_shape_rotation * main_rotor_rotation).getRotationDegrees());

    // Set the tail rotor rotation.
    irr::core::matrix4 tail_rotor_rotation;
    tail_rotor_rotation.setRotationDegrees(irrvec3(0, 0, m_tail_rotor_angle));
    m_tail_rotor_angle += 500 * time_delta;
    irrvec3 tail_rotor_offset(0, -0.77, 2.31);
    (m_rotation).rotateVect(tail_rotor_offset);
    m_tail_rotor_node->setPosition(m_pos - tail_rotor_offset);
    m_tail_rotor_node->setRotation((m_rotation * m_shape_rotation * tail_rotor_rotation).getRotationDegrees());
}

