#include "heli.h"
#include "smooth_rand.h"
#include <iostream>
#include <cmath>
#include <iomanip>

using irr::core::PI;

double GRAVITY_CONSTANT = 9.8;

float norm(irrvec3 vec) {
    return std::sqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
}


BaseHeli::BaseHeli(const HeliParams &params):
         torbulant_rand(3., 1),
         m_params(params)
{
    m_pos = m_params.init_pos;
    m_v = irrvec3(0, 0, 0);
    m_rotor_rotation.setRotationDegrees(m_params.init_rotation);
    m_main_rotor_vel = 0;
}


void print_matrix(const std::string str, const irr::core::matrix4 &mat) {
    std::cout << std::fixed << std::setprecision(3) << std::setw(5) << std::setfill(' ');
    std::cout << "Matrix " << str << ":" << std::endl;
    std::cout << mat(0, 0) << ", ";
    std::cout << mat(0, 1) << ", ";
    std::cout << mat(0, 2) << std::endl;

    std::cout << mat(1, 0) << ", ";
    std::cout << mat(1, 1) << ", ";
    std::cout << mat(1, 2) << std::endl;

    std::cout << mat(2, 0) << ", ";
    std::cout << mat(2, 1) << ", ";
    std::cout << mat(2, 2) << std::endl;
}

static void print_vec(const std::string str, const irrvec3 &vec) {
    std::cout << "Vec " << str << ": (";
    std::cout << vec.X << ", ";
    std::cout << vec.Y << ", ";
    std::cout << vec.Z << ")" << std::endl;;
}

void update_rotation_matrix(irr::core::matrix4 &matrix, const irrvec3 angularv) {

    // Extract rotation columns.
    irrvec3 x(matrix(0, 0), matrix(0, 1), matrix(0, 2));
    irrvec3 y(matrix(1, 0), matrix(1, 1), matrix(1, 2));
    irrvec3 z(matrix(2, 0), matrix(2, 1), matrix(2, 2));

    // Perform the inifinitisimal rotation.
    x += angularv.crossProduct(x);
    y += angularv.crossProduct(y);
    z += angularv.crossProduct(z);

    // Make sure it orthogonal.
    z = x.crossProduct(y);
    x = y.crossProduct(z);
    y = z.crossProduct(x);
    x.normalize();
    y.normalize();
    z.normalize();

    // Write columns back to the matrix.
    matrix(0, 0) = x.X; matrix(0, 1) = x.Y; matrix(0, 2) = x.Z;
    matrix(1, 0) = y.X; matrix(1, 1) = y.Y; matrix(1, 2) = y.Z;
    matrix(2, 0) = z.X; matrix(2, 1) = z.Y; matrix(2, 2) = z.Z;
}


static float clip_servo(float servo_data) {
    return std::max(-1.0f, std::min(servo_data, 1.0f));
}


void BaseHeli::update_body_moments(float time_delta, const irrvec3 &moment_in_world)
{
    irrvec3 moment_in_body;
    m_body_rotation.getTransposed().rotateVect(moment_in_body, moment_in_world);

    // Update body angular velocity according to Eurler's equation.
    m_body_angularv_in_body_coords += time_delta * (
            moment_in_body
            - m_body_angularv_in_body_coords.crossProduct(
                    m_params.body_moment_of_inertia * m_body_angularv_in_body_coords
            )
           - m_body_angularv_in_body_coords * m_params.anti_wobliness
    )  / m_params.body_moment_of_inertia;

    // Get the angularv in world coordinates.
    irrvec3 body_angularv;
    m_body_rotation.rotateVect(body_angularv,  m_body_angularv_in_body_coords);
    update_rotation_matrix(m_body_rotation, body_angularv * time_delta);
}

void BaseHeli::update_rotor_moments(float time_delta, const irrvec3 &moment_in_world)
{
    // Update rotor angular momentum by the torques.
    m_rotor_angular_momentum += time_delta * moment_in_world;
    irrvec3 rot_y = m_rotor_angular_momentum;
    rot_y.normalize();

    // Only at the first frame, the angular_momentum is 0 and it screws up everything.
    // This is a dirty workaround this.
    if (rot_y.X == 0 && rot_y.Y == 0 && rot_y.Z == 0) {
        rot_y = irrvec3(0, 1, 0);
    }

    // Make sure that the rotor doesn't change its angular velocity around Y axis.
    float rotor_omega = 2 * PI * m_main_rotor_vel / 360 + 0.01;
    m_rotor_angular_momentum = rot_y * m_params.rotor_moment_of_inertia * rotor_omega;

    // Update back the rotor rotation matrix;
    irrvec3 rot_z(m_body_rotation(2, 0), m_body_rotation(2, 1), m_body_rotation(2, 2));
    irrvec3 rot_x = rot_y.crossProduct(rot_z);
    rot_z = rot_x.crossProduct(rot_y);
    m_rotor_rotation(0, 0) = rot_x.X; m_rotor_rotation(0, 1) = rot_x.Y; m_rotor_rotation(0, 2) = rot_x.Z;
    m_rotor_rotation(1, 0) = rot_y.X; m_rotor_rotation(1, 1) = rot_y.Y; m_rotor_rotation(1, 2) = rot_y.Z;
    m_rotor_rotation(2, 0) = rot_z.X; m_rotor_rotation(2, 1) = rot_z.Y; m_rotor_rotation(2, 2) = rot_z.Z;
}


void BaseHeli::update_moments(float time_delta,
                              const irrvec3 &wind_speed,
                              const ServoData &servo_data)
{
    // Update angular velocity according to torques.
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    // Calculate the servos torque.
    // Note that the roll and pitch are switched due to the gyro 90 deg effect.
    irrvec3 swash_torque_in_rotor_coords(
        -clip_servo(servo_data.roll) * m_params.swash_torque,
        0,
        clip_servo(servo_data.pitch) * m_params.swash_torque
    );
    irrvec3 yaw_torque_in_rotor_coords(
        0, clip_servo(servo_data.yaw) * m_params.yaw_torque, 0
    );
    swash_torque_in_rotor_coords *= main_rotor_effectiveness;
    yaw_torque_in_rotor_coords *= main_rotor_effectiveness;
    irrvec3 swash_torque_in_world;
    m_rotor_rotation.rotateVect(swash_torque_in_world, swash_torque_in_rotor_coords);

    // calculate the tail-drag moment.
    irrvec3 heli_right_in_world(m_body_rotation(0, 0), m_body_rotation(0, 1), m_body_rotation(0, 2));
    irrvec3 airspeed_in_world = - m_v - wind_speed;
    float tail_wind = heli_right_in_world.dotProduct(airspeed_in_world);
    tail_wind += m_body_angularv_in_body_coords.Y * m_params.tail_length;
    float tail_drag_moment_y = -tail_wind * m_params.tail_drag * m_params.tail_length;

    // Calculate the total tail moments.
    irrvec3 total_tail_moment_in_body_coords(
        0, yaw_torque_in_rotor_coords.Y + tail_drag_moment_y, 0);
    irrvec3 total_tail_moment_in_world;
    m_body_rotation.rotateVect(total_tail_moment_in_world, total_tail_moment_in_body_coords);

    // Calculate the body reaction moment.
    irrvec3 body_up(m_body_rotation(1, 0), m_body_rotation(1, 1), m_body_rotation(1, 2));
    irrvec3 rotor_up(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));
    irrvec3 body_reaction_moment_in_world = rotor_up.crossProduct(body_up) * m_params.rigidness;
    print_vec("Body reaction", body_reaction_moment_in_world);
    print_vec("Swash torque", swash_torque_in_rotor_coords);

    // Update rotations.
    irrvec3 total_body_torques_in_world = total_tail_moment_in_world - body_reaction_moment_in_world;
    irrvec3 total_rotor_torques_in_world = swash_torque_in_world + body_reaction_moment_in_world;
    update_body_moments(time_delta, total_body_torques_in_world);
    update_rotor_moments(time_delta, total_rotor_torques_in_world);
}

void BaseHeli::update(double time_delta,
                      const irrvec3 &wind_speed,
                      const ServoData &servo_data)
{
    // Update main rotor velocity.
    float target_rotor_vel = m_params.main_rotor_max_vel * servo_data.throttle;
    float main_rotor_vel = m_main_rotor_vel / 360;
    if (target_rotor_vel - main_rotor_vel < m_params.main_rotor_acc) {
        main_rotor_vel += time_delta * (target_rotor_vel - main_rotor_vel);
    } else {
        main_rotor_vel += time_delta * m_params.main_rotor_acc;
    }
    m_main_rotor_vel = main_rotor_vel * 360;
    std::cout << "Main_rotor: " << m_main_rotor_vel << std::endl;
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    // Update angular velocity according to servos.
    update_moments(time_delta, wind_speed, servo_data);

    // Gravity is easy - always down.
    irrvec3 gravity(0, -GRAVITY_CONSTANT, 0);

    // Lift is up in the heli axis;
    irrvec3 heli_up(0, 1, 0);
    m_rotor_rotation.rotateVect(heli_up);
    irrvec3 lift = heli_up * servo_data.lift * m_params.max_lift * main_rotor_effectiveness;
    std::cout << "Lift: (" << lift.X << ", " << lift.Y << ", " << lift.Z << ")" << std::endl;

    // Aerodynamic force.
    irrvec3 drag_vec = m_params.drag;
    drag_vec.Y /= main_rotor_effectiveness > 0.1? 1 : 10;
    irrvec3 airspeed = m_v - wind_speed;
    irr::core::matrix4 world_to_heli = m_rotor_rotation.getTransposed();
    world_to_heli.rotateVect(airspeed);  // In heli coord system.
    irrvec3 aerodynamic_drag = drag_vec * airspeed;
    m_rotor_rotation.rotateVect(aerodynamic_drag);  // Back in world coord system.

    // Account for torbulation in low airspeed.
    float torbulant_coeff = m_params.torbulant_airspeed - norm(airspeed);
    torbulant_coeff = torbulant_coeff > 1 ? 1 : torbulant_coeff;
    torbulant_coeff = torbulant_coeff < 0 ? 0 : torbulant_coeff;
    torbulant_coeff *= 0.5 + 0.5*torbulant_rand.update(time_delta);
    float lift_torbulant_effect = 1 - torbulant_coeff * 0.5;
    std::cout << "Lift torbulant effect: " << lift_torbulant_effect << std::endl;
    lift *= lift_torbulant_effect;

    irrvec3 total_force = gravity + lift - aerodynamic_drag;
    irrvec3 acc = total_force / m_params.mass;
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
        return 1.;
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
    .init_pos = irrvec3(0, 0.25, 0),
    .init_rotation = irrvec3(0, 0, 0),
    .mass = 2.,
    .max_lift = 2. * 10 * 2,  // = mass * 2
    .drag = irrvec3(0.25, 2, 0.05),
    .torbulant_airspeed = 5,
    .main_rotor_max_vel = 40,
    .main_rotor_acc = 10,

    .tail_length = 0.6,
    .tail_drag = 1.,

    .swash_torque = 2. * 10,  // = lift/2
    .yaw_torque = 20,
    .rotor_moment_of_inertia = 1./12 * 0.3 * 1, //  = Rod: 1/12 * M * L^2
    .body_moment_of_inertia = irrvec3(
        // pitch: 3 masses - one in the rotor, one is the body and one is the tail
        (0.2 * 0.5*0.5) + (0.5 * 0.05*0.05 + 0.2 * 1),
        // yaw: 2 masses - one in the tail and one close.
        (0.1 * 0.6*0.6) + (0.5 * 0.1*0.1),
        // roll: 2 masses - one in the rotor and one is the body
        (0.2 * 0.5*0.5) + (0.5 * 0.05*0.05)
    ),

    .rigidness = 15,
    .anti_wobliness = 1.
};


BellHeli::BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver):
    BaseHeli(BELL_AERODYNAMICS)
{
    // Create the body mesh.
	irr::scene::IMesh* heli_mesh = smgr->getMesh("media/Bell/source/bell_body.obj");
	m_body_node = smgr->addMeshSceneNode(heli_mesh);
    m_body_node->setScale(irrvec3(0.25, 0.25, 0.25));
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
    m_rotor_node->setScale(irrvec3(0.5/2, 0.5/2, 0.5/2));
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
    m_tail_rotor_node->setScale(irrvec3(0.5, 0.5, 0.5)/2);
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
    m_main_rotor_angle = 0;
    m_tail_rotor_angle = 0;

    update_ui(0);
}

void BellHeli::update_ui(float time_delta) {
    m_body_node->setPosition(m_pos);
    m_tail_rotor_node->setPosition(m_pos);

    m_body_node->setRotation((m_rotor_rotation * m_shape_rotation).getRotationDegrees());
    m_tail_rotor_node->setRotation((m_rotor_rotation * m_shape_rotation).getRotationDegrees());

    // Set the main rotor rotation.
    irr::core::matrix4 main_rotor_rotation;
    main_rotor_rotation.setRotationDegrees(irrvec3(0, m_main_rotor_angle, 0));
    m_main_rotor_angle += m_main_rotor_vel * time_delta / 4;
    irrvec3 main_rotor_offset(0, 0, 0.225/2);
    (m_rotor_rotation).rotateVect(main_rotor_offset);
    m_rotor_node->setPosition(m_pos - main_rotor_offset);
    m_rotor_node->setRotation((m_rotor_rotation * m_shape_rotation * main_rotor_rotation).getRotationDegrees());

    // Set the tail rotor rotation.
    irr::core::matrix4 tail_rotor_rotation;
    tail_rotor_rotation.setRotationDegrees(irrvec3(0, 0, m_tail_rotor_angle));
    m_tail_rotor_angle += 3 * m_main_rotor_vel * time_delta;
    irrvec3 tail_rotor_offset(0, -0.77/2, 2.31/2);
    (m_rotor_rotation).rotateVect(tail_rotor_offset);
    m_tail_rotor_node->setPosition(m_pos - tail_rotor_offset);
    m_tail_rotor_node->setRotation((m_rotor_rotation * m_shape_rotation * tail_rotor_rotation).getRotationDegrees());
}

