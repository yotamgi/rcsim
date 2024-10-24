#include "heli.h"
#include "smooth_rand.h"
#include <iostream>
#include <cmath>
#include <iomanip>

using irr::core::PI;

const double GRAVITY_CONSTANT = 9.8;
const double MAX_TORBULANT_EFFECT = 0.25; 

float norm(irrvec3 vec) {
    return std::sqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
}


BaseHeli::BaseHeli(const HeliParams &params):
         m_torbulant_rand_front(3., 1),
         m_torbulant_rand_back(3., 1),
         m_torbulant_rand_left(3., 1),
         m_torbulant_rand_right(3., 1),
         m_params(params),
         m_pitch_servo(4, 0),
         m_roll_servo(4, 0),
         m_yaw_servo(8, 0),
         m_lift_servo(4, 0),
         m_throttle_servo(0.5, -0.9)
{
    m_pos = m_params.init_pos;
    m_v = irrvec3(0, 0, 0);
    m_rotor_rotation.setRotationDegrees(m_params.init_rotation);
    m_main_rotor_vel = 0;
}

float BaseHeli::ServoFilter::update(float value, float time_delta) {
    float step = time_delta * m_max_rps;
    if (step > std::abs(value - m_current_status)) {
        m_current_status = value;
    }

    float delta = value - m_current_status;
    if (delta < 0) {
        m_current_status -= step;
    } else if (delta > 0) {
        m_current_status += step;
    }
    m_current_status =  std::max(-1.0f, std::min(m_current_status, 1.0f));

    return m_current_status;
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

void print_vec(const std::string str, const irrvec3 &vec) {
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
    )  / m_params.body_moment_of_inertia;

    // Get the angularv in world coordinates.
    irrvec3 body_angularv;
    m_body_rotation.rotateVect(body_angularv,  m_body_angularv_in_body_coords);
    update_rotation_matrix(m_body_rotation, body_angularv * time_delta);
}

void BaseHeli::update_rotor_moments(float time_delta, const irrvec3 &moment_in_world)
{
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;
    // Update rotor angular momentum by the torques.
    m_rotor_angular_momentum_in_world += time_delta * moment_in_world;

    // Update back the rotor rotation matrix:
    // The Y vector is taken to point towards the angular momentun.
    irrvec3 rot_y = m_rotor_angular_momentum_in_world;
    rot_y.normalize();

    // Only at the first frame, the angular_momentum is 0 and it screws up everything.
    // This is a dirty workaround this.
    if (rot_y.X == 0 && rot_y.Y == 0 && rot_y.Z == 0) {
        rot_y = irrvec3(0, 1, 0);
    }

    // The Z (pointing towards tail) is taken from the body.
    irrvec3 rot_z(m_body_rotation(2, 0), m_body_rotation(2, 1), m_body_rotation(2, 2));

    // The rest are updated.
    irrvec3 rot_x = rot_y.crossProduct(rot_z);
    rot_z = rot_x.crossProduct(rot_y);

    // Tackle numerical instability in low RPMs - since in low RPMs the torques 
    // on the rotor can change its orientation quite a bit, this causes a lot
    // of numerical instability.
    if (main_rotor_effectiveness < 0.2) {
        m_rotor_rotation = m_body_rotation;
        rot_y.X = m_rotor_rotation(1, 0); rot_y.Y = m_rotor_rotation(1, 1); rot_y.Z = m_rotor_rotation(1, 2);
        m_rotor_angular_momentum_in_world = norm(m_rotor_angular_momentum_in_world) * rot_y;
    } else {
        m_rotor_rotation(0, 0) = rot_x.X; m_rotor_rotation(0, 1) = rot_x.Y; m_rotor_rotation(0, 2) = rot_x.Z;
        m_rotor_rotation(1, 0) = rot_y.X; m_rotor_rotation(1, 1) = rot_y.Y; m_rotor_rotation(1, 2) = rot_y.Z;
        m_rotor_rotation(2, 0) = rot_z.X; m_rotor_rotation(2, 1) = rot_z.Y; m_rotor_rotation(2, 2) = rot_z.Z;
    }
}


void BaseHeli::update_moments(float time_delta,
                              const irrvec3 &wind_speed)
{
    // Update angular velocity according to torques.
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    // Calculate the engine moment
    float throttle_ratio = (m_throttle_servo.get() + 1.05) / 2.05;  // Change from (-1, 1) to (0.05, 1).
    m_main_rotor_target_rps = m_params.main_rotor_max_vel * throttle_ratio;
    float target_rotor_omega = m_main_rotor_target_rps * 2 * PI;
    float main_rotor_omega = norm(m_rotor_angular_momentum_in_world) / m_params.rotor_moment_of_inertia;
    float main_rotor_torque;
    float omega_ratio = main_rotor_omega / target_rotor_omega;
    if (omega_ratio < 0.9) {
        main_rotor_torque = m_params.main_rotor_torque;
    } else {
        main_rotor_torque = (1 - (omega_ratio - 0.9) * 10) * m_params.main_rotor_torque;
    }
    m_main_rotor_vel = main_rotor_omega / (2 * PI) * 360;
    irrvec3 rotor_y(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));
    irrvec3 engine_torque_in_world = main_rotor_torque * rotor_y;

    // Calculate the rotor drag force, trying to slow down the rotor.
    float angle_of_attack = m_params.main_rotor_max_angle_of_attack * m_lift_servo.get() / 360 * 2 * PI;
    irrvec3 rotor_drag_torque_in_world =
            -m_lift_force
            * m_params.main_rotor_length / 3  // Geometric coeffcient from rotor lift to drag torque.
            * std::tan(angle_of_attack)  // Ratio between wing's lift and drag.
            * rotor_y;  // Directed upwards.

    // Calculate the servos torque.
    // Note that the roll and pitch are switched due to the gyro 90 deg effect.
    irrvec3 swash_torque_in_rotor_coords(
        -m_roll_servo.get() * m_params.swash_torque,
        0,
        m_pitch_servo.get() * m_params.swash_torque
    );
    irrvec3 yaw_torque_in_rotor_coords(
        0, m_yaw_servo.get() * m_params.yaw_torque, 0
    );
    swash_torque_in_rotor_coords *= main_rotor_effectiveness;
    yaw_torque_in_rotor_coords *= main_rotor_effectiveness;
    m_tail_rotor_force = yaw_torque_in_rotor_coords.Y / m_params.tail_length;
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

    irrvec3 body_reaction_moment;
    m_rotor_rotation.getTransposed().rotateVect(body_reaction_moment, body_reaction_moment_in_world);

    // Anti-wobliness moment.
    irrvec3 dbody_reaction_moment = (body_reaction_moment - m_prev_reaction_in_body) / time_delta;
    m_prev_reaction_in_body = body_reaction_moment;
    irrvec3 anti_wobliness_in_world;
    m_rotor_rotation.rotateVect(anti_wobliness_in_world, dbody_reaction_moment);
    body_reaction_moment_in_world += anti_wobliness_in_world * m_params.anti_wobliness;

    // Limit the external torque.
    irrvec3 external_torque = m_external_torque;
    if (norm(external_torque) > m_params.external_torque_limit) {
        external_torque /= norm(external_torque)/m_params.external_torque_limit;
    }

    // Update rotations.
    irrvec3 total_body_torques_in_world = total_tail_moment_in_world 
                                        - engine_torque_in_world
                                        - body_reaction_moment_in_world
                                        + external_torque;
    irrvec3 total_rotor_torques_in_world = swash_torque_in_world
                                            + engine_torque_in_world
                                            + body_reaction_moment_in_world
                                            + rotor_drag_torque_in_world;
    update_body_moments(time_delta, total_body_torques_in_world);
    update_rotor_moments(time_delta, total_rotor_torques_in_world);
}

void BaseHeli::update(double time_delta,
                      const irrvec3 &wind_speed,
                      const ServoData &servo_data)
{
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    m_pitch_servo.update(servo_data.pitch, time_delta);
    m_roll_servo.update(servo_data.roll, time_delta);
    m_yaw_servo.update(servo_data.yaw, time_delta);
    m_lift_servo.update(servo_data.lift, time_delta);
    m_throttle_servo.update(servo_data.throttle, time_delta);

    // Update angular velocity according to servos.
    update_moments(time_delta, wind_speed);

    // Gravity is easy - always down.
    irrvec3 gravity(0, -GRAVITY_CONSTANT, 0);

    // Lift is up in the heli axis;
    irrvec3 heli_up(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));
    irrvec3 lift = heli_up * servo_data.lift * m_params.max_lift * main_rotor_effectiveness;

    // Tail rotor thrust also adds forces.
    irrvec3 heli_right(m_rotor_rotation(0, 0), m_rotor_rotation(0, 1), m_rotor_rotation(0, 2));
    irrvec3 tail_thrust = heli_right * m_tail_rotor_force;

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
    irrvec3 torbulant_base = - MAX_TORBULANT_EFFECT * lift * torbulant_coeff / 4;
    irrvec3 front_torbulation_force = torbulant_base * m_torbulant_rand_front.update(time_delta);
    irrvec3 back_torbulation_force = torbulant_base * m_torbulant_rand_back.update(time_delta);
    irrvec3 left_torbulation_force = torbulant_base * m_torbulant_rand_left.update(time_delta);
    irrvec3 right_torbulation_force = torbulant_base * m_torbulant_rand_right.update(time_delta);
    lift += front_torbulation_force + back_torbulation_force + left_torbulation_force + right_torbulation_force;

    // Account for ground effects.
    float ground_effect_intensity = (m_pos.Y > 0.5) ? 0 : ((0.5 - m_pos.Y) / 0.5);
    lift *= (1 + ground_effect_intensity*2);

    irrvec3 total_force = gravity + lift - aerodynamic_drag + tail_thrust + m_external_force;
    irrvec3 acc = total_force / m_params.mass;
    m_v += time_delta * acc;
    m_pos += time_delta * m_v;

    update_ui(time_delta);
    m_lift_force = norm(lift);
}

static irrvec3 rotate(irr::core::matrix4 matrix, irrvec3 vec) {
    matrix.rotateVect(vec);
    return vec;
}


void BaseHeli::add_force(unsigned int touchpoint_index, const irrvec3 &force) {
    m_external_force += force;
    m_external_torque += rotate(m_body_rotation,
            m_params.touchpoints_in_heli[touchpoint_index]).crossProduct(force);
}


std::vector<BaseHeli::TouchPoint> BaseHeli::get_touchpoints_in_world() {
    std::vector<BaseHeli::TouchPoint> touchpoints_in_world;
    for (auto touchpoint_in_body : m_params.touchpoints_in_heli) {
        BaseHeli::TouchPoint tp;
        tp.pos_in_world = m_pos + rotate(m_body_rotation, touchpoint_in_body);
        tp.vel_in_world = m_v + rotate(m_body_rotation,
                    m_body_angularv_in_body_coords.crossProduct(touchpoint_in_body));
        touchpoints_in_world.push_back(tp);
    }
    return touchpoints_in_world;
}


BaseHeli::Telemetry BaseHeli::get_telemetry() const {
    Telemetry telemetry;
    telemetry.main_rotor_rps = m_main_rotor_vel / 360;
    telemetry.main_rotor_target_rps = m_main_rotor_target_rps;
    return telemetry;
}


class MainRotorBlur : public RotorBlur {
public:
    MainRotorBlur(irr::scene::ISceneManager *smgr, irr::scene::IMeshSceneNode *parent) {
        init_ui(
            smgr,
            3.75,  // radius.
            irrvec3(-0.0, 1.8/4, 0),  // position.
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
            irrvec3(0, 0.57, 0),  // position.
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
    .init_pos = irrvec3(0, 0.13 + -0.019, 0),
    .init_rotation = irrvec3(0, 0, 0),
    .mass = 2.,
    .max_lift = 2. * 10 * 2.5,  // = mass * 2.5
    .drag = irrvec3(0.25, 2, 0.05),
    .torbulant_airspeed = 5,
    .main_rotor_max_vel = 55,
    .main_rotor_torque = 1.6,
    .main_rotor_length = 1.,
    .main_rotor_max_angle_of_attack = 12.,

    .tail_length = 0.6,
    .tail_drag = 1.,

    .swash_torque = 4. * 10,  // = lift
    .yaw_torque = 50,
    .rotor_moment_of_inertia = 1./12 * 0.3 * 1, //  = Rod: 1/12 * M * L^2
    .body_moment_of_inertia = irrvec3(
        // pitch: 2 masses - one in the rotor and one in the body.
        (0.2 * 0.5*0.5) + 1.5 * 0.15*0.15,
        // yaw: 2 masses - one in the tail and one close.
        (0.1 * 0.6*0.6) + (0.5 * 0.1*0.1),
        // roll: 1 masses - the body.
        (1.5 * 0.1*0.1)
    ),

    .rigidness = 10,
    .anti_wobliness = 1./10,

    .touchpoints_in_heli = std::vector<irrvec3>({
     irrvec3( 0.19, -0.128,  0.17),
     irrvec3(-0.19, -0.128,  0.17),
     irrvec3(-0.19, -0.128, -0.28),
     irrvec3( 0.19, -0.128, -0.28),
     irrvec3(0, 0.05, -1.12)
    }),

    .external_torque_limit = 2.
};


BellHeli::BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver):
    BaseHeli(BELL_AERODYNAMICS)
{
    // Create the body mesh.
	irr::scene::IMesh* heli_mesh = smgr->getMesh("media/Bell/source/bell_body.obj");
	m_body_node = smgr->addMeshSceneNode(heli_mesh);
    m_body_node->setScale(irrvec3(1./4, 1./4, 1./4));
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
    main_rotor_translation.setTranslation(irrvec3(0.45, -1.4, 0));
    smgr->getMeshManipulator()->apply(
        irr::scene::SVertexPositionTransformManipulator(main_rotor_translation), main_rotor_mesh);
	m_rotor_node = smgr->addMeshSceneNode(main_rotor_mesh, m_body_node);
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
	m_tail_rotor_node = smgr->addMeshSceneNode(tail_rotor_mesh, m_body_node);
    m_tail_rotor_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    m_tail_rotor_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_tail_rotor_node->setDebugDataVisible(irr::scene::EDS_OFF);
    m_tail_rotor_node->setMaterialTexture(0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
    m_tail_rotor_node->addShadowVolumeSceneNode();
    for (unsigned int i=0; i < m_tail_rotor_node->getMaterialCount(); i++) {
        m_tail_rotor_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
    }

    m_shape_rotation.setRotationDegrees(irrvec3(0, -90, 0));

    m_main_rotor_blur = std::make_shared<MainRotorBlur>(smgr, m_rotor_node);
    m_flybar_blur = std::make_shared<FlybarBlur>(smgr, m_rotor_node);
    m_tail_prop_blur = std::make_shared<TailRotorBlur>(smgr, m_body_node);
    m_main_rotor_angle = 0;
    m_tail_rotor_angle = 0;

    update_ui(0);
}

void BellHeli::update_ui(float time_delta) {
    m_body_node->setPosition(m_pos);
    m_tail_rotor_node->setPosition(m_pos);

    m_body_node->setRotation((m_body_rotation * m_shape_rotation).getRotationDegrees());
    m_tail_rotor_node->setRotation((m_rotor_rotation * m_shape_rotation).getRotationDegrees());

    // Set the main rotor rotation.
    irr::core::matrix4 main_rotor_rotation;
    main_rotor_rotation.setRotationDegrees(irrvec3(0, m_main_rotor_angle, 0));
    m_main_rotor_angle += m_main_rotor_vel * time_delta / 4;
    irrvec3 main_rotor_offset(0.45, -1.4, 0);
    m_rotor_node->setPosition(-main_rotor_offset);
    irr::core::matrix4 rotor_rotation_in_body_coords = m_body_rotation.getTransposed() * m_rotor_rotation;
    m_rotor_node->setRotation((rotor_rotation_in_body_coords * main_rotor_rotation).getRotationDegrees());

    // Set the tail rotor rotation.
    irr::core::matrix4 tail_rotor_rotation;
    tail_rotor_rotation.setRotationDegrees(irrvec3(0, 0, m_tail_rotor_angle));
    m_tail_rotor_angle += 3 * m_main_rotor_vel * time_delta;
    irrvec3 tail_rotor_offset(2.31*2, -0.77*2, 0);
    m_tail_rotor_node->setPosition(-tail_rotor_offset);
    m_tail_rotor_node->setRotation((tail_rotor_rotation).getRotationDegrees());
}

