#include "heli.h"
#include "smooth_rand.h"
#include <iostream>
#include <cmath>
#include <iomanip>

using irr::core::PI;

const double GRAVITY_CONSTANT = 9.8;
const double MAX_TORBULANT_EFFECT = 0.6;
const float AIR_DENSITY = 1.2;

float norm(irrvec3 vec) {
    return std::sqrt(vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z);
}


BaseHeli::BaseHeli(const HeliParams &params):
         m_torbulant_rand_front(3., 0.6),
         m_torbulant_rand_back(3., 0.6),
         m_torbulant_rand_left(3., 0.6),
         m_torbulant_rand_right(3., 0.6),
         m_pitch_servo(4, 0),
         m_roll_servo(4, 0),
         m_yaw_servo(8, 0),
         m_lift_servo(4, 0),
         m_throttle_servo(0.5, -0.9),
         m_params(params)
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
    irrvec3 rot_x = rot_y.crossProduct(rot_z).normalize();
    rot_z = rot_x.crossProduct(rot_y).normalize();

    // Tackle numerical instability in low RPMs - since in low RPMs the torques 
    // on the rotor can change its orientation quite a bit, this causes a lot
    // of numerical instability.
    if (main_rotor_effectiveness < 0.3) {
        m_rotor_rotation = m_body_rotation;
        rot_y.X = m_rotor_rotation(1, 0); rot_y.Y = m_rotor_rotation(1, 1); rot_y.Z = m_rotor_rotation(1, 2);
        rot_y = rot_y.normalize();
        m_rotor_angular_momentum_in_world = norm(m_rotor_angular_momentum_in_world) * rot_y;
    } else {
        m_rotor_rotation(0, 0) = rot_x.X; m_rotor_rotation(0, 1) = rot_x.Y; m_rotor_rotation(0, 2) = rot_x.Z;
        m_rotor_rotation(1, 0) = rot_y.X; m_rotor_rotation(1, 1) = rot_y.Y; m_rotor_rotation(1, 2) = rot_y.Z;
        m_rotor_rotation(2, 0) = rot_z.X; m_rotor_rotation(2, 1) = rot_z.Y; m_rotor_rotation(2, 2) = rot_z.Z;
    }
}

irrvec3 BaseHeli::calc_body_rotor_reaction_moment(float time_delta) {

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

    return body_reaction_moment_in_world;
}

void BaseHeli::calc_tail_rotor_force(const irrvec3 &wind_speed, 
            irrvec3 &out_force_in_world,
            irrvec3 &out_torque_in_world) 
{
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    irrvec3 tail_torque_in_rotor_coords(
        0, m_yaw_servo.get() * m_params.tail_rotor_max_force * m_params.tail_length, 0
    );
    tail_torque_in_rotor_coords *= main_rotor_effectiveness;
    float tail_rotor_force_magnitude = tail_torque_in_rotor_coords.Y / m_params.tail_length;
    irrvec3 heli_right(m_rotor_rotation(0, 0), m_rotor_rotation(0, 1), m_rotor_rotation(0, 2));
    irrvec3 tail_rotor_force_in_world = tail_rotor_force_magnitude * heli_right;

    irrvec3 tail_torque_in_world;
    m_body_rotation.rotateVect(tail_torque_in_world, tail_torque_in_rotor_coords);

    out_force_in_world = tail_rotor_force_in_world;
    out_torque_in_world = tail_torque_in_world;
}

irrvec3 BaseHeli::calc_engine_torque() {
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    float throttle_ratio = (m_throttle_servo.get() + 1.05) / 2.05;  // Change from (-1, 1) to (0.05, 1).
    m_main_rotor_target_rps = m_params.main_rotor_max_vel * throttle_ratio;
    float target_rotor_omega = m_main_rotor_target_rps * 2 * PI;
    float main_rotor_omega = norm(m_rotor_angular_momentum_in_world) / m_params.rotor_moment_of_inertia;
    float main_rotor_torque;
    float omega_ratio = main_rotor_omega / target_rotor_omega;
    if (omega_ratio < 0.9) {
        main_rotor_torque = m_params.main_rotor_torque * (main_rotor_effectiveness + 0.1);
    } else {
        main_rotor_torque = (1 - (omega_ratio - 0.9) * 10) * m_params.main_rotor_torque;
    }
    float motor_drag_torque = -0.03 * m_params.main_rotor_torque;
    main_rotor_torque = main_rotor_torque < motor_drag_torque ? motor_drag_torque : main_rotor_torque;
    m_main_rotor_vel = main_rotor_omega / (2 * PI) * 360;
    irrvec3 rotor_y(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));
    return main_rotor_torque * rotor_y;
}

irrvec3 BaseHeli::calc_swash_torque() {
    float main_rotor_effectiveness = m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

    // Note that the roll and pitch are switched due to the gyro 90 deg effect.
    irrvec3 swash_torque_in_rotor_coords(
        -m_roll_servo.get() * m_params.swash_torque,
        0,
        m_pitch_servo.get() * m_params.swash_torque
    );
    swash_torque_in_rotor_coords *= main_rotor_effectiveness;
    irrvec3 swash_torque_in_world;
    m_rotor_rotation.rotateVect(swash_torque_in_world, swash_torque_in_rotor_coords);
    return swash_torque_in_world;
}

irrvec3 BaseHeli::torbulant_force(
        const irrvec3 &pos_in_world,
        const irrvec3 &airspeed_in_world,
        const irrvec3 &lift_in_world)
{
    // Each torbulant point has a target velocity in which the torbulant effect
    // start effcting. On points that get fresh air should have half of the
    // of the velocity.
    float pos_aligned = (pos_in_world / norm(pos_in_world)).dotProduct(
            airspeed_in_world / (norm(airspeed_in_world) + 0.1));
    pos_aligned = pos_aligned >  1 ? 1 : pos_aligned;
    pos_aligned = pos_aligned < -1 ? -1 : pos_aligned;

    // The target velocity changes from 30% on the back to 90% on the top
    float target_vel = m_params.torbulant_airspeed*0.6;
    target_vel += pos_aligned * m_params.torbulant_airspeed*0.3;

    float torbulant_coeff = 1 - norm(airspeed_in_world) / target_vel;
    torbulant_coeff = torbulant_coeff < 0 ? 0 : torbulant_coeff;

    return -MAX_TORBULANT_EFFECT * lift_in_world * torbulant_coeff / 4;
}

void BaseHeli::update_torbulation(float time_delta,
                                  const irrvec3 &lift_in_world,
                                  const irrvec3 &wind_speed,
                                  irrvec3 &out_torbulant_force_in_world,
                                  irrvec3 &out_torbulant_torque_in_world)
{
    irrvec3 airspeed_in_world = - (m_v - wind_speed);

    irrvec3 front = irrvec3(m_rotor_rotation(2, 0), m_rotor_rotation(2, 1), m_rotor_rotation(2, 2))
                    * m_params.main_rotor_length/2;
    irrvec3 right = irrvec3(m_rotor_rotation(0, 0), m_rotor_rotation(0, 1), m_rotor_rotation(0, 2))
                    * m_params.main_rotor_length/2;

    irrvec3 front_torbulation_force = torbulant_force(front, airspeed_in_world, lift_in_world)
                                     * (0.75 + 0.25*m_torbulant_rand_front.update(time_delta));
    irrvec3 back_torbulation_force = torbulant_force(-front, airspeed_in_world, lift_in_world)
                                     * (0.75 + 0.25*m_torbulant_rand_back.update(time_delta));
    irrvec3 right_torbulation_force = torbulant_force(right, airspeed_in_world, lift_in_world)
                                     * (0.75 + 0.25*m_torbulant_rand_right.update(time_delta));
    irrvec3 left_torbulation_force = torbulant_force(-right, airspeed_in_world, lift_in_world)
                                     * (0.75 + 0.25*m_torbulant_rand_left.update(time_delta));
    out_torbulant_force_in_world = front_torbulation_force
                        + back_torbulation_force
                        + left_torbulation_force
                        + right_torbulation_force;
    out_torbulant_torque_in_world = front_torbulation_force.crossProduct(front)
                        + back_torbulation_force.crossProduct(-front)
                        + right_torbulation_force.crossProduct(right)
                        + left_torbulation_force.crossProduct(-right);
}

void BaseHeli::calc_aerodynamic_drag(
        const irrvec3 &wind_speed,
        irrvec3 &out_force_in_world,
        irrvec3 &out_torque_in_world)
{
    irrvec3 airspeed_in_world = - (m_v - wind_speed);

    // Calculate the drag force.
    irrvec3 drag_vec = m_params.drag;
    irr::core::matrix4 world_to_heli = m_rotor_rotation.getTransposed();
    irrvec3 airspeed_in_heli;
    world_to_heli.rotateVect(airspeed_in_heli, airspeed_in_world);  // In heli coord system.
    irrvec3 aerodynamic_drag_force = drag_vec * airspeed_in_heli;
    m_rotor_rotation.rotateVect(aerodynamic_drag_force);  // Back in world coord system.

    // calculate the tail-drag torque.
    irrvec3 heli_right_in_world(m_body_rotation(0, 0), m_body_rotation(0, 1), m_body_rotation(0, 2));
    float tail_wind = heli_right_in_world.dotProduct(airspeed_in_world);
    tail_wind += m_body_angularv_in_body_coords.Y * m_params.tail_length;
    float tail_drag_moment_y = -tail_wind * m_params.tail_drag * m_params.tail_length;
    irrvec3 aerodynamic_drag_torque(0, tail_drag_moment_y, 0);
    irrvec3 total_tail_torque_in_world;
    m_body_rotation.rotateVect(total_tail_torque_in_world, aerodynamic_drag_torque);

    // Return;
    out_force_in_world = - aerodynamic_drag_force;
    out_torque_in_world = - aerodynamic_drag_torque;
}



irrvec3 BaseHeli::calc_dissimetry_of_lift(const irrvec3 &wind_speed, float lift_magnitude) {
    irrvec3 airspeed_in_world = - (m_v - wind_speed);
    float rotor_radius = m_params.main_rotor_length / 2.;
    float rotor_omega = m_main_rotor_vel / 360 * 2 * PI;
    float max_airspeed = rotor_omega * rotor_radius * 0.8 + 0.0001;

    irrvec3 rotor_up(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));
    irrvec3 transient_flow = airspeed_in_world - rotor_up * airspeed_in_world.dotProduct(rotor_up);
    float effect_magnitude = norm(transient_flow) / max_airspeed;

    irrvec3 dol_direction = transient_flow / (norm(transient_flow) + 0.0001);

    return dol_direction * effect_magnitude * lift_magnitude * rotor_radius * 0.9;
}


// The lift forces and engine torques.
void BaseHeli::calc_lift_force(float time_delta,
                     const irrvec3 &wind_speed,
                     irrvec3 &out_force_in_world,
                     irrvec3 &out_rotor_torque_in_world)
{
    irrvec3 rotor_up(m_rotor_rotation(1, 0), m_rotor_rotation(1, 1), m_rotor_rotation(1, 2));

    // Calc the basic lift force.
    irrvec3 airspeed_in_world = - (m_v - wind_speed);
    float rotor_inflow_velocity = -rotor_up.dotProduct(airspeed_in_world);
    float rotor_radius = m_params.main_rotor_length / 2.;
    float rotor_angle_of_attack = m_params.main_rotor_max_angle_of_attack * m_lift_servo.get() / 360 * 2 * PI;
    float rotor_omega = m_main_rotor_vel / 360 * 2 * PI;
    float rotor_thrust_velocity = rotor_omega * std::tan(rotor_angle_of_attack) * rotor_radius * 0.8;
    float rotor_surface_area = rotor_radius * rotor_radius * PI;
    float rotor_mass_flow = rotor_thrust_velocity * rotor_surface_area * AIR_DENSITY;  // [Mass / Sec]
    float rotor_lift_force_magnitude = std::abs(rotor_mass_flow) * rotor_thrust_velocity;
    float drag_lift_force_magnitude = - std::abs(rotor_mass_flow) * rotor_inflow_velocity;
    irrvec3 lift = rotor_up * (rotor_lift_force_magnitude + drag_lift_force_magnitude * 0.7);

    // Account for torbulation force and torque.
    irrvec3 torbulant_force_in_world, torbulant_torque_in_world;
    update_torbulation(time_delta, lift, wind_speed,
                       torbulant_force_in_world, torbulant_torque_in_world);

    // Calculate the rotor drag force, trying to slow down the rotor.
    irrvec3 rotor_drag_torque =
            - norm(lift + torbulant_force_in_world)
            * m_params.main_rotor_length / 3  // Geometric coeffcient from rotor lift to drag torque.
            * std::tan(std::abs(rotor_angle_of_attack))  // Ratio between wing's lift and drag.
            * rotor_up;  // Directed upwards.

    // Account for Dissimetry of Lift.
    irrvec3 dissimetry_of_lift_torque = calc_dissimetry_of_lift(wind_speed, rotor_lift_force_magnitude);

    // Ground effect force.
    float ground_effect_intensity = (m_pos.Y > 0.3) ? 0 : ((0.3 - m_pos.Y) / 0.3);
    lift *= (1 + ground_effect_intensity);

    // Return;
    out_force_in_world = lift + torbulant_force_in_world;
    out_rotor_torque_in_world = rotor_drag_torque + torbulant_torque_in_world + dissimetry_of_lift_torque;
}


void BaseHeli::update(double time_delta,
                      const irrvec3 &wind_speed,
                      const ServoData &servo_data)
{
    m_pitch_servo.update(servo_data.pitch, time_delta);
    m_roll_servo.update(servo_data.roll, time_delta);
    m_yaw_servo.update(servo_data.yaw, time_delta);
    m_lift_servo.update(servo_data.lift, time_delta);
    m_throttle_servo.update(servo_data.throttle, time_delta);

    // Engine torque.
    irrvec3 engine_torque_in_world = calc_engine_torque();

    // Swash torques.
    irrvec3 swash_torque_in_world = calc_swash_torque();

    // Tail torque and thrust force.
    irrvec3 total_tail_torque_in_world, tail_rotor_force_in_world;
    calc_tail_rotor_force(wind_speed, tail_rotor_force_in_world, total_tail_torque_in_world);

    // The body-rotor reaction forces.
    irrvec3 body_reaction_moment_in_world = calc_body_rotor_reaction_moment(time_delta);

    // Gravity force.
    irrvec3 gravity(0, -GRAVITY_CONSTANT * m_params.mass, 0);

    // Aerodynamic drag force and torque.
    irrvec3 aerodynamic_drag_force, aerodynamic_drag_torque;
    calc_aerodynamic_drag(wind_speed, aerodynamic_drag_force, aerodynamic_drag_torque);

    // Lift force.
    irrvec3 lift_force, rotor_lift_torque;
    calc_lift_force(time_delta, wind_speed, lift_force, rotor_lift_torque);

    // Limit the external torque.
    irrvec3 external_torque = m_external_torque;
    if (norm(external_torque) > m_params.external_torque_limit) {
        external_torque /= norm(external_torque)/m_params.external_torque_limit;
    }

    // Update pos/speed.
    irrvec3 total_force = gravity + lift_force - aerodynamic_drag_force - tail_rotor_force_in_world
            + m_external_force;
    irrvec3 acc = total_force / m_params.mass;
    m_v += time_delta * acc;
    m_pos += time_delta * m_v;

    // Update rotations/angular velocities.
    update_body_moments(time_delta, total_tail_torque_in_world 
                                        + aerodynamic_drag_torque
                                        - engine_torque_in_world
                                        - body_reaction_moment_in_world
                                        + external_torque);
    update_rotor_moments(time_delta, swash_torque_in_world
                                            + rotor_lift_torque
                                            + engine_torque_in_world
                                            + body_reaction_moment_in_world);

    // Update UI.
    update_ui(time_delta);
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
    .drag = irrvec3(1, 0.6, 0.2),
    .torbulant_airspeed = 7,
    .main_rotor_max_vel = 35,
    .main_rotor_torque = 8.,
    .main_rotor_length = 1.,
    .main_rotor_max_angle_of_attack = 10.,

    .tail_length = 0.6,
    .tail_drag = 0.1,
    .tail_rotor_max_force = 30,

    .swash_torque = 4 * 10,  // ~= Mass * G * 1M
    .rotor_moment_of_inertia = 1./12 * 0.3 * 1, //  = Rod: 1/12 * M * L^2
    .body_moment_of_inertia = irrvec3(
        // pitch: 2 masses - one in the rotor and one in the body.
        (0.2 * 0.5*0.5) + 1.5 * 0.15*0.15,
        // yaw: 2 masses - one in the tail and one close.
        (0.1 * 0.6*0.6) + (0.5 * 0.1*0.1),
        // roll: 1 masses - the body.
        (1.5 * 0.1*0.1)
    ),

    .rigidness = 20,
    .anti_wobliness = 1./20,

    .touchpoints_in_heli = std::vector<irrvec3>({
     irrvec3( 0.19*(5./6.), -0.128*(5./6.),  0.17*(5./6.)),
     irrvec3(-0.19*(5./6.), -0.128*(5./6.),  0.17*(5./6.)),
     irrvec3(-0.19*(5./6.), -0.128*(5./6.), -0.28*(5./6.)),
     irrvec3( 0.19*(5./6.), -0.128*(5./6.), -0.28*(5./6.)),
     irrvec3(0, 0.05, -1.12*(5./6.))
    }),

    .external_torque_limit = 1
};


BellHeli::BellHeli(irr::scene::ISceneManager* smgr, irr::video::IVideoDriver* driver):
    BaseHeli(BELL_AERODYNAMICS)
{
    // Create the body mesh.
	irr::scene::IMesh* heli_mesh = smgr->getMesh("media/Bell/source/bell_body.obj");
	m_body_node = smgr->addMeshSceneNode(heli_mesh);
    m_body_node->setScale(irrvec3(1./5, 1./5, 1./5));
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
    m_main_rotor_angle += m_main_rotor_vel * time_delta / 2.1;
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

