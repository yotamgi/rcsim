#include "heli.h"
#include "smooth_rand.h"
#include <cmath>
#include <iomanip>
#include <iostream>

const double GRAVITY_CONSTANT = 9.8;
const double MAX_TORBULANT_EFFECT = 0.4;
const float AIR_DENSITY = 1.2;

float sign(float value) { return (value > 0) ? 1 : -1; }

BaseHeli::BaseHeli(const HeliParams &params)
    : m_v(0, 0, 0), m_pos(params.init_pos), m_main_rotor_vel(0),
      m_external_torque(0, 0, 0), m_external_force(0, 0, 0),
      m_rotor_angular_momentum_in_world(0, 0, 0),
      m_rotor_rotation(
          engine::mat4::RotateXYZ(params.init_rotation / 180 * PI)),
      m_body_rotation(engine::mat4::Identity()),
      m_body_angularv_in_body_coords(0, 0, 0), m_torbulant_rand_front(3., 0.6),
      m_torbulant_rand_back(3., 0.6), m_torbulant_rand_left(3., 0.6),
      m_torbulant_rand_right(3., 0.6), m_prev_reaction_in_body(0, 0, 0),
      m_pitch_servo(4, 0), m_roll_servo(4, 0), m_yaw_servo(8, 0),
      m_lift_servo(4, 0), m_throttle_servo(0.5, -0.9), m_params(params) {}

static void print_matrix(const std::string str, engine::mat4 &mat) {
  std::cout << std::fixed << std::setprecision(3) << std::setw(5)
            << std::setfill(' ');
  std::cout << "Matrix " << str << ":" << std::endl;
  std::cout << engine::mat_get(mat, 0, 0) << ", ";
  std::cout << engine::mat_get(mat, 0, 1) << ", ";
  std::cout << engine::mat_get(mat, 0, 2) << std::endl;

  std::cout << engine::mat_get(mat, 1, 0) << ", ";
  std::cout << engine::mat_get(mat, 1, 1) << ", ";
  std::cout << engine::mat_get(mat, 1, 2) << std::endl;

  std::cout << engine::mat_get(mat, 2, 0) << ", ";
  std::cout << engine::mat_get(mat, 2, 1) << ", ";
  std::cout << engine::mat_get(mat, 2, 2) << std::endl;
}

void print_vec(const std::string str, const engine::vec3 &vec) {
  std::cout << "Vec " << str << vec.ToString() << std::endl;
}

void BaseHeli::update_body_moments(float time_delta,
                                   const engine::vec3 &moment_in_world) {
  engine::vec3 moment_in_body = m_body_rotation.Transpose() * moment_in_world;

  // Update body angular velocity according to Eurler's equation.
  m_body_angularv_in_body_coords +=
      time_delta *
      (moment_in_body -
       m_body_angularv_in_body_coords.CrossProduct(
           m_params.body_moment_of_inertia * m_body_angularv_in_body_coords)) /
      m_params.body_moment_of_inertia;

  // Get the angularv in world coordinates.
  engine::vec3 body_angularv = m_body_rotation * m_body_angularv_in_body_coords;
  update_rotation_matrix(m_body_rotation, body_angularv * time_delta);
}

void BaseHeli::update_rotor_moments(float time_delta,
                                    const engine::vec3 &moment_in_world) {
  float main_rotor_effectiveness =
      m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;
  // Update rotor angular momentum by the torques.
  m_rotor_angular_momentum_in_world += time_delta * moment_in_world;

  // Update back the rotor rotation matrix:
  // The Y vector is taken to point towards the angular momentun.
  engine::vec3 rot_y = m_rotor_angular_momentum_in_world;
  rot_y = rot_y.Normalize();
  engine::vec3 body_y(engine::mat_get(m_body_rotation, 1, 0),
                      engine::mat_get(m_body_rotation, 1, 1),
                      engine::mat_get(m_body_rotation, 1, 2));
  // Tackle numerical instability in low RPMs - since in low RPMs the torques
  // on the rotor can change its orientation quite a bit, this causes a lot
  // of numerical instability.
  rot_y = main_rotor_effectiveness * m_rotor_angular_momentum_in_world +
          (1 - main_rotor_effectiveness) * body_y;
  rot_y = rot_y.Normalize();

  // Only at the first frame, the angular_momentum is 0 and it screws up
  // everything. This is a dirty workaround this.
  if (rot_y.x == 0 && rot_y.y == 0 && rot_y.z == 0) {
    rot_y = engine::vec3(0, 1, 0);
  }

  // The Z (pointing towards tail) is taken from the body.
  engine::vec3 rot_z(engine::mat_get(m_body_rotation, 2, 0),
                     engine::mat_get(m_body_rotation, 2, 1),
                     engine::mat_get(m_body_rotation, 2, 2));

  // The rest are updated.
  engine::vec3 rot_x = rot_y.CrossProduct(rot_z).Normalize();
  rot_z = rot_x.CrossProduct(rot_y).Normalize();

  // update the angular velocity according to the angular momentum.
  float main_rotor_omega = m_rotor_angular_momentum_in_world.Length() /
                           m_params.rotor_moment_of_inertia;
  m_main_rotor_vel = main_rotor_omega / (2 * PI) * 360;
  m_main_rotor_vel *=
      sign(body_y.DotProduct(m_rotor_angular_momentum_in_world));

  engine::mat_get(m_rotor_rotation, 0, 0) = rot_x.x;
  engine::mat_get(m_rotor_rotation, 0, 1) = rot_x.y;
  engine::mat_get(m_rotor_rotation, 0, 2) = rot_x.z;
  engine::mat_get(m_rotor_rotation, 1, 0) = rot_y.x;
  engine::mat_get(m_rotor_rotation, 1, 1) = rot_y.y;
  engine::mat_get(m_rotor_rotation, 1, 2) = rot_y.z;
  engine::mat_get(m_rotor_rotation, 2, 0) = rot_z.x;
  engine::mat_get(m_rotor_rotation, 2, 1) = rot_z.y;
  engine::mat_get(m_rotor_rotation, 2, 2) = rot_z.z;
}

engine::vec3 BaseHeli::calc_body_rotor_reaction_moment(float time_delta) {

  // Calculate the body reaction moment.
  engine::vec3 body_up(engine::mat_get(m_body_rotation, 1, 0),
                       engine::mat_get(m_body_rotation, 1, 1),
                       engine::mat_get(m_body_rotation, 1, 2));
  engine::vec3 rotor_up(engine::mat_get(m_rotor_rotation, 1, 0),
                        engine::mat_get(m_rotor_rotation, 1, 1),
                        engine::mat_get(m_rotor_rotation, 1, 2));
  engine::vec3 body_reaction_moment_in_world =
      rotor_up.CrossProduct(body_up) * m_params.rigidness;

  engine::vec3 body_reaction_moment =
      m_rotor_rotation.Transpose() * body_reaction_moment_in_world;

  // Anti-wobliness moment.
  engine::vec3 dbody_reaction_moment =
      (body_reaction_moment - m_prev_reaction_in_body) / time_delta;
  m_prev_reaction_in_body = body_reaction_moment;
  engine::vec3 anti_wobliness_in_world =
      m_rotor_rotation * dbody_reaction_moment;
  body_reaction_moment_in_world +=
      anti_wobliness_in_world * m_params.anti_wobliness;

  return body_reaction_moment_in_world;
}

void BaseHeli::calc_tail_rotor_force(const engine::vec3 &wind_speed,
                                     engine::vec3 &out_force_in_world,
                                     engine::vec3 &out_torque_in_world) {
  float main_rotor_effectiveness =
      m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

  engine::vec3 tail_torque_in_rotor_coords(
      0,
      m_yaw_servo.get() * m_params.tail_rotor_max_force * m_params.tail_length,
      0);
  tail_torque_in_rotor_coords *= main_rotor_effectiveness;
  float tail_rotor_force_magnitude =
      tail_torque_in_rotor_coords.y / m_params.tail_length;
  std::cout << "Yaw servo " << m_yaw_servo.get() << std::endl;
  engine::vec3 heli_right(engine::mat_get(m_rotor_rotation, 0, 0),
                          engine::mat_get(m_rotor_rotation, 0, 1),
                          engine::mat_get(m_rotor_rotation, 0, 2));
  engine::vec3 tail_rotor_force_in_world =
      tail_rotor_force_magnitude * heli_right;

  engine::vec3 tail_torque_in_world =
      m_body_rotation * tail_torque_in_rotor_coords;

  out_force_in_world = tail_rotor_force_in_world;
  out_torque_in_world = tail_torque_in_world;
}

engine::vec3 BaseHeli::calc_engine_torque() {
  float main_rotor_effectiveness =
      m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

  float throttle_ratio =
      (m_throttle_servo.get() + 1) / 2; // Change from (-1, 1) to (0, 1).
  m_main_rotor_target_rps = m_params.main_rotor_max_vel * throttle_ratio;
  float target_rotor_omega = m_main_rotor_target_rps * 2 * PI;
  float main_rotor_omega = m_rotor_angular_momentum_in_world.Length() /
                           m_params.rotor_moment_of_inertia;
  float main_rotor_torque;
  float omega_ratio = main_rotor_omega / target_rotor_omega;
  if (omega_ratio < 0.9) {
    main_rotor_torque =
        m_params.main_rotor_torque * (main_rotor_effectiveness + 0.1);
  } else {
    main_rotor_torque =
        (1 - (omega_ratio - 0.9) * 10) * m_params.main_rotor_torque;
  }
  main_rotor_torque = (main_rotor_torque < 0) ? 0 : main_rotor_torque;

  // Calculate the motor drag torque, trying to hold back the rotor.
  float max_motor_drag_torque = 0.01 * m_params.main_rotor_torque;
  float motor_drag_torque = -sign(m_main_rotor_vel) * max_motor_drag_torque;
  main_rotor_torque += motor_drag_torque;

  engine::vec3 rotor_y(engine::mat_get(m_rotor_rotation, 1, 0),
                       engine::mat_get(m_rotor_rotation, 1, 1),
                       engine::mat_get(m_rotor_rotation, 1, 2));
  return main_rotor_torque * rotor_y;
}

engine::vec3 BaseHeli::calc_swash_torque() {
  float main_rotor_effectiveness =
      m_main_rotor_vel / 360 / m_params.main_rotor_max_vel;

  // Note that the roll and pitch are switched due to the gyro 90 deg effect.
  engine::vec3 swash_torque_in_rotor_coords(
      -m_roll_servo.get() * m_params.swash_torque, 0,
      m_pitch_servo.get() * m_params.swash_torque);
  swash_torque_in_rotor_coords *= main_rotor_effectiveness;
  engine::vec3 swash_torque_in_world =
      m_rotor_rotation * swash_torque_in_rotor_coords;
  return swash_torque_in_world;
}

engine::vec3 BaseHeli::torbulant_force(const engine::vec3 &pos_in_world,
                                       const engine::vec3 &airspeed_in_world,
                                       const engine::vec3 &lift_in_world) {
  // Each torbulant point has a target velocity in which the torbulant effect
  // start effcting. On points that get fresh air should have half of the
  // of the velocity.
  float pos_aligned =
      (pos_in_world / pos_in_world.Length())
          .DotProduct(airspeed_in_world / (airspeed_in_world.Length() + 0.1));
  pos_aligned = pos_aligned > 1 ? 1 : pos_aligned;
  pos_aligned = pos_aligned < -1 ? -1 : pos_aligned;

  // The target velocity changes from 30% on the back to 90% on the top
  float target_vel = m_params.torbulant_airspeed * 0.6;
  target_vel += pos_aligned * m_params.torbulant_airspeed * 0.3;

  float torbulant_coeff = 1 - airspeed_in_world.Length() / target_vel;
  torbulant_coeff = torbulant_coeff < 0 ? 0 : torbulant_coeff;

  return -MAX_TORBULANT_EFFECT * lift_in_world * torbulant_coeff / 4;
}

void BaseHeli::update_torbulation(float time_delta,
                                  const engine::vec3 &lift_in_world,
                                  const engine::vec3 &wind_speed,
                                  engine::vec3 &out_torbulant_force_in_world,
                                  engine::vec3 &out_torbulant_torque_in_world) {
  engine::vec3 airspeed_in_world = -(m_v - wind_speed);

  engine::vec3 front = engine::vec3(engine::mat_get(m_rotor_rotation, 2, 0),
                                    engine::mat_get(m_rotor_rotation, 2, 1),
                                    engine::mat_get(m_rotor_rotation, 2, 2)) *
                       m_params.main_rotor_length / 2;
  engine::vec3 right = engine::vec3(engine::mat_get(m_rotor_rotation, 0, 0),
                                    engine::mat_get(m_rotor_rotation, 0, 1),
                                    engine::mat_get(m_rotor_rotation, 0, 2)) *
                       m_params.main_rotor_length / 2;

  engine::vec3 front_torbulation_force =
      torbulant_force(front, airspeed_in_world, lift_in_world) *
      (0.75 + 0.25 * m_torbulant_rand_front.update(time_delta));
  engine::vec3 back_torbulation_force =
      torbulant_force(-front, airspeed_in_world, lift_in_world) *
      (0.75 + 0.25 * m_torbulant_rand_back.update(time_delta));
  engine::vec3 right_torbulation_force =
      torbulant_force(right, airspeed_in_world, lift_in_world) *
      (0.75 + 0.25 * m_torbulant_rand_right.update(time_delta));
  engine::vec3 left_torbulation_force =
      torbulant_force(-right, airspeed_in_world, lift_in_world) *
      (0.75 + 0.25 * m_torbulant_rand_left.update(time_delta));
  out_torbulant_force_in_world =
      front_torbulation_force + back_torbulation_force +
      left_torbulation_force + right_torbulation_force;
  out_torbulant_torque_in_world = front_torbulation_force.CrossProduct(front) +
                                  back_torbulation_force.CrossProduct(-front) +
                                  right_torbulation_force.CrossProduct(right) +
                                  left_torbulation_force.CrossProduct(-right);
}

void BaseHeli::calc_aerodynamic_drag(const engine::vec3 &wind_speed,
                                     engine::vec3 &out_force_in_world,
                                     engine::vec3 &out_torque_in_world) {
  engine::vec3 airspeed_in_world = -(m_v - wind_speed);

  // Calculate the drag force.
  engine::vec3 drag_vec = m_params.drag;
  engine::mat4 world_to_heli = m_rotor_rotation.Transpose();
  engine::vec3 airspeed_in_heli = world_to_heli * airspeed_in_world;

  engine::vec3 aerodynamic_drag_force =
      m_rotor_rotation * drag_vec *
      airspeed_in_heli; // Back in world coord system.

  // calculate the tail-drag torque.
  engine::vec3 heli_right_in_world(engine::mat_get(m_body_rotation, 0, 0),
                                   engine::mat_get(m_body_rotation, 0, 1),
                                   engine::mat_get(m_body_rotation, 0, 2));
  float tail_wind = heli_right_in_world.DotProduct(airspeed_in_world);
  tail_wind += m_body_angularv_in_body_coords.y * m_params.tail_length;
  float tail_drag_moment_y =
      -tail_wind * m_params.tail_drag * m_params.tail_length;
  engine::vec3 aerodynamic_drag_torque(0, tail_drag_moment_y, 0);
  engine::vec3 total_tail_torque_in_world =
      m_body_rotation * aerodynamic_drag_torque;

  // Return;
  out_force_in_world = -aerodynamic_drag_force;
  out_torque_in_world = -aerodynamic_drag_torque;
}

engine::vec3 BaseHeli::calc_dissimetry_of_lift(const engine::vec3 &wind_speed,
                                               float lift_magnitude) {
  engine::vec3 airspeed_in_world = -(m_v - wind_speed);
  float rotor_radius = m_params.main_rotor_length / 2.;
  float rotor_omega = m_main_rotor_vel / 360 * 2 * PI;
  float max_airspeed = rotor_omega * rotor_radius * 0.8 + 0.0001;

  engine::vec3 rotor_up(engine::mat_get(m_rotor_rotation, 1, 0),
                        engine::mat_get(m_rotor_rotation, 1, 1),
                        engine::mat_get(m_rotor_rotation, 1, 2));
  engine::vec3 transient_flow =
      airspeed_in_world - rotor_up * airspeed_in_world.DotProduct(rotor_up);
  float effect_magnitude = transient_flow.Length() / max_airspeed;

  engine::vec3 dol_direction =
      transient_flow / (transient_flow.Length() + 0.0001);

  return dol_direction * effect_magnitude * lift_magnitude * rotor_radius * 0.9;
}

// The lift forces and engine torques.
void BaseHeli::calc_lift_force(float time_delta, const engine::vec3 &wind_speed,
                               engine::vec3 &out_force_in_world,
                               engine::vec3 &out_rotor_torque_in_world) {
  engine::vec3 rotor_up(engine::mat_get(m_rotor_rotation, 1, 0),
                        engine::mat_get(m_rotor_rotation, 1, 1),
                        engine::mat_get(m_rotor_rotation, 1, 2));

  // Calc the rotor inflow.
  engine::vec3 airspeed_in_world = -(m_v - wind_speed);
  float rotor_inflow_velocity = -rotor_up.DotProduct(airspeed_in_world);
  float rotor_radius = m_params.main_rotor_length / 2.;

  // Calc the rotor outflow.
  float rotor_angle_of_attack = m_params.main_rotor_max_angle_of_attack *
                                m_lift_servo.get() / 360 * 2 * PI;
  float rotor_omega = m_main_rotor_vel / 360 * 2 * PI;
  float rotor_target_outflow_velocity =
      rotor_omega * std::tan(rotor_angle_of_attack) * rotor_radius * 0.8;
  float rotor_outflow_velocity =
      m_params.main_rotor_traction * rotor_target_outflow_velocity +
      (1 - m_params.main_rotor_traction) * rotor_inflow_velocity;

  // Calc the lift force.
  float rotor_surface_area =
      rotor_radius * rotor_radius * PI * 0.8; // Cicle without the center.
  float rotor_mass_flow =
      rotor_outflow_velocity * rotor_surface_area * AIR_DENSITY; // [Mass / Sec]
  float rotor_lift_force_magnitude =
      std::abs(rotor_mass_flow) *
      (rotor_outflow_velocity - rotor_inflow_velocity);
  engine::vec3 lift = rotor_up * rotor_lift_force_magnitude;

  // Account for torbulation force and torque.
  engine::vec3 torbulant_force_in_world, torbulant_torque_in_world;
  update_torbulation(time_delta, lift, wind_speed, torbulant_force_in_world,
                     torbulant_torque_in_world);

  // Calculate the rotor drag force, trying to slow down the rotor.
  engine::vec3 rotor_drag_torque =
      -(lift + torbulant_force_in_world).DotProduct(rotor_up) *
      m_params.main_rotor_length /
      3 // Geometric coeffcient from rotor lift to drag torque.
      * std::tan(rotor_angle_of_attack) // Ratio between wing's lift and drag.
      * rotor_up;                       // Directed upwards.

  // Account for Dissimetry of Lift.
  engine::vec3 dissimetry_of_lift_torque =
      calc_dissimetry_of_lift(wind_speed, rotor_lift_force_magnitude);

  // Ground effect force.
  float ground_effect_intensity = (m_pos.y > 0.3) ? 0 : ((0.3 - m_pos.y) / 0.3);
  lift *= (1 + ground_effect_intensity);

  // Return;
  out_force_in_world = lift + torbulant_force_in_world;
  out_rotor_torque_in_world =
      rotor_drag_torque + torbulant_torque_in_world + dissimetry_of_lift_torque;
}

ServoFilter &BaseHeli::get_servo(int channel) {
  switch (channel) {
  case HELI_CHANNEL_PITCH:
    return m_pitch_servo;
  case HELI_CHANNEL_ROLL:
    return m_roll_servo;
  case HELI_CHANNEL_YAW:
    return m_yaw_servo;
  case HELI_CHANNEL_LIFT:
    return m_lift_servo;
  case HELI_CHANNEL_THROTTLE:
    return m_throttle_servo;
  default:
    throw std::runtime_error("Invalid channel");
  }
}

void BaseHeli::update(double time_delta, const engine::vec3 &wind_speed) {

  // Engine torque.
  engine::vec3 engine_torque_in_world = calc_engine_torque();

  // Swash torques.
  engine::vec3 swash_torque_in_world = calc_swash_torque();

  // Tail torque and thrust force.
  engine::vec3 total_tail_torque_in_world, tail_rotor_force_in_world;
  calc_tail_rotor_force(wind_speed, tail_rotor_force_in_world,
                        total_tail_torque_in_world);

  // The body-rotor reaction forces.
  engine::vec3 body_reaction_moment_in_world =
      calc_body_rotor_reaction_moment(time_delta);

  // Gravity force.
  engine::vec3 gravity(0, -GRAVITY_CONSTANT * m_params.mass, 0);

  // Aerodynamic drag force and torque.
  engine::vec3 aerodynamic_drag_force, aerodynamic_drag_torque;
  calc_aerodynamic_drag(wind_speed, aerodynamic_drag_force,
                        aerodynamic_drag_torque);

  // Lift force.
  engine::vec3 lift_force(0, 0, 0), rotor_lift_torque(0, 0, 0);
  calc_lift_force(time_delta, wind_speed, lift_force, rotor_lift_torque);

  // Limit the external torque.
  engine::vec3 external_torque = m_external_torque;
  if (external_torque.Length() > m_params.external_torque_limit) {
    external_torque /=
        external_torque.Length() / m_params.external_torque_limit;
  }

  // Update pos/speed.
  engine::vec3 total_force = gravity + lift_force - aerodynamic_drag_force -
                             tail_rotor_force_in_world + m_external_force;
  engine::vec3 acc = total_force / m_params.mass;
  m_v += time_delta * acc;
  m_pos += time_delta * m_v;

  // Update rotations/angular velocities.
  update_body_moments(time_delta,
                      total_tail_torque_in_world + aerodynamic_drag_torque -
                          engine_torque_in_world -
                          body_reaction_moment_in_world + external_torque);
  update_rotor_moments(time_delta, swash_torque_in_world + rotor_lift_torque +
                                       engine_torque_in_world +
                                       body_reaction_moment_in_world);

  // Update UI.
  update_ui(time_delta);
}

void BaseHeli::add_force(unsigned int touchpoint_index,
                         const engine::vec3 &force) {
  m_external_force += force;
  m_external_torque +=
      (m_body_rotation * m_params.touchpoints_in_heli[touchpoint_index])
          .CrossProduct(force);
}

std::vector<BaseHeli::TouchPoint> BaseHeli::get_touchpoints_in_world() const {
  std::vector<BaseHeli::TouchPoint> touchpoints_in_world;
  for (auto touchpoint_in_body : m_params.touchpoints_in_heli) {
    BaseHeli::TouchPoint tp;
    tp.pos = m_pos + m_body_rotation * touchpoint_in_body;
    tp.vel =
        m_v + m_body_rotation * m_body_angularv_in_body_coords.CrossProduct(
                                    touchpoint_in_body);
    tp.friction_coeff = diag2(15.0f, 15.0f);
    touchpoints_in_world.push_back(tp);
  }
  return touchpoints_in_world;
}

BaseHeli::Telemetry BaseHeli::get_telemetry() const {
  Telemetry telemetry;
  telemetry.rps = m_main_rotor_vel / 360;
  telemetry.target_rps = m_main_rotor_target_rps;
  return telemetry;
}

// class MainRotorBlur : public RotorBlur {
// public:
//   MainRotorBlur(irr::scene::ISceneManager *smgr,
//                 irr::scene::IMeshSceneNode *parent) {
//     init_ui(smgr,
//             3.75,                      // radius.
//             irrvec3(-0.0, 1.8 / 4, 0), // position.
//             irrvec3(0, 0, 0),          // rotation.
//             0.03,                      // thichkess.
//             parent                     // parent_node
//     );
//   }

// private:
//   virtual float get_width(float along_radius) const {
//     if (along_radius < 0.2)
//       return 0.2;
//     return 1.;
//   }
//   virtual irr::video::SColor get_color(float along_radious) const {
//     if (along_radious < 0.2)
//       return irr::video::SColor(255, 128, 128, 128);
//     if (along_radious > 3.52 && along_radious < 3.6)
//       return irr::video::SColor(255, 255, 255, 255);
//     else
//       return irr::video::SColor(255, 0, 0, 0);
//   }
// };

// class FlybarBlur : public RotorBlur {
// public:
//   FlybarBlur(irr::scene::ISceneManager *smgr,
//              irr::scene::IMeshSceneNode *parent) {
//     init_ui(smgr,
//             0.62,                // radius.
//             irrvec3(0, 0.57, 0), // position.
//             irrvec3(0, 0, 0),    // rotation.
//             0.03,                // thichkess.
//             parent               // parent_node
//     );
//   }

// private:
//   virtual float get_width(float along_radius) const {
//     if (along_radius > 0.57)
//       return 0.7;
//     else if (along_radius < 0.3)
//       return 0.7;
//     else
//       return 0.5;
//   }
//   virtual irr::video::SColor get_color(float along_radious) const {
//     return irr::video::SColor(255, 128, 128, 128);
//   }
// };

// class TailRotorBlur : public RotorBlur {
// public:
//   TailRotorBlur(irr::scene::ISceneManager *smgr,
//                 irr::scene::IMeshSceneNode *parent) {
//     init_ui(smgr,
//             0.66,                        // radius.
//             irrvec3(-4.62, 1.54, -0.14), // position.
//             irrvec3(90, 0, 0),           // rotation.
//             0.03,                        // thichkess.
//             parent                       // parent_node
//     );
//   }

// private:
//   virtual float get_width(float along_radius) const { return 0.4; }
//   virtual irr::video::SColor get_color(float along_radious) const {
//     if (along_radious > 0.53 && along_radious < 0.64)
//       return irr::video::SColor(255, 255, 255, 255);
//     return irr::video::SColor(255, 0, 0, 0);
//   }
// };

const struct HeliParams RC_BELL_AERODYNAMICS = {
    .init_pos = engine::vec3(0, 0.13 + -0.019, 0),
    .init_rotation = engine::vec3(0, 0, 0),
    .mass = 2.,
    .drag = engine::vec3(1, 0.6, 0.2),
    .torbulant_airspeed = 7,
    .main_rotor_max_vel = 35,
    .main_rotor_torque = 8.,
    .main_rotor_length = 1.,
    .main_rotor_max_angle_of_attack = 12.,
    .main_rotor_traction = 0.9,

    .tail_length = 0.6,
    .tail_drag = 0.1,
    .tail_rotor_max_force = 30,

    .swash_torque = 4 * 10,                       // ~= Mass * G * 1M
    .rotor_moment_of_inertia = 1. / 12 * 0.3 * 1, //  = Rod: 1/12 * M * L^2
    .body_moment_of_inertia = engine::vec3(
        // pitch: 2 masses - one in the rotor and one in the body.
        (0.2 * 0.5 * 0.5) + 1.5 * 0.15 * 0.15,
        // yaw: 2 masses - one in the tail and one close.
        (0.1 * 0.6 * 0.6) + (0.5 * 0.1 * 0.1),
        // roll: 1 masses - the body.
        (1.5 * 0.1 * 0.1)),

    .rigidness = 20,
    .anti_wobliness = 1. / 16,

    .touchpoints_in_heli = std::vector<engine::vec3>(
        {engine::vec3(0.19 * (5. / 6.), -0.128 * (5. / 6.), 0.17 * (5. / 6.)),
         engine::vec3(-0.19 * (5. / 6.), -0.128 * (5. / 6.), 0.17 * (5. / 6.)),
         engine::vec3(-0.19 * (5. / 6.), -0.128 * (5. / 6.), -0.28 * (5. / 6.)),
         engine::vec3(0.19 * (5. / 6.), -0.128 * (5. / 6.), -0.28 * (5. / 6.)),
         engine::vec3(0, 0.05, -1.12 * (5. / 6.))}),

    .external_torque_limit = 1};

RcBellHeli::RcBellHeli(engine::RaylibDevice *device)
    : BaseHeli(RC_BELL_AERODYNAMICS),
      m_texture("resources/media/Bell/textures/1001_albedo.jpg") {
  // Create the body mesh.
  m_body_node = device->load_model("resources/media/Bell/source/bell_body.obj");
  auto materials = m_body_node->get_materials();
  for (engine::Material *material : materials) {
    material->SetTexture(MATERIAL_MAP_DIFFUSE, m_texture);
  }

  // Create the main rotor mesh.
  m_rotor_node =
      device->load_model("resources/media/Bell/source/bell_main_rotor.obj");
  m_rotor_node->set_transform(engine::mat4::Translate(0.45, -1.4, 0));
  materials = m_rotor_node->get_materials();
  for (engine::Material *material : materials) {
    material->SetTexture(MATERIAL_MAP_DIFFUSE, m_texture);
  }

  // Create the tail rotor mesh.
  m_tail_rotor_node = device->load_model(
      "resources/media/Bell/source/bell_tail_rotor.obj", m_body_node);
  m_tail_rotor_node->set_transform(engine::mat4::Translate(4.62, -1.54, 0));
  materials = m_tail_rotor_node->get_materials();
  for (engine::Material *material : materials) {
    material->SetTexture(MATERIAL_MAP_DIFFUSE, m_texture);
  }

  // m_main_rotor_blur = std::make_shared<MainRotorBlur>(smgr, m_rotor_node);
  // m_flybar_blur = std::make_shared<FlybarBlur>(smgr, m_rotor_node);
  // m_tail_prop_blur = std::make_shared<TailRotorBlur>(smgr, m_body_node);
  m_main_rotor_angle = 0;
  m_tail_rotor_angle = 0;

  update_ui(0);
}

void RcBellHeli::update_ui(float time_delta) {
  m_body_node->set_transform(
      engine::mat4::Scale(1. / 5, 1. / 5, 1. / 5) *
      engine::mat4::RotateY(90. / 180 * PI) * m_body_rotation *
      engine::mat4::Translate(m_pos.x, m_pos.y, m_pos.z));

  m_main_rotor_angle += m_main_rotor_vel * time_delta / 180 * PI;
  m_rotor_node->set_transform(
      engine::mat4::Scale(1. / 5, 1. / 5, 1. / 5) *
      engine::mat4::Translate(-0.09, -0.28, 0) *
      engine::mat4::RotateY(m_main_rotor_angle) *
      engine::mat4::Translate(0.09, 0.28, 0) *
      engine::mat4::RotateY(90. / 180 * PI) * m_rotor_rotation *
      engine::mat4::Translate(m_pos.x, m_pos.y, m_pos.z));
  
  m_tail_rotor_angle += 4 * m_main_rotor_vel * time_delta / 180 * PI;
  m_tail_rotor_node->set_transform(
      engine::mat4::Translate(-4.62, -1.54, 0)
      * engine::mat4::RotateZ(m_tail_rotor_angle)
      * engine::mat4::Translate(4.62, 1.54, 0)
  );

  // m_tail_rotor_node->setPosition(m_pos);
  // m_tail_rotor_node->setRotation(
  //     (m_rotor_rotation * m_shape_rotation).getRotationDegrees());

  // // Set the main rotor rotation.
  // irr::core::matrix4 main_rotor_rotation;
  // main_rotor_rotation.setRotationDegrees(irrvec3(0, m_main_rotor_angle, 0));
  // m_main_rotor_angle += m_main_rotor_vel * time_delta;
  // irrvec3 main_rotor_offset(0.45, -1.4, 0);
  // m_rotor_node->setPosition(-main_rotor_offset);
  // irr::core::matrix4 rotor_rotation_in_body_coords =
  //     m_body_rotation.getTransposed() * m_rotor_rotation;
  // m_rotor_node->setRotation(
  //     (rotor_rotation_in_body_coords * main_rotor_rotation)
  //         .getRotationDegrees());

  // // Set the tail rotor rotation.
  // irr::core::matrix4 tail_rotor_rotation;
  // tail_rotor_rotation.setRotationDegrees(irrvec3(0, 0, m_tail_rotor_angle));
  // m_tail_rotor_angle += 3 * m_main_rotor_vel * time_delta;
  // irrvec3 tail_rotor_offset(2.31 * 2, -0.77 * 2, 0);
  // m_tail_rotor_node->setPosition(-tail_rotor_offset);
  // m_tail_rotor_node->setRotation((tail_rotor_rotation).getRotationDegrees());
}

// const struct HeliParams BELL_AERODYNAMICS = {
//     .init_pos = irrvec3(0, (0.13 + -0.019) * 5, 0),
//     .init_rotation = irrvec3(0, 0, 0),
//     .mass = 3000.,
//     .drag = irrvec3(1, 0.6, 0.2),
//     .torbulant_airspeed = 7,
//     .main_rotor_max_vel = 20,
//     .main_rotor_torque = 30000.,
//     .main_rotor_length = 14.,
//     .main_rotor_max_angle_of_attack = 8.,
//     .main_rotor_traction = 0.4,

//     .tail_length = 9,
//     .tail_drag = 10,
//     .tail_rotor_max_force = 8000,

//     .swash_torque = 3000 * 10 * 12, // ~= Mass * G
//     .rotor_moment_of_inertia =
//         3742, //
//         https://www.quora.com/What-is-the-weight-of-a-normal-light-weighted-helicopter-rotor-blades
//     .body_moment_of_inertia = irrvec3(
//         // pitch: 2 masses - one in the tail and one in the body.
//         100 * 9 * 9 + 2500 * 1 * 1,
//         // yaw: 2 masses - one in the tail and one close.
//         100 * 9 * 9 + 2500 * 1 * 1,
//         // roll: 1 masses - the body.
//         (2500 * 1 * 1)),

//     .rigidness = 80000,
//     .anti_wobliness = 1. / 10,

//     .touchpoints_in_heli = std::vector<irrvec3>(
//         {irrvec3(0.19 * (5. / 6.) * 5, -0.12 * (5. / 6.) * 5,
//                  0.17 * (5. / 6.) * 5),
//          irrvec3(-0.19 * (5. / 6.) * 5, -0.12 * (5. / 6.) * 5,
//                  0.17 * (5. / 6.) * 5),
//          irrvec3(-0.19 * (5. / 6.) * 5, -0.12 * (5. / 6.) * 5,
//                  -0.28 * (5. / 6.) * 5),
//          irrvec3(0.19 * (5. / 6.) * 5, -0.12 * (5. / 6.) * 5,
//                  -0.28 * (5. / 6.) * 5),
//          irrvec3(0, 0.05 * 5, -1.12 * (5. / 6.) * 5)}),

//     .external_torque_limit = 10000};

// BellHeli::BellHeli(irr::scene::ISceneManager *smgr,
//                    irr::video::IVideoDriver *driver)
//     : BaseHeli(BELL_AERODYNAMICS) {
//   // Create the body mesh.
//   irr::scene::IMesh *heli_mesh =
//       smgr->getMesh("media/Bell/source/bell_body.obj");
//   m_body_node = smgr->addMeshSceneNode(heli_mesh);
//   m_body_node->setScale(irrvec3(1., 1., 1.));
//   m_body_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
//   m_body_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
//   m_body_node->setDebugDataVisible(irr::scene::EDS_OFF);
//   m_body_node->setMaterialTexture(
//       0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
//   m_body_node->addShadowVolumeSceneNode();
//   for (unsigned int i = 0; i < m_body_node->getMaterialCount(); i++) {
//     m_body_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
//   }

//   // Create the main rotor mesh.
//   irr::scene::IMesh *main_rotor_mesh =
//       smgr->getMesh("media/Bell/source/bell_main_rotor.obj");
//   irr::core::matrix4 main_rotor_translation;
//   main_rotor_translation.setTranslation(irrvec3(0.45, -1.4, 0));
//   smgr->getMeshManipulator()->apply(
//       irr::scene::SVertexPositionTransformManipulator(main_rotor_translation),
//       main_rotor_mesh);
//   m_rotor_node = smgr->addMeshSceneNode(main_rotor_mesh, m_body_node);
//   m_rotor_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
//   m_rotor_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
//   m_rotor_node->setDebugDataVisible(irr::scene::EDS_OFF);
//   m_rotor_node->setMaterialTexture(
//       0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
//   m_rotor_node->addShadowVolumeSceneNode();
//   for (unsigned int i = 0; i < m_rotor_node->getMaterialCount(); i++) {
//     m_rotor_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
//   }

//   // Create the tail rotor mesh.
//   irr::scene::IMesh *tail_rotor_mesh =
//       smgr->getMesh("media/Bell/source/bell_tail_rotor.obj");
//   irr::core::matrix4 tail_rotor_translation;
//   tail_rotor_translation.setTranslation(irrvec3(4.62, -1.54, 0));
//   smgr->getMeshManipulator()->apply(
//       irr::scene::SVertexPositionTransformManipulator(tail_rotor_translation),
//       tail_rotor_mesh);
//   m_tail_rotor_node = smgr->addMeshSceneNode(tail_rotor_mesh, m_body_node);
//   m_tail_rotor_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
//   m_tail_rotor_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS,
//   true); m_tail_rotor_node->setDebugDataVisible(irr::scene::EDS_OFF);
//   m_tail_rotor_node->setMaterialTexture(
//       0, driver->getTexture("media/Bell/textures/1001_albedo.jpg"));
//   m_tail_rotor_node->addShadowVolumeSceneNode();
//   for (unsigned int i = 0; i < m_tail_rotor_node->getMaterialCount(); i++) {
//     m_tail_rotor_node->getMaterial(i).AmbientColor.set(255, 255, 255, 255);
//   }

//   m_shape_rotation.setRotationDegrees(irrvec3(0, -90, 0));

//   m_main_rotor_blur = std::make_shared<MainRotorBlur>(smgr, m_rotor_node);
//   m_flybar_blur = std::make_shared<FlybarBlur>(smgr, m_rotor_node);
//   m_tail_prop_blur = std::make_shared<TailRotorBlur>(smgr, m_body_node);
//   m_main_rotor_angle = 0;
//   m_tail_rotor_angle = 0;

//   update_ui(0);
// }

// void BellHeli::update_ui(float time_delta) {
//   m_body_node->setPosition(m_pos);
//   m_tail_rotor_node->setPosition(m_pos);

//   m_body_node->setRotation(
//       (m_body_rotation * m_shape_rotation).getRotationDegrees());
//   m_tail_rotor_node->setRotation(
//       (m_rotor_rotation * m_shape_rotation).getRotationDegrees());

//   // Set the main rotor rotation.
//   irr::core::matrix4 main_rotor_rotation;
//   main_rotor_rotation.setRotationDegrees(irrvec3(0, m_main_rotor_angle, 0));
//   m_main_rotor_angle += m_main_rotor_vel * time_delta;
//   irrvec3 main_rotor_offset(0.45, -1.4, 0);
//   m_rotor_node->setPosition(-main_rotor_offset);
//   irr::core::matrix4 rotor_rotation_in_body_coords =
//       m_body_rotation.getTransposed() * m_rotor_rotation;
//   m_rotor_node->setRotation(
//       (rotor_rotation_in_body_coords * main_rotor_rotation)
//           .getRotationDegrees());

//   // Set the tail rotor rotation.
//   irr::core::matrix4 tail_rotor_rotation;
//   tail_rotor_rotation.setRotationDegrees(irrvec3(0, 0, m_tail_rotor_angle));
//   m_tail_rotor_angle += 3 * m_main_rotor_vel * time_delta;
//   irrvec3 tail_rotor_offset(2.31 * 2, -0.77 * 2, 0);
//   m_tail_rotor_node->setPosition(-tail_rotor_offset);
//   m_tail_rotor_node->setRotation((tail_rotor_rotation).getRotationDegrees());
// }
