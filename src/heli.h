#ifndef __HELI_H__
#define __HELI_H__

#include "flying_object.h"
#include "rotor_blur.h"
#include "smooth_rand.h"
#include <irrlicht/irrlicht.h>
#include <memory>
#include <string>
#include <vector>

typedef irr::core::vector3df irrvec3;

enum Channels {
  HELI_CHANNEL_THROTTLE = 0,
  HELI_CHANNEL_PITCH = 1,
  HELI_CHANNEL_ROLL = 2,
  HELI_CHANNEL_YAW = 3,
  HELI_CHANNEL_LIFT = 4,
};

struct HeliParams {
  irrvec3 init_pos;
  irrvec3 init_rotation;

  double mass;              //  [Kg]
  irrvec3 drag;             // [N / (M / SEC)]
  float torbulant_airspeed; // [M / SEC]

  float main_rotor_max_vel;             // [Rotations / SEC]
  float main_rotor_torque;              //  [N * M^2]
  float main_rotor_length;              // [M]
  float main_rotor_max_angle_of_attack; // [degrees]
  float main_rotor_traction; // [0 - 1] How much traction the rotor has on the
                             // air.

  float tail_length; // [M]
  float tail_drag;   // Tail drag to apply yaw torque on airspeed [Newton / [M /
                     // SEC]]
  float tail_rotor_max_force; //  The tail rotor maximal thrust force.

  double swash_torque;            //  The swash moment strength [N * M]
  float rotor_moment_of_inertia;  // [Kg * M] around rotor rotating axis.
  irrvec3 body_moment_of_inertia; //  Moment of inertia [Kg * M] coefficients
                                  //  around center of Mass.

  float rigidness; // [rad / [torque]] A number describing torque applied to
                   // keep the heli
                   //                   rotor connected
  float anti_wobliness; // [[torque] / [M/Sec]] A number describing the
                        // helicopter deformation friction, e.g. the eneregy
                        // loss due to non-rigid movements.
  std::vector<irrvec3> touchpoints_in_heli;

  float external_torque_limit; // [torque] Maximum external torque allowed on
                               // the helicopter.
                               //          High value here may cause numerical
                               //          instability when the heli touches the
                               //          ground.
};

class BaseHeli : public FlyingObject {
public:
  BaseHeli(const HeliParams &params);

  virtual ServoFilter &get_servo(int channel);
  virtual void update(double time_delta, const irrvec3 &wind_speed);
  virtual irrvec3 get_position() const { return m_pos; }
  virtual void set_position(const irrvec3 new_pos) { m_pos = new_pos; }
  virtual irrvec3 get_velocity() const { return m_v; }
  virtual void set_velocity(const irrvec3 new_v) { m_v = new_v; }
  virtual irrvec3 get_gyro_angularv() const {
    return m_body_angularv_in_body_coords;
  }

  virtual void add_force(unsigned int touchpoints_index, const irrvec3 &force);
  virtual void reset_force() {
    m_external_force = irrvec3();
    m_external_torque = irrvec3();
  }

  virtual std::vector<TouchPoint> get_touchpoints_in_world() const;

  virtual Telemetry get_telemetry() const;
  virtual double get_max_rps() const { return m_params.main_rotor_max_vel; }
  virtual double get_mass() const { return m_params.mass; }

protected:
  virtual void update_ui(float time_delta) = 0;
  void update_body_moments(float time_delta, const irrvec3 &moment_in_world);
  void update_rotor_moments(float time_delta, const irrvec3 &moment_in_world);
  irrvec3 m_v;
  irrvec3 m_pos;
  float m_main_rotor_vel;
  irrvec3 m_external_torque;
  irrvec3 m_external_force;

  // Torbulat related.
  irrvec3 torbulant_force(const irrvec3 &pos_in_world,
                          const irrvec3 &airspeed_in_world,
                          const irrvec3 &lift_in_world);
  void update_torbulation(float time_delta, const irrvec3 &lift_in_world,
                          const irrvec3 &wind_speed,
                          irrvec3 &out_torbulant_force_in_world,
                          irrvec3 &out_torbulant_torque_in_world);
  SmoothRandFloat m_torbulant_rand_front;
  SmoothRandFloat m_torbulant_rand_back;
  SmoothRandFloat m_torbulant_rand_left;
  SmoothRandFloat m_torbulant_rand_right;

  // Body-rotor forces.
  irrvec3 calc_body_rotor_reaction_moment(float time_delta);
  irrvec3 m_prev_reaction_in_body;

  // Tail force and torque.
  void calc_tail_rotor_force(const irrvec3 &wind_speed,
                             irrvec3 &out_force_in_world,
                             irrvec3 &out_torque_in_world);

  // Engine torque.
  irrvec3 calc_engine_torque();

  // The swash torque.
  irrvec3 calc_swash_torque();

  // The aerodynamic forces and torques.
  void calc_aerodynamic_drag(const irrvec3 &wind_speed,
                             irrvec3 &out_force_in_world,
                             irrvec3 &out_torque_in_world);

  // The lift forces and engine torques.
  void calc_lift_force(float time_delta, const irrvec3 &wind_speed,
                       irrvec3 &out_force_in_world,
                       irrvec3 &out_rotor_torqu_in_world);

  // The dissimetry of lift.
  irrvec3 calc_dissimetry_of_lift(const irrvec3 &wind_speed,
                                  float lift_magnitude);

  // Body and rotor orientation properties.
  irrvec3 m_rotor_angular_momentum_in_world;
  irr::core::matrix4 m_rotor_rotation;
  irr::core::matrix4 m_body_rotation;
  irrvec3 m_body_angularv_in_body_coords;

  // For telemetry, some parameters are stored;
  float m_main_rotor_target_rps;

  // Servos
  ServoFilter m_pitch_servo;
  ServoFilter m_roll_servo;
  ServoFilter m_yaw_servo;
  ServoFilter m_lift_servo;
  ServoFilter m_throttle_servo;

  HeliParams m_params;
};

class RcBellHeli : public BaseHeli {
public:
  RcBellHeli(irr::scene::ISceneManager *smgr, irr::video::IVideoDriver *driver);

private:
  virtual void update_ui(float time_delta);

  irr::core::matrix4 m_shape_rotation;
  irr::scene::IMeshSceneNode *m_body_node;
  irr::scene::IMeshSceneNode *m_rotor_node;
  irr::scene::IMeshSceneNode *m_tail_rotor_node;
  std::shared_ptr<RotorBlur> m_main_rotor_blur;
  std::shared_ptr<RotorBlur> m_flybar_blur;
  std::shared_ptr<RotorBlur> m_tail_prop_blur;

  float m_main_rotor_angle;
  float m_tail_rotor_angle;
};

class BellHeli : public BaseHeli {
public:
  BellHeli(irr::scene::ISceneManager *smgr, irr::video::IVideoDriver *driver);

private:
  virtual void update_ui(float time_delta);

  irr::core::matrix4 m_shape_rotation;
  irr::scene::IMeshSceneNode *m_body_node;
  irr::scene::IMeshSceneNode *m_rotor_node;
  irr::scene::IMeshSceneNode *m_tail_rotor_node;
  std::shared_ptr<RotorBlur> m_main_rotor_blur;
  std::shared_ptr<RotorBlur> m_flybar_blur;
  std::shared_ptr<RotorBlur> m_tail_prop_blur;

  float m_main_rotor_angle;
  float m_tail_rotor_angle;
};

#endif
