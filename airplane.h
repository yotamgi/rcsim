#ifndef __AIRPLANE_H__
#define __AIRPLANE_H__

#include "arrow.h"
#include "flying_object.h"
#include <irrlicht/irrlicht.h>
#include <map>
#include <memory>
#include <vector>

typedef irr::core::matrix4 irrmat4;

enum AirplaneChannels {
  AIRPLANE_CHANNEL_THROTTLE = 0,
  AIRPLANE_CHANNEL_PITCH = 1,
  AIRPLANE_CHANNEL_ROLL = 2,
  AIRPLANE_CHANNEL_YAW = 3,
  AIRPLANE_CHANNEL_FLAPRON = 4,
  AIRPLANE_CHANNEL_FLAPS = 5,
};

struct AppliedForce {
  irrvec3 force;
  irrvec3 position_in_airplane;
};

struct PointAirFoil {
public:
  struct Params {
    float area;
    float flap_area;
    // The drag is only applied on the airflow_direction direction.
    float drag_area;
    irrvec3 position_in_airplane;
    irrvec3 normal;
    // While lift can happen in all directions, this indicates the
    // airflow expeected direction. This sets the flaps affect and the
    // stall/extra lift effects.
    irrvec3 wing_airflow_direction;
    float stall_angle_min;
    float stall_angle_max;
  };

  PointAirFoil(const Params &params) : m_params(params) {}

  void set_flap_normal(const irrvec3 &flap_normal) {
    m_flap_normal = flap_normal;
  };
  AppliedForce get_force(const irrvec3 &airflow);
  const irrvec3 &get_position() const { return m_params.position_in_airplane; }

  void init_ui(irr::scene::ISceneManager *smgr,
               irr::video::IVideoDriver *driver,
               irr::scene::ISceneNode *parent);

private:
  Params m_params;
  irrvec3 m_flap_normal;

  // Needed to save calculation.
  irrvec3 m_along_wing_direction;

  std::shared_ptr<Arrow> m_force_arrow = nullptr;
};

/**
 * A rectangular AirFoil.                     Z
 *                                            ^
 *      ------------------------------------+  |
 *      +                                   |   -> X
 *      X   X   X   X   X   X   X   X   X   X
 *      +       +---------------+           |
 *      +-------+---------------+-----------+
 *
 * Example with
 *  - num_points=10
 *  - flap_start=2
 *  - flap_end=6
 */
class RectangularAirFoil {
public:
  struct Params {

    // Geometry params:
    float x_length;
    float z_width;
    float y_thickness;
    irrvec3 rotation_angles;
    irrvec3 position_in_airplane;
    int num_points;

    // Aerodynamic params:
    float stall_angle_min;
    float stall_angle_max;

    // Flap params:
    bool has_flap;
    float flap_area;
    int flap_point_from; // starting from left.
    int flap_point_to;
    float max_flap_angle;
    float flap_mid_angle = 0.;
    float min_flap_angle = 0.;
  };

  RectangularAirFoil(const Params &params);

  std::vector<AppliedForce> calc_force(const irrvec3 &wind_in_airplane,
                                       const irrvec3 &velocity_in_airplane,
                                       const irrvec3 &omega_in_airplane);

  void set_flap(float value);

  // UI functions:
  void init_ui(irr::scene::ISceneManager *smgr,
               irr::video::IVideoDriver *driver,
               irr::scene::ISceneNode *parent);

private:
  std::vector<std::shared_ptr<PointAirFoil>> m_point_airfoils;
  Params m_params;
  irrvec3 m_along_wing_direction;
  irrvec3 m_wing_normal;
};

class Propellant {
public:
  struct Params {
    irrvec3 direction_in_airplane;
    irrvec3 position_in_airplane;
    float thrust_airspeed;
    float max_thrust;
  };

  Propellant(const Params &params) : m_params(params) {}

  AppliedForce calc_force(const irrvec3 &wind_in_airplane,
                          const irrvec3 &velocity_in_airplane);

  void set_throttle(float throttle) { m_throttle = 0.5f + throttle / 2.0f; }

  // // UI functions:
  void init_ui(irr::scene::ISceneManager *smgr,
               irr::video::IVideoDriver *driver,
               irr::scene::ISceneNode *parent);

private:
  float m_throttle;
  Params m_params;
  std::shared_ptr<Arrow> m_force_arrow = nullptr;
};

class Airplane : public FlyingObject {
public:
  struct Params {
    float mass;
    irrvec3 moi;
    std::vector<RectangularAirFoil::Params> airfoils;
    std::map<int, int> channel_flap_mapping;
    std::vector<Propellant::Params> propellants;
    std::map<int, int> channel_prop_mapping;
    std::vector<TouchPoint> touchpoints_in_airplane;

    struct Wheel {
      int servo_index;
      float max_angle;
    };

    // Wheel parameters.
    std::map<size_t, Wheel> touchpoint_to_channel_mapping;

    // Servo parameters.
    std::vector<float> servo_max_rps;
    std::vector<float> servo_init_values;

    // Initial parameters.
    irrvec3 init_position;
    irrvec3 init_velocity;
    irrvec3 init_rotation;

    // Debug shape.
    bool show_skeleton = true;
  };

  Airplane(const Params &params, irr::scene::ISceneManager *smgr,
           irr::video::IVideoDriver *driver);

  virtual ServoFilter &get_servo(int channel) { return m_servos[channel]; }
  virtual void update(double time_delta, const irrvec3 &wind_speed);
  virtual void add_force(unsigned int touchpoints_index, const irrvec3 &force);
  virtual void reset_force();

  virtual std::vector<TouchPoint> get_touchpoints_in_world() const;
  virtual double get_mass() const { return m_params.mass; }
  virtual irrvec3 get_position() const { return m_position_in_world; }
  virtual void set_position(const irrvec3 new_pos) {
    m_position_in_world = new_pos;
  }
  virtual irrvec3 get_velocity() const { return m_velocity_in_world; }
  virtual void set_velocity(const irrvec3 new_v) {
    m_velocity_in_world = new_v;
  }

  virtual Telemetry get_telemetry() const;
  virtual double get_max_rps() const { return 100; }

protected:
  void init_ui(irr::scene::ISceneManager *smgr,
               irr::video::IVideoDriver *driver);
  virtual void update_ui();

  irrvec3 m_position_in_world;
  irrvec3 m_velocity_in_world;
  irrvec3 m_angular_velocity_in_airplane;
  irrmat4 m_rotation_in_world;

  // UI node
  irr::scene::ISceneNode *m_ui_node;

  // External forces and torques.
  irrvec3 m_external_force_in_world;
  irrvec3 m_external_torque_in_airplane;

  std::vector<std::shared_ptr<RectangularAirFoil>> m_airfoils;
  std::vector<std::shared_ptr<Propellant>> m_propellants;

  std::vector<ServoFilter> m_servos;

  Params m_params;
};

#endif // __AIRPLANE_H__
