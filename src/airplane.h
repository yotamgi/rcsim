#ifndef __AIRPLANE_H__
#define __AIRPLANE_H__

// #include "arrow.h"
#include "flying_object.h"
#include "raylib_engine.h"
#include <map>
#include <memory>
#include <vector>

enum AirplaneChannels {
  AIRPLANE_CHANNEL_THROTTLE = 0,
  AIRPLANE_CHANNEL_PITCH = 1,
  AIRPLANE_CHANNEL_ROLL = 2,
  AIRPLANE_CHANNEL_YAW = 3,
  AIRPLANE_CHANNEL_FLAPRON = 4,
  AIRPLANE_CHANNEL_FLAPS = 5,
};

struct AppliedForce {
  engine::vec3 force;
  engine::vec3 position_in_airplane;
};

struct PointAirFoil {
public:
  struct Params {
    float area;
    float flap_area;
    // The drag is only applied on the airflow_direction direction.
    float drag_area;
    engine::vec3 position_in_airplane;
    engine::vec3 normal;
    // While lift can happen in all directions, this indicates the
    // airflow expeected direction. This sets the flaps affect and the
    // stall/extra lift effects.
    engine::vec3 wing_airflow_direction;
    float stall_angle_min;
    float stall_angle_max;
  };

  PointAirFoil(const Params &params) : m_params(params), m_flap_normal(0) {}

  void set_flap_normal(const engine::vec3 &flap_normal) {
    m_flap_normal = flap_normal;
  };
  AppliedForce get_force(const engine::vec3 &airflow);
  const engine::vec3 &get_position() const {
    return m_params.position_in_airplane;
  }
  void init_ui(engine::RaylibDevice *device,
               std::shared_ptr<engine::Model> parent);

private:
  Params m_params;
  engine::vec3 m_flap_normal;

  // std::shared_ptr<Arrow> m_force_arrow = nullptr;
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
    engine::vec3 rotation_angles;
    engine::vec3 position_in_airplane;
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

  std::vector<AppliedForce> calc_force(const engine::vec3 &wind_in_airplane,
                                       const engine::vec3 &velocity_in_airplane,
                                       const engine::vec3 &omega_in_airplane);

  void set_flap(float value);

  // UI functions:
  void init_ui(engine::RaylibDevice *device,
               std::shared_ptr<engine::Model> parent);

private:
  std::vector<std::shared_ptr<PointAirFoil>> m_point_airfoils;
  Params m_params;
  engine::vec3 m_along_wing_direction;
  engine::vec3 m_wing_normal;
};

class Propellant {
public:
  struct Params {
    engine::vec3 direction_in_airplane;
    engine::vec3 position_in_airplane;
    float thrust_airspeed;
    float max_thrust;
  };

  Propellant(const Params &params) : m_params(params) {}

  AppliedForce calc_force(const engine::vec3 &wind_in_airplane,
                          const engine::vec3 &velocity_in_airplane);

  void set_throttle(float throttle) { m_throttle = 0.5f + throttle / 2.0f; }

  // // UI functions:
  // void init_ui(engine::RaylibDevice *device,
  //              irr::scene::ISceneNode *parent);

private:
  float m_throttle;
  Params m_params;
  // std::shared_ptr<Arrow> m_force_arrow = nullptr;
};

class Airplane : public FlyingObject {
public:
  struct Params {
    float mass;
    engine::vec3 moi;
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
    engine::vec3 init_position;
    engine::vec3 init_velocity;
    engine::vec3 init_rotation;

    // Debug shape.
    bool show_skeleton = true;
  };

  Airplane(const Params &params, engine::RaylibDevice *device);

  virtual ServoFilter &get_servo(int channel) { return m_servos[channel]; }
  virtual void update(double time_delta, const engine::vec3 &wind_speed);
  virtual void add_force(unsigned int touchpoints_index,
                         const engine::vec3 &force);
  virtual void reset_force();

  virtual std::vector<TouchPoint> get_touchpoints_in_world() const;
  virtual double get_mass() const { return m_params.mass; }
  virtual engine::vec3 get_position() const { return m_position_in_world; }
  virtual void set_position(const engine::vec3 new_pos) {
    m_position_in_world = new_pos;
  }
  virtual engine::vec3 get_velocity() const { return m_velocity_in_world; }
  virtual void set_velocity(const engine::vec3 new_v) {
    m_velocity_in_world = new_v;
  }

  virtual Telemetry get_telemetry() const;
  virtual double get_max_rps() const { return 100; }

protected:
  void init_ui(engine::RaylibDevice *device);
  virtual void update_ui();

  engine::vec3 m_position_in_world;
  engine::vec3 m_velocity_in_world;
  engine::vec3 m_angular_velocity_in_airplane;
  engine::mat4 m_rotation_in_world;

  engine::RaylibDevice *m_device;
  std::shared_ptr<engine::Model> m_ui_node;

  // External forces and torques.
  engine::vec3 m_external_force_in_world;
  engine::vec3 m_external_torque_in_airplane;

  std::vector<std::shared_ptr<RectangularAirFoil>> m_airfoils;
  std::vector<std::shared_ptr<Propellant>> m_propellants;

  std::vector<ServoFilter> m_servos;

  Params m_params;
  float m_airspeed;
};

#endif // __AIRPLANE_H__
