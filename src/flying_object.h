#ifndef __FLYING_OBJ__
#define __FLYING_OBJ__

#include <vector>
#include "raylib_engine.h"


static inline engine::mat4 diag2(float x, float z) {
  engine::mat4 mat;
  engine::mat_get(mat, 0, 0) = x;
  engine::mat_get(mat, 1, 1) = 0;
  engine::mat_get(mat, 2, 2) = z;
  return mat;
}


/** A helper function for inifinitisimal update of rotation matrix. */
void update_rotation_matrix(engine::mat4 &matrix, const engine::vec3 angularv);

class ServoFilter {
public:
  ServoFilter(float max_rps, float init_value)
      : m_max_rps(max_rps), m_current_status(init_value) {}

  // Values are expected to be in [-1, 1] range.
  float update(float value, float time_delta);
  float get() const { return m_current_status; }

private:
  float m_max_rps;
  float m_current_status;
};

class FlyingObject {
public:
  virtual ServoFilter &get_servo(int channel) = 0;
  virtual void update(double time_delta, const engine::vec3 &wind_speed) = 0;
  virtual void add_force(unsigned int touchpoints_index,
                         const engine::vec3 &force) = 0;
  virtual void reset_force() = 0;

  struct TouchPoint {
    engine::vec3 pos;
    engine::vec3 vel;
    engine::mat4 friction_coeff; // A symmetric matrix indicating the friction
                                 // coefficient in (x, z) direction. The Y direction
                                 // isn't used.
  };

  virtual std::vector<TouchPoint> get_touchpoints_in_world() const = 0;
  virtual double get_mass() const = 0;
  virtual engine::vec3 get_position() const = 0;
  virtual void set_position(const engine::vec3 new_pos) = 0;
  virtual engine::vec3 get_velocity() const = 0;
  virtual void set_velocity(const engine::vec3 new_v) = 0;
  // For telemetry.
  struct Telemetry {
    float rps;
    float target_rps;
    float velocity_magnitude;
    float airspeed;
  };

  virtual Telemetry get_telemetry() const = 0;
  virtual double get_max_rps() const = 0;
};

#endif // __FLYING_OBJ__
