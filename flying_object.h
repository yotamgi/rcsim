#ifndef __FLYING_OBJ__
#define __FLYING_OBJ__

#include <irrlicht/irrlicht.h>
#include <vector>

typedef irr::core::vector3df irrvec3;


/** A helper function for inifinitisimal update of rotation matrix. */
void update_rotation_matrix(irr::core::matrix4 &matrix,
                            const irrvec3 angularv);

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
  virtual ServoFilter & get_servo(int channel) = 0;
  virtual void update(double time_delta, const irrvec3 &wind_speed) = 0;
  virtual void add_force(unsigned int touchpoints_index,
                         const irrvec3 &force) = 0;
  virtual void reset_force() = 0;

  struct TouchPoint {
    irrvec3 pos_in_world;
    irrvec3 vel_in_world;
  };

  virtual std::vector<TouchPoint> get_touchpoints_in_world() const = 0;
  virtual double get_mass() const = 0;
  virtual irrvec3 get_position() const = 0;
  virtual void set_position(const irrvec3 new_pos) = 0;
  virtual irrvec3 get_velocity() const = 0;
  virtual void set_velocity(const irrvec3 new_v) = 0;

  // For telemetry.
  struct Telemetry {
    float rps;
    float target_rps;
  };

  virtual Telemetry get_telemetry() const = 0;
  virtual double get_max_rps() const = 0;
};

#endif // __FLYING_OBJ__
