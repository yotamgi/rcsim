#ifndef __FLYING_OBJ__
#define __FLYING_OBJ__

#include <irrlicht/irrlicht.h>
#include <vector>

typedef irr::core::vector3df irrvec3;


/**
 * Servo data, from -1 to 1.
 */
struct ServoData {
    double roll;
    double pitch;
    double yaw;
    double lift;
    double throttle;
};


class FlyingObject {
public:

    virtual void update(double time_delta, const irrvec3 &wind_speed, const ServoData &servo_data) = 0;
    virtual void add_force(unsigned int touchpoints_index, const irrvec3 &force) = 0;
    virtual void reset_force() = 0;

    struct TouchPoint {
        irrvec3 pos_in_world;
        irrvec3 vel_in_world;
    };

    virtual std::vector<TouchPoint> get_touchpoints_in_world() const = 0;
    virtual double get_mass() const = 0;

    // For telemetry.
    struct Telemetry {
        float rps;
        float target_rps;
    };

    virtual Telemetry get_telemetry() const = 0;
    virtual double get_max_rps() const = 0;
};

#endif // __FLYING_OBJ__
