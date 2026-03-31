#ifndef __MODEL_CONFIGURATION_H__
#define __MODEL_CONFIGURATION_H__

#include "controls.h"
#include "dashboard.h"
#include "flying_object.h"
#include "input_event_reciever.h"
#include "raylib_engine.h"
#include <memory>

class Configuration {
public:
  Configuration(engine::vec3 init_position, engine::vec3 init_velocity,
                engine::vec3 init_rotation)
      : INIT_POSITION(init_position), INIT_VELOCITY(init_velocity),
        INIT_ROTATION(init_rotation) {}
  virtual std::shared_ptr<FlyingObject> model() = 0;
  virtual std::shared_ptr<Controls> controls() = 0;
  virtual std::shared_ptr<Dashboard> dashboard() = 0;

  virtual void reset_for_animation() = 0;
  virtual void animate(float time_delta, UserInput user_input) = 0;

  virtual void reset_for_simulation() = 0;

  const engine::vec3 INIT_POSITION;
  const engine::vec3 INIT_VELOCITY;
  const engine::vec3 INIT_ROTATION;
};

class BellHeliConf : public Configuration {
public:
  BellHeliConf(engine::RaylibDevice *device);

  virtual std::shared_ptr<FlyingObject> model() { return m_model; }
  virtual std::shared_ptr<Controls> controls() { return m_controls; }
  virtual std::shared_ptr<Dashboard> dashboard() { return m_dashboard; }

  virtual void reset_for_animation();
  virtual void animate(float time_delta, UserInput user_input);
  virtual void reset_for_simulation();

protected:
  std::shared_ptr<BaseHeli> m_model;
  std::shared_ptr<HeliControls> m_controls;
  std::shared_ptr<HeliDashboard> m_dashboard;
  std::shared_ptr<HeliFlightController> m_flight_controller;
};

class CessnaConf : public Configuration {
public:
  CessnaConf(engine::RaylibDevice *device);

  virtual std::shared_ptr<FlyingObject> model() { return m_model; }
  virtual std::shared_ptr<Controls> controls() { return m_controls; }
  virtual std::shared_ptr<Dashboard> dashboard() { return m_dashboard; }

  virtual void reset_for_animation();
  virtual void animate(float time_delta, UserInput user_input);
  virtual void reset_for_simulation();

protected:
  std::shared_ptr<Airplane> m_model;
  std::shared_ptr<AirplaneControls> m_controls;
  std::shared_ptr<PlaneDashboard> m_dashboard;
};

class GliderConf : public Configuration {
public:
  GliderConf(engine::RaylibDevice *device);

  virtual std::shared_ptr<FlyingObject> model() { return m_model; }
  virtual std::shared_ptr<Controls> controls() { return m_controls; }
  virtual std::shared_ptr<Dashboard> dashboard() { return m_dashboard; }

  virtual void reset_for_animation();
  virtual void animate(float time_delta, UserInput user_input);
  virtual void reset_for_simulation();

protected:
  std::shared_ptr<Airplane> m_model;
  std::shared_ptr<AirplaneControls> m_controls;
  std::shared_ptr<PlaneDashboard> m_dashboard;
};

#endif // __MODEL_CONFIGURATION_H__
