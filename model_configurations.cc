#include "model_configurations.h"
#include "airplane_models.h"
#include "flight_controller.h"

Configuration create_rc_bell_heli(irr::video::IVideoDriver *driver,
                                  irr::scene::ISceneManager *smgr) {

  std::vector<ControllerCurve> throttle_curves = {
      // Normal mode:
      ControllerCurve({
          ControllerCurve::Point(-1, -1),
          ControllerCurve::Point(-0.5, 0.2),
          ControllerCurve::Point(1, 0.2),
      }),
      // Idle-up mode:
      ControllerCurve({
          ControllerCurve::Point(-1, 0.5),
          ControllerCurve::Point(1, 0.5),
      }),
  };

  std::vector<ControllerCurve> lift_curves = {
      // Normal mode:
      ControllerCurve({
          ControllerCurve::Point(-1, -0.1),
          ControllerCurve::Point(-0.5, -0.0),
          ControllerCurve::Point(1, 1.),
      }),
      // Idle-up mode:
      ControllerCurve({
          ControllerCurve::Point(-1, -1),
          ControllerCurve::Point(1, 1),
      }),
  };

  std::shared_ptr<BaseHeli> heli_model =
      std::make_shared<RcBellHeli>(smgr, driver);
  std::shared_ptr<HeliFlightController> flight_controller =
      std::make_shared<HeliFlightController>(heli_model);

  Configuration conf;
  conf.model = heli_model;
  conf.controls = std::make_shared<HeliControls>(flight_controller,
                                                 throttle_curves, lift_curves);
  conf.dashboard = std::make_shared<HeliDashboard>(
      driver, throttle_curves, lift_curves, heli_model->get_max_rps());
  return conf;
}

Configuration create_rc_glider(irr::video::IVideoDriver *driver,
                               irr::scene::ISceneManager *smgr) {
  std::shared_ptr<FlyingObject> airplane =
      std::make_shared<SimpleGlider>(smgr, driver);
  std::shared_ptr<AirplaneControls> controls =
      std::make_shared<AirplaneControls>(true);
  std::shared_ptr<HeliDashboard> dashboard = std::make_shared<HeliDashboard>(
      driver, std::vector<ControllerCurve>(), std::vector<ControllerCurve>(),
      airplane->get_max_rps());
  return Configuration{.model = airplane,
                       .controls = controls,
                       .dashboard = dashboard};
}

Configuration create_rc_trainer(irr::video::IVideoDriver *driver,
                                irr::scene::ISceneManager *smgr) {
  std::shared_ptr<FlyingObject> airplane =
      std::make_shared<Trainer>(smgr, driver);
  std::shared_ptr<AirplaneControls> controls =
      std::make_shared<AirplaneControls>(true);
  std::shared_ptr<HeliDashboard> dashboard = std::make_shared<HeliDashboard>(
      driver, std::vector<ControllerCurve>(), std::vector<ControllerCurve>(),
      airplane->get_max_rps());
  return Configuration{.model = airplane,
                       .controls = controls,
                       .dashboard = dashboard};
}

std::vector<ModelConfiguration> MODEL_CONFIGURATIONS = {
    {.name = std::string("RC Bell Heli"), .create = create_rc_bell_heli},
    {.name = std::string("RC Glider"), .create = create_rc_glider},
    {.name = std::string("RC Trainer"), .create = create_rc_trainer}};
