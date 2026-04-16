#include "model_configurations.h"
#include "airplane_models.h"
#include "flight_controller.h"
#include <sstream>

BellHeliConf::BellHeliConf(engine::RaylibDevice *device)
    : Configuration(engine::vec3(0, 0.13 + -0.019, 0), engine::vec3(0, 0, 0),
                    engine::vec3(0, 0, 0)) {

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

  m_model = std::make_shared<RcBellHeli>(device);
  m_flight_controller = std::make_shared<HeliFlightController>(m_model);

  m_controls = std::make_shared<HeliControls>(m_flight_controller,
                                              throttle_curves, lift_curves);
  m_dashboard = std::make_shared<HeliDashboard>(
      device, throttle_curves, lift_curves, m_model->get_max_rps());
}

std::string BellHeliConf::get_name() const { return "RC Bell Helicopter"; }

std::string BellHeliConf::get_summary() const {
  std::stringstream ss;
  ss << "- 450 sized flybarless RC helicopter\n"
     << "- Toggle-able 6dof gyro\n"
     << "- Normal and idle-up throttle/lift curves\n"
     << "- Mass: " << m_model->get_mass() << " kg\n"
     << "- Rotor max RPS: " << m_model->get_max_rps();
  return ss.str();
}

void BellHeliConf::reset_for_animation() {
  m_model->get_servo(HELI_CHANNEL_THROTTLE).reset(0.3);
  m_model->set_rotor_rps(10.);
  m_model->set_rotation(engine::mat4::RotateY(PI / 2));
  m_flight_controller->reset(engine::vec3(0, 0, 0));
}

void BellHeliConf::animate(float time_delta, UserInput user_input) {
  user_input.controls_input.throttle_stick = 0.3;
  user_input.controls_input.yaw_stick += 0.03;
  ServoData servo_data =
      m_controls->get_servo_data(user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    m_model->get_servo(channel).update(servo_data[channel], time_delta);
  }
  m_model->update(time_delta, engine::vec3(0, 0, 0));
  m_model->set_velocity(engine::vec3(0, 0, 0));
}

void BellHeliConf::reset_for_simulation() {
  m_model->get_servo(HELI_CHANNEL_THROTTLE).reset(-0.9);
  m_model->set_rotor_rps(1.);
  m_model->set_rotation(engine::mat4::Identity());
  m_flight_controller->reset(engine::vec3(0, 0, 0));
}

CessnaConf::CessnaConf(engine::RaylibDevice *device)
    : Configuration(engine::vec3(0, 0.3, 0), engine::vec3(0, 0, 0),
                    engine::vec3(0, 0, 0)) {
  m_model = std::make_shared<Trainer>(device);
  m_controls = std::make_shared<AirplaneControls>(true);
  m_dashboard = std::make_shared<PlaneDashboard>(device, 30);
}

std::string CessnaConf::get_name() const { return "Cessna 172 Trainer"; }

std::string CessnaConf::get_summary() const {
  std::stringstream ss;
  ss << "- Cessna 172 trainer model\n"
     << "- 4 control channels\n"
     << "- Flappron supported.\n"
     << "- low (1 degree) dihedral angle\n"
     << "- [-10, 15] degrees stall angle\n"
     << "- Mass: " << m_model->get_mass() << " kg\n";
  return ss.str();
}

void CessnaConf::reset_for_animation() {
  m_model->set_rotation(engine::mat4::RotateY(PI / 2));
}

void CessnaConf::reset_for_simulation() {
  m_model->set_rotation(engine::mat4::Identity());
  m_model->get_servo(AIRPLANE_CHANNEL_THROTTLE).reset(-0.9);
}

void CessnaConf::animate(float time_delta, UserInput user_input) {
  user_input.controls_input.throttle_stick = 0.7;
  ServoData servo_data =
      m_controls->get_servo_data(user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    m_model->get_servo(channel).update(servo_data[channel], time_delta);
  }
  m_model->update(time_delta, engine::vec3(-15, 0, 0));
  m_model->set_velocity(engine::vec3(0, 0, 0));
}

GliderConf::GliderConf(engine::RaylibDevice *device)
    : Configuration(engine::vec3(1, 10, -1), engine::vec3(0, 0, 10),
                    engine::vec3(0, 0, 0)) {
  m_model = std::make_shared<SimpleGlider>(device);
  m_controls = std::make_shared<AirplaneControls>(true);
  m_dashboard = std::make_shared<PlaneDashboard>(device, 20);
}

std::string GliderConf::get_name() const { return "Simple Glider"; }

std::string GliderConf::get_summary() const {
  std::stringstream ss;
  ss << "A simple glider model, shouldn't be visibile for now.\n"
     << "Mass: " << m_model->get_mass() << " kg\n";
  return ss.str();
}

void GliderConf::reset_for_animation() {
  m_model->set_rotation(engine::mat4::RotateY(PI / 2));
}

void GliderConf::reset_for_simulation() {
  m_model->set_rotation(engine::mat4::Identity());
}

void GliderConf::animate(float time_delta, UserInput user_input) {
  user_input.controls_input.throttle_stick = 0.7;
  ServoData servo_data =
      m_controls->get_servo_data(user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    m_model->get_servo(channel).update(servo_data[channel], time_delta);
  }
  m_model->update(time_delta, engine::vec3(-15, 0, 0));
  m_model->set_velocity(engine::vec3(0, 0, 0));
}
