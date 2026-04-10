#ifndef __INTPUT_EVENT_RECIEVER_H__
#define __INTPUT_EVENT_RECIEVER_H__

#include "controls.h"
#include "raylib_engine.h"
#include <map>

struct UserInput {
  ControlsInput controls_input;
};

class UserInputReciever {
public:
  UserInputReciever();

  UserInput update_input(float time_delta);

  struct Config {
    std::string active_joystick_name; // Empty string indicates keyboard.
    struct Channel {
      int joystick_channel;
      bool reverse;
    };
    Channel pitch_channel;
    Channel roll_channel;
    Channel yaw_channel;
    Channel throttle_channel;
  };

  Config &get_config() { return m_config; }

private:
  void update_value(float &value, int key_up, int key_down,
                    float change_amount);

  void update_buttons();
  void update_controls_from_keyboard(float time_delta);
  void update_controls_from_joystick();

  std::map<std::string, engine::Joystick> m_joysticks;
  UserInput m_user_input;
  Config m_config = {
      .active_joystick_name = "",
      .pitch_channel = {1, true},
      .roll_channel = {0, true},
      .yaw_channel = {5, false},
      .throttle_channel = {2, true},
  };
};

#endif //  __INTPUT_EVENT_RECIEVER_H__
