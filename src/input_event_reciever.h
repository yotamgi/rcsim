#ifndef __INTPUT_EVENT_RECIEVER_H__
#define __INTPUT_EVENT_RECIEVER_H__

#include "controls.h"
#include "raylib_engine.h"

struct UserInput {
  ControlsInput controls_input;
};

class UserInputReciever {
public:
  UserInputReciever();

  UserInput update_input(float time_delta);

private:
  void update_value(float &value, int key_up, int key_down,
                    float change_amount);

  void update_buttons();
  void update_controls_from_keyboard(float time_delta);
  void update_controls_from_joystick();

  std::optional<engine::Joystick> m_joystick;
  UserInput m_user_input;
};

#endif //  __INTPUT_EVENT_RECIEVER_H__
