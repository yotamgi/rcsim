#include "input_event_reciever.h"

const int NUM_AVAILABLE_CURVES = 2;

UserInputReciever::UserInputReciever() {
  m_joystick = engine::Joystick::get_available();

  m_user_input.controls_input.pitch_stick = 0;
  m_user_input.controls_input.roll_stick = 0;
  m_user_input.controls_input.yaw_stick = 0;
  m_user_input.controls_input.throttle_stick = -1;
  m_user_input.controls_input.active_curve_index = 0;
  m_user_input.controls_input.gyro_6dof = true;
  m_user_input.controls_input.throttle_hold = false;
}

UserInput UserInputReciever::update_input(float time_delta) {
  update_buttons();
  if (m_joystick) {
    update_controls_from_joystick();
  } else {
    update_controls_from_keyboard(time_delta);
  }
  return m_user_input;
}

void UserInputReciever::update_controls_from_joystick() {
  std::vector<float> axes = m_joystick->get_axes();
  m_user_input.controls_input.pitch_stick = -axes[1];
  m_user_input.controls_input.roll_stick = -axes[0];
  m_user_input.controls_input.yaw_stick = axes[5];
  m_user_input.controls_input.throttle_stick = -axes[2];
}

void UserInputReciever::update_controls_from_keyboard(float time_delta) {
  float change_amount = time_delta * 2;
  update_value(m_user_input.controls_input.pitch_stick, KEY_UP, KEY_DOWN,
               change_amount);
  update_value(m_user_input.controls_input.roll_stick, KEY_RIGHT, KEY_LEFT,
               change_amount);
  update_value(m_user_input.controls_input.yaw_stick, KEY_A, KEY_D,
               change_amount);

  if (engine::IsKeyDown(KEY_W))
    m_user_input.controls_input.throttle_stick += time_delta;
  else if (engine::IsKeyDown(KEY_S))
    m_user_input.controls_input.throttle_stick -= time_delta;

  m_user_input.controls_input.throttle_stick =
      m_user_input.controls_input.throttle_stick > 1
          ? 1
          : m_user_input.controls_input.throttle_stick;
  m_user_input.controls_input.throttle_stick =
      m_user_input.controls_input.throttle_stick < -1
          ? -1
          : m_user_input.controls_input.throttle_stick;
}

void UserInputReciever::update_value(float &value, int key_up, int key_down,
                                     float change_amount) {
  if (engine::IsKeyDown(key_up))
    value += change_amount;
  else if (engine::IsKeyDown(key_down))
    value -= change_amount;
  else {
    if (std::abs(value) < 0.1)
      value = 0;
    else {
      value += change_amount * ((float)(value < 0) * 2 - 1);
    }
  }

  value = value > 0.5 ? 0.5 : value;
  value = value < -0.5 ? -0.5 : value;
}

void UserInputReciever::update_buttons() {
  // When 'i' is down, update the current curve index.
  if (engine::IsKeyPressed(KEY_I)) {
    m_user_input.controls_input.active_curve_index =
        (m_user_input.controls_input.active_curve_index + 1) %
        NUM_AVAILABLE_CURVES;
  }
  // When 'g' is down, update the gyro mode.
  if (engine::IsKeyPressed(KEY_G)) {
    m_user_input.controls_input.gyro_6dof =
        !m_user_input.controls_input.gyro_6dof;
  }
  // When 't' is down, update the throttle-hold status.
  if (engine::IsKeyPressed(KEY_T)) {
    m_user_input.controls_input.throttle_hold =
        !m_user_input.controls_input.throttle_hold;
  }
}
