#include "input_event_reciever.h"

using namespace irr;
const int NUM_AVAILABLE_CURVES = 2;

EventReceiver::EventReceiver()
{
	for (u32 i=0; i<irr::KEY_KEY_CODES_COUNT; ++i)
		KeyIsDown[i] = false;
    m_joystick_active = false;
    m_user_input.controls_input.pitch_stick = 0;
    m_user_input.controls_input.roll_stick = 0;
    m_user_input.controls_input.yaw_stick = 0;
    m_user_input.controls_input.throttle_stick = -1;
    m_user_input.controls_input.active_curve_index = 0;
    m_user_input.controls_input.gyro_6dof = true;
}

bool EventReceiver::OnEvent(const irr::SEvent& event) {
	// Remember whether each key is down or up
	if (event.EventType == irr::EET_KEY_INPUT_EVENT) {
		KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

        // When 'i' is down, update the current curve index.
        if (event.KeyInput.Key == KEY_KEY_I && event.KeyInput.PressedDown) {
            m_user_input.controls_input.active_curve_index = 
                (m_user_input.controls_input.active_curve_index + 1) % NUM_AVAILABLE_CURVES;
        }
        // When 'g' is down, update the gyro mode.
        if (event.KeyInput.Key == KEY_KEY_G && event.KeyInput.PressedDown) {
            m_user_input.controls_input.gyro_6dof = !m_user_input.controls_input.gyro_6dof;
        }
    }


    // The state of each connected joystick is sent to us
    // once every run() of the Irrlicht device.  Store the
    // state of the first joystick, ignoring other joysticks.
    // This is currently only supported on Windows and Linux.
    if (event.EventType == irr::EET_JOYSTICK_INPUT_EVENT
        && event.JoystickEvent.Joystick == 0)
    {
        JoystickState = event.JoystickEvent;
        m_joystick_active = true;
    }

	return false;
}

void EventReceiver::update_value(float &value, irr::EKEY_CODE key_up, irr::EKEY_CODE key_down, float change_amount) {
    if (IsKeyDown(key_up)) value += change_amount;
    else if (IsKeyDown(key_down)) value -= change_amount;
    else {
        if (std::abs(value) < 0.1) value = 0;
        else {
            value += change_amount * ((float)(value < 0)*2 - 1);
        }
    }

    value = value > 1 ? 1 : value;
    value = value < -1 ? -1 : value;
}

UserInput EventReceiver::update_input(float time_delta) {
    if (m_joystick_active) {
        m_user_input.controls_input.pitch_stick = -(float)JoystickState.Axis[1] / 32768;
        m_user_input.controls_input.roll_stick = -(float)JoystickState.Axis[0] / 32768;
        m_user_input.controls_input.yaw_stick = (float)JoystickState.Axis[4] / 32768;
        m_user_input.controls_input.throttle_stick = -(float)JoystickState.Axis[2] / 32768;
        return m_user_input;
    }

    float change_amount = time_delta * 4;
    update_value(m_user_input.controls_input.pitch_stick, KEY_UP, KEY_DOWN, change_amount);
    update_value(m_user_input.controls_input.roll_stick, KEY_LEFT, KEY_RIGHT, change_amount);
    update_value(m_user_input.controls_input.yaw_stick, KEY_KEY_D, KEY_KEY_A, change_amount);

    if (IsKeyDown(KEY_KEY_W))  m_user_input.controls_input.throttle_stick += time_delta;
    else if (IsKeyDown(KEY_KEY_S)) m_user_input.controls_input.throttle_stick -= time_delta;

    m_user_input.controls_input.throttle_stick = m_user_input.controls_input.throttle_stick > 1 ? 
            1 : m_user_input.controls_input.throttle_stick;
    m_user_input.controls_input.throttle_stick = m_user_input.controls_input.throttle_stick < -1 ?
            -1 : m_user_input.controls_input.throttle_stick;
    return m_user_input;
}

bool EventReceiver::IsKeyDown(irr::EKEY_CODE keyCode) const {
	return KeyIsDown[keyCode];
}



