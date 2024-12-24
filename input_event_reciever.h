#ifndef __INTPUT_EVENT_RECIEVER_H__
#define __INTPUT_EVENT_RECIEVER_H__

#include "controls.h"
#include <irrlicht/irrlicht.h>

struct UserInput {
    ControlsInput controls_input;
};

/**
 * To receive events like mouse and keyboard input, or GUI events like "the OK
 * button has been clicked", we need an object which is derived from the
 * irr::IEventReceiver object. There is only one method to override:
 * irr::IEventReceiver::OnEvent(). This method will be called by the engine once
 * when an event happens. What we really want to know is whether a key is being
 * held down, and so we will remember the current state of each key.
 */
class EventReceiver : public irr::IEventReceiver
{
public:
    EventReceiver();

    // This is the one method that we have to implement
    virtual bool OnEvent(const irr::SEvent& event);

    // This is used to check whether a key is being held down
    virtual bool IsKeyDown(irr::EKEY_CODE keyCode) const;

    UserInput update_input(float time_delta);

private:

    void update_value(float &value, irr::EKEY_CODE key_up, irr::EKEY_CODE key_down, float change_amount);

    // We use this array to store the current state of each key
    bool KeyIsDown[irr::KEY_KEY_CODES_COUNT];

    irr::SEvent::SJoystickEvent JoystickState;
    bool m_joystick_active;
    UserInput m_user_input;
};

#endif //  __INTPUT_EVENT_RECIEVER_H__
