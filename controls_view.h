#ifndef __CONTROLS_VIEW_H__
#define __CONTROLS_VIEW_H__


#include <irrlicht/irrlicht.h>
#include "heli.h"


class ControlsView {
public:
    ControlsView(irr::video::IVideoDriver *driver);

    void update_ui(const ServoData &before_controller, const ServoData &after_controller);
private:
    irr::video::ITexture *m_controls_image;
    irr::video::ITexture *m_pin_image;
    irr::video::ITexture *m_second_pin_image;
    irr::video::ITexture *m_yaw_image;
    irr::video::ITexture *m_lift_image;
    irr::video::IVideoDriver* m_driver;
};

#endif // __CONTROLS_VIEW_H__
