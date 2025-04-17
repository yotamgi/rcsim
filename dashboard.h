#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "flying_object.h"
#include "controls.h"
#include <irrlicht/irrlicht.h>
#include <vector>


class Dashboard {
public:
    Dashboard(
            irr::video::IVideoDriver *driver,
            std::vector<ControllerCurve> throttle_curves,
            std::vector<ControllerCurve> lift_curves,
            float max_rps
    );

    void update_ui(
        const Controls::Telemetry &controls_telemetry,
        const FlyingObject::Telemetry &telemetry
    );

private:
    // Controls images.
    irr::video::ITexture *m_controls_image;
    irr::video::ITexture *m_pin_image;
    irr::video::ITexture *m_second_pin_image;
    irr::video::ITexture *m_yaw_image;
    std::vector<irr::video::ITexture *> m_curves_images;
    irr::video::ITexture *m_curves_vertical_line;

    // Texts.
    irr::video::ITexture *m_throttle_text;
    irr::video::ITexture *m_throttle_hold_text;
    irr::video::ITexture *m_curves_text;
    irr::video::ITexture *m_switch_curves_text;
    irr::video::ITexture *m_controls_and_gyro_view_text;

    // Heli telemetry images.
    irr::video::ITexture *m_main_rotor_indicator_image;
    irr::video::ITexture *m_main_rotor_indicator_hand_image;
    irr::video::ITexture *m_main_rotor_indicator_target_hand_image;
    irr::video::IVideoDriver* m_driver;

    irr::video::ITexture *m_dashboard_background;

    std::vector<ControllerCurve> m_throttle_curves;
    std::vector<ControllerCurve>  m_lift_curves;
    float m_max_rps;
};


#endif  // __DASHBOARD_H__
