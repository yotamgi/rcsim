#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "heli.h"
#include "controls.h"
#include <irrlicht/irrlicht.h>
#include <vector>


class Dashboard {
public:
    Dashboard(
            irr::video::IVideoDriver *driver,
            std::vector<ControllerCurve> throttle_curves,
            std::vector<ControllerCurve> lift_curves,
            float main_rotor_max_rps
    );

    void update_ui(
        const Controls::Telemetry &controls_telemetry,
        const BaseHeli::Telemetry &heli_telemetry
    );

private:
    // Controls images.
    irr::video::ITexture *m_controls_image;
    irr::video::ITexture *m_pin_image;
    irr::video::ITexture *m_second_pin_image;
    irr::video::ITexture *m_yaw_image;
    std::vector<irr::video::ITexture *> m_curves_images;
    irr::video::ITexture *m_curves_vertical_line;

    // Heli telemetry images.
    irr::video::ITexture *m_main_rotor_indicator_image;
    irr::video::ITexture *m_main_rotor_indicator_hand_image;
    irr::video::ITexture *m_main_rotor_indicator_target_hand_image;
    irr::video::IVideoDriver* m_driver;

    irr::video::ITexture *m_dashboard_background;

    std::vector<ControllerCurve> m_throttle_curves;
    std::vector<ControllerCurve>  m_lift_curves;
    float m_main_rotor_max_rps;
};


#endif  // __DASHBOARD_H__
