#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "heli.h"
#include <memory>

class HeliFlightController {
public:
  HeliFlightController(std::shared_ptr<const BaseHeli> heli)
      : m_heli(heli), m_six_axis(true) {}
  std::vector<float> translate(const std::vector<float> &servo_data, float time_delta);

  void set_six_axis(bool set) { m_six_axis = set; }

private:
  std::shared_ptr<const BaseHeli> m_heli;
  irr::core::vector3df m_heli_angles;
  irr::core::vector3df m_wanted_angles;
  irr::core::vector3df m_prev_error;
  irr::core::vector3df m_error_integral;

  bool m_six_axis;
};

#endif // __CONTROLLER_H__
