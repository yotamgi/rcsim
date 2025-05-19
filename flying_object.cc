#include "flying_object.h"

float ServoFilter::update(float value, float time_delta) {
  float step = time_delta * m_max_rps;
  if (step > std::abs(value - m_current_status)) {
    m_current_status = value;
  }

  float delta = value - m_current_status;
  if (delta < 0) {
    m_current_status -= step;
  } else if (delta > 0) {
    m_current_status += step;
  }
  m_current_status = std::max(-1.0f, std::min(m_current_status, 1.0f));

  return m_current_status;
}
