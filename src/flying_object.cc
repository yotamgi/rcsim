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

void update_rotation_matrix(engine::mat4 &m, const engine::vec3 angularv) {

  // Extract rotation columns.
  engine::vec3 x(engine::mat_get(m, 0, 0), engine::mat_get(m, 0, 1),
            engine::mat_get(m, 0, 2));
  engine::vec3 y(engine::mat_get(m, 1, 0), engine::mat_get(m, 1, 1),
            engine::mat_get(m, 1, 2));
  engine::vec3 z(engine::mat_get(m, 2, 0), engine::mat_get(m, 2, 1),
            engine::mat_get(m, 2, 2));

  // Perform the inifinitisimal rotation.
  x += angularv.CrossProduct(x);
  y += angularv.CrossProduct(y);
  z += angularv.CrossProduct(z);

  // Make sure it orthogonal.
  z = x.CrossProduct(y);
  x = y.CrossProduct(z);
  y = z.CrossProduct(x);
  x = x.Normalize();
  y = y.Normalize();
  z = z.Normalize();

  // Write columns back to the matrix.
  engine::mat_get(m, 0, 0) = x.x;
  engine::mat_get(m, 0, 1) = x.y;
  engine::mat_get(m, 0, 2) = x.z;
  engine::mat_get(m, 1, 0) = y.x;
  engine::mat_get(m, 1, 1) = y.y;
  engine::mat_get(m, 1, 2) = y.z;
  engine::mat_get(m, 2, 0) = z.x;
  engine::mat_get(m, 2, 1) = z.y;
  engine::mat_get(m, 2, 2) = z.z;
}
