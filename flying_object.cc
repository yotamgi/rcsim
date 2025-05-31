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

void update_rotation_matrix(irr::core::matrix4 &matrix,
                            const irrvec3 angularv) {

  // Extract rotation columns.
  irrvec3 x(matrix(0, 0), matrix(0, 1), matrix(0, 2));
  irrvec3 y(matrix(1, 0), matrix(1, 1), matrix(1, 2));
  irrvec3 z(matrix(2, 0), matrix(2, 1), matrix(2, 2));

  // Perform the inifinitisimal rotation.
  x += angularv.crossProduct(x);
  y += angularv.crossProduct(y);
  z += angularv.crossProduct(z);

  // Make sure it orthogonal.
  z = x.crossProduct(y);
  x = y.crossProduct(z);
  y = z.crossProduct(x);
  x.normalize();
  y.normalize();
  z.normalize();

  // Write columns back to the matrix.
  matrix(0, 0) = x.X;
  matrix(0, 1) = x.Y;
  matrix(0, 2) = x.Z;
  matrix(1, 0) = y.X;
  matrix(1, 1) = y.Y;
  matrix(1, 2) = y.Z;
  matrix(2, 0) = z.X;
  matrix(2, 1) = z.Y;
  matrix(2, 2) = z.Z;
}

