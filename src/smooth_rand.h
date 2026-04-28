#ifndef __SMOOTH_RAND_H__
#define __SMOOTH_RAND_H__
#include "raylib_engine.h"

float rand_float();
engine::vec3 rand_vec3();

/**
 * Templatic smooth random generator.
 */
template <typename T, T(*rand_function)()>
class SmoothRand {
public:
  SmoothRand(float change_pace, float change_time);

  void reset();

  T update(float time_delta);

private:
  T m_prev_val;
  T m_target_val;
  float m_change_time;
  float m_change_pace;
  float m_timer;

  const float m_char_change_pace;
  const float m_char_change_time;
};

////////////////////////////////////////////////////////////////////////////////
// Specific types of smooth random.
////////////////////////////////////////////////////////////////////////////////

typedef SmoothRand<float, rand_float> SmoothRandFloat;
typedef SmoothRand<engine::vec3, rand_vec3> SmoothRandVec3;

////////////////////////////////////////////////////////////////////////////////
// Inline implementation for template.
////////////////////////////////////////////////////////////////////////////////

template <typename T, T (*rand_function)()>
SmoothRand<T, rand_function>::SmoothRand(float change_pace, float change_time)
    : m_char_change_pace(change_pace), m_char_change_time(change_time) {
  m_target_val = rand_function();
  reset();
}

template <typename T, T (*rand_function)()>
void SmoothRand<T, rand_function>::reset() {
  // Randomize the time and pace from the characteristics values.
  m_change_time = m_char_change_time;
  m_change_time *= 0.5 + rand_float();
  m_change_pace = m_char_change_pace;
  m_change_pace *= 0.5 + rand_float();

  m_prev_val = m_target_val;
  m_target_val = rand_function();
  m_timer = 0;
}

template <typename T, T (*rand_function)()> 
T SmoothRand<T, rand_function>::update(float time_delta) {
  m_timer += time_delta;

  if (m_timer > m_change_time) {
    reset();
  }

  float progress = m_timer / m_change_time;
  progress = progress > 1 ? 1 : progress;

  return m_target_val * progress + m_prev_val * (1 - progress);
}
#endif //  __SMOOTH_RAND_H__
