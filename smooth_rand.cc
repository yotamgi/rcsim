
#include "smooth_rand.h"
#include <cstdlib>


float rand_float() {
    return (float)std::rand() / (float)RAND_MAX;
}


SmoothRandFloat::SmoothRandFloat(float change_pace, float change_time)
        :m_char_change_pace(change_pace), m_char_change_time(change_time)
{
    m_target_val = 0;
    reset();
}

void SmoothRandFloat::reset() {
    // Randomize the time and pace from the characteristics values.
    m_change_time = m_char_change_time;
    m_change_time *= 0.5 + rand_float();
    m_change_pace = m_char_change_pace;
    m_change_pace *= 0.5 + rand_float();

    m_prev_val = m_target_val;
    m_target_val = rand_float();
    m_timer = 0;
}

float SmoothRandFloat::update(float time_delta) {
    m_timer += time_delta;

    if (m_timer > m_change_time) {
        reset();
    }

    float progress = m_timer / m_change_time;
    progress = progress > 1 ? 1 : progress;

    return m_target_val * progress + m_prev_val * (1 - progress);
}

