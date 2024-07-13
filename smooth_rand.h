#ifndef __SMOOTH_RAND_H__
#define __SMOOTH_RAND_H__

/**
 * Smooth random float in [0, 1].
 */
class SmoothRandFloat {
public:
    SmoothRandFloat(float change_pace, float change_time);

    void reset();

    float update(float time_delta);
private:

    float m_prev_val;
    float m_target_val;
    float m_change_time;
    float m_change_pace;
    float m_timer;
    
    const float m_char_change_pace;
    const float m_char_change_time;
};

#endif //  __SMOOTH_RAND_H__
