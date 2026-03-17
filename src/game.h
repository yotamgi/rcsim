#ifndef __GAME_H__
#define __GAME_H__

#include "raylib_engine.h"
#include "model_configurations.h"
#include "input_event_reciever.h"

const std::string FULL_HELP =
    "Arrows - Pitch ad roll\n"
    "a / d - Yaw\n"
    "w / s - Throttle\n"
    "g - Toggle 6dof gyro (helicopter) / Flappron (airplane)\n"
    "t - Throttle hold\n"
    "i - Switch throttle/lift curves\n"
    "h - Toggle this help text\n";

class Game {
public:
  Game();

  void frame();

private:
  std::shared_ptr<engine::Model> add_banana(const engine::vec3 &pos,
                                            const engine::vec3 &rotation);

  engine::RaylibDevice m_device;
  UserInputReciever m_input_receiver;
  Configuration m_model_conf;
  raylib::Texture m_stadium_texture;
  std::shared_ptr<engine::Text2D> m_help_text;
  std::shared_ptr<engine::Square2D> m_full_help_text_background;
  std::shared_ptr<engine::Text2D> m_full_help_text;
};

#endif // __GAME_H__