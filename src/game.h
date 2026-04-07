#ifndef __GAME_H__
#define __GAME_H__

#include "input_event_reciever.h"
#include "model_configurations.h"
#include "raylib_engine.h"

class Game;
class GameScreen;

class GameScreen {
public:
  GameScreen(Game *game) : m_game(game) {}
  virtual void frame(float time_delta) = 0;

protected:
  Game *m_game;
  friend class Game;
};

class LoadingScreen : public GameScreen {
  /** Loads the global game state and presents a loading screen while doing so.
   */
public:
  LoadingScreen(Game *game) : GameScreen(game) {}
  void frame(float time_delta);
};

class ModelChooseScreen : public GameScreen {
  /** Loads the global game state and presents a loading screen while doing so.
   */
public:
  ModelChooseScreen(Game *game);
  void frame(float time_delta);

protected:
  float m_current_angle = 0;
  float m_target_angle = 0;
  std::vector<float> m_model_base_angles;

  const engine::vec3 MODEL_WHEEL_POS{0, 1, -2};
  const engine::vec3 CAMERA_POS{0, 2, -2};
  const engine::Color LIGHTBULB_COLOR{150, 150, 150, 255};
  const float MODEL_WHEEL_RADIUS = 3.0f;
};

class TransitionToSimulatorScreen : public GameScreen {
  /** A transition screen between the model choose screen and the simulator
   * screen. */
public:
  TransitionToSimulatorScreen(Game *game);
  void frame(float time_delta);

protected:
  float get_alpha() const;
  engine::vec3 m_model_position_from;
  engine::vec3 m_model_position_to;
  engine::vec3 m_model_rotation_from;
  engine::vec3 m_model_rotation_to;
  engine::vec3 m_camera_position_from;
  engine::vec3 m_camera_position_to;
  engine::Color m_ambient_color_from;
  engine::Color m_ambient_color_to;
  engine::Color m_sun_light_color_from;
  engine::Color m_sun_light_color_to;
  engine::Color m_lightbulb_color_from;
  engine::Color m_lightbulb_color_to;

  float m_timer;
  float m_transition_time = 0.5f;
};

class SimulatorScreen : public GameScreen {
  /** The main screen for the simulator. */
public:
  SimulatorScreen(Game *game);
  ~SimulatorScreen();
  void frame(float time_delta);

  static const engine::vec3 CAMERA_POSITION;
  static const engine::Color AMBIENT_LIGHT_COLOR;
  static const engine::Color SUN_LIGHT_COLOR;
  static const engine::Color LIGHTBULB_COLOR;

protected:
  std::shared_ptr<engine::Text2D> m_help_text;
  std::shared_ptr<engine::Square2D> m_full_help_text_background;
  std::shared_ptr<engine::Text2D> m_full_help_text;
};

class Game {
public:
  Game();

  void frame();

protected:
  std::shared_ptr<engine::Model> add_banana(const engine::vec3 &pos,
                                            const engine::vec3 &rotation);

  // Global Game state.
  engine::RaylibDevice m_device;
  UserInputReciever m_input_receiver;
  std::vector<std::shared_ptr<Configuration>> m_model_confs;
  raylib::Texture m_stadium_texture;
  size_t m_chosen_model;
  std::shared_ptr<engine::Light> m_sun_light;
  std::shared_ptr<engine::Light> m_light_bulb;

  std::shared_ptr<GameScreen> m_current_screen;
  friend class LoadingScreen;
  friend class ModelChooseScreen;
  friend class TransitionToSimulatorScreen;
  friend class SimulatorScreen;
};

#endif // __GAME_H__