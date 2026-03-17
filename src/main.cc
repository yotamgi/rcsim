/** Simulator main file. */
#include "input_event_reciever.h"
#include "model_configurations.h"
#include "raylib_engine.h"

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

#ifdef PLATFORM_WEB
#include <emscripten.h>
#endif

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

std::shared_ptr<engine::Model> Game::add_banana(const engine::vec3 &pos,
                                                const engine::vec3 &rotation) {
  std::shared_ptr<engine::Model> banana_model =
      m_device.load_model("resources/media/banana/source/banana.obj");
  banana_model->get_materials()[0]->SetTexture(
      0, ::LoadTexture("resources/media/banana/textures/rgb.jpeg"));

  banana_model->set_transform(
      engine::mat4::Scale(1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f) *
      engine::mat4::RotateX(rotation.x) * engine::mat4::RotateY(rotation.y) *
      engine::mat4::RotateZ(rotation.z) *
      engine::mat4::Translate(pos.x, pos.y, pos.z));
  return banana_model;
}

Game::Game()
    : m_device(1440, 900, "rcsim - RC Simulator"), m_input_receiver(),
      m_help_text(m_device.create_text2d("Press 'h' for help", 20,
                                         engine::Color(0, 0, 0, 0xff),
                                         engine::TextAlignment::RIGHT)),
      m_full_help_text_background(m_device.create_square2d(
          {0, 0, 0, 0}, engine::Color(255, 255, 255, 128))),
      m_full_help_text(m_device.create_text2d(FULL_HELP, 20,
                                              engine::Color(0, 0, 0, 0xff),
                                              engine::TextAlignment::CENTER)) {

  // Update Loading texture.
  auto loading_bg =
      m_device.create_square2d({0, 0, (float)m_device.get_screen_width(),
                                (float)m_device.get_screen_height()},
                               engine::Color(230, 230, 230, 255));
  auto loading_headline =
      m_device.create_text2d("Loading...", 40, engine::Color(0, 0, 0, 0xff));
  loading_headline->set_position({10, 10});
  auto loading_text = m_device.create_text2d("\n\nLoading setting...", 30,
                                             engine::Color(0, 0, 0, 0xff),
                                             engine::TextAlignment::LEFT);
  loading_text->set_position({40, 40});
  m_device.draw_frame();

  m_device.create_light(
      engine::LIGHT_DIRECTIONAL, raylib::Vector3(100, 200, -50),
      raylib::Vector3(0, 0, 0), raylib::Color(150, 150, 150, 255));

  // Load the stadium model.
  std::shared_ptr<engine::Model> stadium_model = m_device.load_model(
      "resources/media/BasketballStadium/source/63BasketBallZemin.obj");
  raylib::Material *stadium_material = stadium_model->get_materials()[0];
  m_stadium_texture = raylib::Texture(
      "resources/media/BasketballStadium/textures/BasketZemin_Color.png");
  m_stadium_texture.GenMipmaps();
  m_stadium_texture.SetFilter(TEXTURE_FILTER_TRILINEAR);
  stadium_material->SetTexture(MATERIAL_MAP_DIFFUSE, m_stadium_texture);

  stadium_model->set_transform(engine::mat4::Translate(0, 0, 0) *
                               engine::mat4::Scale(4, 4, 4));

  // Add some bananas for scale.
  m_device.add_shadow_group(
      {add_banana(engine::vec3(-0.5, 0.05, 0.2), engine::vec3(90, 73, 0)),
       add_banana(engine::vec3(0.9, 0.05, 0.3), engine::vec3(90, 40, 0)),
       add_banana(engine::vec3(0.4, 0.05, 0.0), engine::vec3(90, 0, 0))},
      512, 2);

  // Add skybox.
  loading_text->set_text(loading_text->get_text() + "\n\nLoading skies...");
  m_device.draw_frame();

  m_device.add_skybox_from_6_images(
      "resources/media/skybox/px.png", "resources/media/skybox/nx.png",
      "resources/media/skybox/py.png", "resources/media/skybox/ny.png",
      "resources/media/skybox/pz.png", "resources/media/skybox/nz.png");

  m_device.get_camera().SetPosition(raylib::Vector3(0.5f, 1.6f, -5.0f));

  loading_text->set_text(loading_text->get_text() + "\n\nLoading model...");
  m_device.draw_frame();
  m_model_conf = MODEL_CONFIGURATIONS[2].create(&m_device);

  m_device.delete_drawable2d(loading_headline);
  m_device.delete_drawable2d(loading_text);
  m_device.delete_drawable2d(loading_bg);
}

void Game::frame() {
  float time_delta = ::GetFrameTime();
  time_delta = time_delta == 0 ? 1e-3 : time_delta;
  time_delta = time_delta > (1. / 30) ? (1. / 30) : time_delta;

  // Update the model.
  UserInput user_input = m_input_receiver.update_input(time_delta);
  ServoData servo_data = m_model_conf.controls->get_servo_data(
      user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    m_model_conf.model->get_servo(channel).update(servo_data[channel],
                                                  time_delta);
  }
  m_model_conf.model->update(time_delta, engine::vec3(0, 0, 0));

  // Apply external force on the helicopter touch points.
  float model_mass = m_model_conf.model->get_mass();
  m_model_conf.model->reset_force();
  std::vector<FlyingObject::TouchPoint> touchpoints =
      m_model_conf.model->get_touchpoints_in_world();
  for (unsigned int i = 0; i < touchpoints.size(); i++) {
    FlyingObject::TouchPoint tp = touchpoints[i];
    if (tp.pos.y < 0) {
      engine::vec3 tp_force = engine::vec3(0, -500 * tp.pos.y, 0) * model_mass;
      engine::vec3 friction_force;
      friction_force = tp.friction_coeff * tp.vel;
      friction_force.y = 10.0f * tp.vel.y;
      tp_force += -friction_force * (-tp.pos.y / 0.03) * model_mass;
      m_model_conf.model->add_force(i, tp_force);
    }
  }

  // Help texts.
  m_help_text->set_position({(float)m_device.get_screen_width() - 20, 0});
  m_full_help_text->set_position(
      {(float)m_device.get_screen_width() / 2,
       (float)m_device.get_screen_height() / 2 - 100});
  m_full_help_text->set_visible(::IsKeyDown(KEY_H));
  m_full_help_text_background->set_position(
      {0, 0, (float)m_device.get_screen_width(),
       (float)m_device.get_screen_height()});
  m_full_help_text_background->set_visible(::IsKeyDown(KEY_H));

  // Draw.
  m_device.get_camera().SetTarget(m_model_conf.model->get_position());
  m_model_conf.dashboard->update_ui(m_model_conf.controls->get_telemetry(),
                                    m_model_conf.model->get_telemetry());
  m_device.draw_frame();
}

int main() {
  Game game;

#ifdef PLATFORM_WEB
  emscripten_set_main_loop_arg(
      [](void *arg) { static_cast<Game *>(arg)->frame(); }, &game, 0, true);
#else
  while (!WindowShouldClose()) {
    // calculate the delta time, and make sure it does not exceed 100 fps
    game.frame();
  }
#endif

  return 0;
}