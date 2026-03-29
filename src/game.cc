#include "game.h"

using Origin = engine::Rect2D::Origin;

////////////////////////////////////////////////////////////////////////////////
// Loading Screen implementation.
////////////////////////////////////////////////////////////////////////////////

void LoadingScreen::frame(float time_delta) {

  // Update Loading texture.
  auto loading_bg =
      m_game->m_device.create_square2d(engine::Color(230, 230, 230, 255));
  loading_bg->set_position(engine::Rect2D{0, 0, 1.0f, 1.0f});
  auto loading_headline = m_game->m_device.create_text2d(
      "Loading...",
      engine::Text2D::FontOptions{80, engine::Color(0, 0, 0, 0xff)});
  loading_headline->set_position(0.5f, 0.1f, Origin::MID, Origin::MID);
  auto loading_text = m_game->m_device.create_text2d(
      "\n\nLoading setting...\n",
      engine::Text2D::FontOptions{20, engine::Color(0, 0, 0, 0xff)});
  loading_text->set_position(0.5f, 0.2f, Origin::MID, Origin::MIN);
  m_game->m_device.draw_frame();

  m_game->m_device.create_light(
      engine::LIGHT_DIRECTIONAL, raylib::Vector3(100, 200, -50),
      raylib::Vector3(0, 0, 0), raylib::Color(150, 150, 150, 255));

  // Load the stadium model.
  std::shared_ptr<engine::Model> stadium_model = m_game->m_device.load_model(
      "resources/media/BasketballStadium/source/63BasketBallZemin.obj");
  raylib::Material *stadium_material = stadium_model->get_materials()[0];
  m_game->m_stadium_texture = raylib::Texture(
      "resources/media/BasketballStadium/textures/BasketZemin_Color.png");
  m_game->m_stadium_texture.GenMipmaps();
  m_game->m_stadium_texture.SetFilter(TEXTURE_FILTER_TRILINEAR);
  stadium_material->SetTexture(MATERIAL_MAP_DIFFUSE, m_game->m_stadium_texture);

  stadium_model->set_transform(engine::mat4::Translate(0, 0, 0) *
                               engine::mat4::Scale(4, 4, 4));

  // Add some bananas for scale.
  m_game->m_device.add_shadow_group(
      {m_game->add_banana(engine::vec3(-0.5, 0.05, 0.2),
                          engine::vec3(90, 73, 0)),
       m_game->add_banana(engine::vec3(0.9, 0.05, 0.3),
                          engine::vec3(90, 40, 0)),
       m_game->add_banana(engine::vec3(0.4, 0.05, 0.0),
                          engine::vec3(90, 0, 0))},
      512, 2);

  // Add skybox.
  loading_text->set_text(loading_text->get_text() + "Loading skies...\n");
  m_game->m_device.draw_frame();

  m_game->m_device.add_skybox_from_6_images(
      "resources/media/skybox/px.png", "resources/media/skybox/nx.png",
      "resources/media/skybox/py.png", "resources/media/skybox/ny.png",
      "resources/media/skybox/pz.png", "resources/media/skybox/nz.png");

  loading_text->set_text(loading_text->get_text() + "Loading helicopter...\n");
  m_game->m_device.draw_frame();
  Configuration heli_conf = MODEL_CONFIGURATIONS[0].create(&m_game->m_device);
  heli_conf.model->set_visible(false);
  heli_conf.dashboard->set_visible(false);
  m_game->m_model_confs.push_back(heli_conf);
  loading_text->set_text(loading_text->get_text() + "Loading airplane...\n");
  m_game->m_device.draw_frame();
  Configuration plane_conf = MODEL_CONFIGURATIONS[2].create(&m_game->m_device);
  plane_conf.model->set_visible(false);
  plane_conf.dashboard->set_visible(false);
  m_game->m_model_confs.push_back(plane_conf);

  m_game->m_chosen_model = 0;

  m_game->m_device.delete_drawable2d(loading_headline);
  m_game->m_device.delete_drawable2d(loading_text);
  m_game->m_device.delete_drawable2d(loading_bg);

  // Change to next screen - the simulator.
  m_game->m_current_screen = std::make_shared<ModelChooseScreen>(m_game);
}

////////////////////////////////////////////////////////////////////////////////
// Model Choose Screen implementation
////////////////////////////////////////////////////////////////////////////////

ModelChooseScreen::ModelChooseScreen(Game *game) : GameScreen(game) {
  int angles_delta = 360.0f / m_game->m_model_confs.size();
  for (int i = 0; i < m_game->m_model_confs.size(); i++) {
    m_model_base_angles.push_back(i * angles_delta);
    m_game->m_model_confs[i].model->set_visible(true);
  }

  // Set the camera.
  m_game->m_device.get_camera().SetPosition(CAMERA_POS);
  m_game->m_device.get_camera().SetTarget(
      MODEL_WHEEL_POS + engine::vec3(0, 0, 1) * MODEL_WHEEL_RADIUS);
}

void ModelChooseScreen::frame(float time_delta) {
  auto &confs = m_game->m_model_confs;

  // Update the angle
  m_current_angle += 4 * time_delta * (m_target_angle - m_current_angle);

  // Set the models positions.
  for (int i = 0; i < confs.size(); i++) {
    float angle = (m_current_angle + m_model_base_angles[i]) / 180. * PI;
    engine::vec3 pos =
        MODEL_WHEEL_POS + engine::vec3(std::sin(angle) * MODEL_WHEEL_RADIUS, 0,
                                       std::cos(angle) * MODEL_WHEEL_RADIUS);
    confs[i].model->update(time_delta, engine::vec3(0, 0, 0));
    confs[i].model->set_position(pos);
    confs[i].model->set_velocity(engine::vec3(0, 0, 0));
    confs[i].model->set_rotation(engine::mat4::RotateY(PI / 2));
  }

  // Update chosen model.
  if (engine::IsKeyPressed(KEY_SPACE)) {
    m_game->m_chosen_model = (m_game->m_chosen_model + 1) % confs.size();
    m_target_angle = m_model_base_angles[m_game->m_chosen_model];
  }

  // Start game on "Enter"
  if (engine::IsKeyPressed(KEY_ENTER)) {
    m_game->m_current_screen = std::make_shared<SimulatorScreen>(m_game);
  }

  m_game->m_device.draw_frame();
}

////////////////////////////////////////////////////////////////////////////////
// Simulator Screen implementation.
////////////////////////////////////////////////////////////////////////////////

const std::string FULL_HELP =
    "Arrows - Pitch ad roll\n"
    "a / d - Yaw\n"
    "w / s - Throttle\n"
    "g - Toggle 6-dof gyro (helicopter) / Flappron (airplane)\n"
    "t - Throttle hold\n"
    "i - Switch throttle-lift curves\n"
    "h - Toggle this help text\n";

SimulatorScreen::SimulatorScreen(Game *game)
    : GameScreen(game),
      m_help_text(m_game->m_device.create_text2d(
          "Press 'h' for help", engine::Text2D::FontOptions{20})),
      m_full_help_text_background(
          m_game->m_device.create_square2d(engine::Color(0, 0, 0, 50))),
      m_full_help_text(m_game->m_device.create_text2d(
          FULL_HELP, engine::Text2D::FontOptions{20},
          m_full_help_text_background)) {
  for (int i = 0; i < m_game->m_model_confs.size(); i++) {
    Configuration conf = m_game->m_model_confs[i];
    bool visible = (i == m_game->m_chosen_model);
    conf.model->set_visible(visible);
    conf.dashboard->set_visible(visible);
  }
  m_help_text->set_position(1.0f, 20, Origin::MAX, Origin::MIN);
  m_full_help_text_background->set_position(engine::Rect2D{0, 0, 1.0f, 1.0f});
  m_full_help_text->set_position(0.5f, 0.3f, Origin::MID, Origin::MIN);

  // Set the camera
  m_game->m_device.get_camera().SetPosition(raylib::Vector3(0.5f, 1.6f, -5.0f));
}

SimulatorScreen::~SimulatorScreen() {
  m_game->m_device.delete_drawable2d(m_help_text);
  m_game->m_device.delete_drawable2d(m_full_help_text);
  m_game->m_device.delete_drawable2d(m_full_help_text_background);
}

void SimulatorScreen::frame(float time_delta) {
  // Update the model.
  Configuration conf = m_game->m_model_confs[m_game->m_chosen_model];
  UserInput user_input = m_game->m_input_receiver.update_input(time_delta);
  ServoData servo_data =
      conf.controls->get_servo_data(user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    conf.model->get_servo(channel).update(servo_data[channel], time_delta);
  }
  conf.model->update(time_delta, engine::vec3(0, 0, 0));

  // Apply external force on the helicopter touch points.
  float model_mass = conf.model->get_mass();
  conf.model->reset_force();
  std::vector<FlyingObject::TouchPoint> touchpoints =
      conf.model->get_touchpoints_in_world();
  for (unsigned int i = 0; i < touchpoints.size(); i++) {
    FlyingObject::TouchPoint tp = touchpoints[i];
    if (tp.pos.y < 0) {
      engine::vec3 tp_force = engine::vec3(0, -500 * tp.pos.y, 0) * model_mass;
      engine::vec3 friction_force;
      friction_force = tp.friction_coeff * tp.vel;
      friction_force.y = 10.0f * tp.vel.y;
      tp_force += -friction_force * (-tp.pos.y / 0.03) * model_mass;
      conf.model->add_force(i, tp_force);
    }
  }

  // Help texts.
  m_full_help_text->set_visible(::IsKeyDown(KEY_H));
  m_full_help_text_background->set_visible(::IsKeyDown(KEY_H));

  // Draw.
  m_game->m_device.get_camera().SetTarget(conf.model->get_position());
  conf.dashboard->update_ui(conf.controls->get_telemetry(),
                            conf.model->get_telemetry());
  m_game->m_device.draw_frame();
}

////////////////////////////////////////////////////////////////////////////////
// Game implementation.
////////////////////////////////////////////////////////////////////////////////

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

Game::Game() : m_device(1440, 900, "rcsim - RC Simulator"), m_input_receiver() {
  m_current_screen = std::make_shared<LoadingScreen>(this);
}

void Game::frame() {
  float time_delta = ::GetFrameTime();
  time_delta = time_delta == 0 ? 1e-3 : time_delta;
  time_delta = time_delta > (1. / 30) ? (1. / 30) : time_delta;

  m_current_screen->frame(time_delta);
}
