#include "game.h"
#include <algorithm>

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

  m_game->m_sun_light = m_game->m_device.create_light(
      engine::LIGHT_DIRECTIONAL, raylib::Vector3(100, 200, -50),
      raylib::Vector3(0, 0, 0), raylib::Color(130, 130, 130, 255));

  m_game->m_light_bulb = m_game->m_device.create_light(
      engine::LIGHT_POINT, raylib::Vector3(0.0, 1.5, 0.8),
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
  std::shared_ptr<Configuration> heli_conf =
      std::make_shared<BellHeliConf>(&m_game->m_device);
  heli_conf->model()->set_visible(false);
  heli_conf->dashboard()->set_visible(false);
  m_game->m_model_confs.push_back(heli_conf);
  loading_text->set_text(loading_text->get_text() + "Loading airplane...\n");
  m_game->m_device.draw_frame();
  std::shared_ptr<Configuration> plane_conf =
      std::make_shared<CessnaConf>(&m_game->m_device);
  plane_conf->model()->set_visible(false);
  plane_conf->dashboard()->set_visible(false);
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

ModelChooseScreen::ModelChooseScreen(Game *game)
    : GameScreen(game), m_text_background(m_game->m_device.create_square2d(
                            engine::Color(255, 255, 255, 128))),
      m_separator_line(m_game->m_device.create_square2d(
          engine::Color(0, 0, 0, 255), m_text_background)),
      m_model_name_text(m_game->m_device.create_text2d(
          "", engine::Text2D::FontOptions{40, engine::Color(0, 0, 0, 255)},
          m_text_background)),
      m_model_summary_text(m_game->m_device.create_text2d(
          "", engine::Text2D::FontOptions{20, engine::Color(0, 0, 0, 255), 1.5},
          m_text_background)),
      m_help_text(m_game->m_device.create_text2d(
          "Press 'Space' for next model, 'Enter' to start",
          engine::Text2D::FontOptions{30, engine::Color(0, 0, 0, 255)})) {
  int angles_delta = 360.0f / m_game->m_model_confs.size();
  for (int i = 0; i < m_game->m_model_confs.size(); i++) {
    m_model_base_angles.push_back(i * angles_delta);
    m_game->m_model_confs[i]->model()->set_visible(true);
  }

  // Set the camera.
  m_game->m_device.get_camera().SetPosition(CAMERA_POS);
  m_game->m_device.get_camera().SetTarget(
      MODEL_WHEEL_POS + engine::vec3(0, 0, 1) * MODEL_WHEEL_RADIUS +
      engine::vec3(0.5, 0, 0));

  // Rotate the models to look right.
  auto &confs = m_game->m_model_confs;
  for (int i = 0; i < confs.size(); i++) {
    confs[i]->reset_for_animation();
  }

  // Set the lighting.
  m_game->m_device.set_ambient_light(engine::Color(5, 5, 5, 255));
  m_game->m_sun_light->set_enabled(true);
  m_game->m_sun_light->set_color(engine::Color(5, 5, 5, 255));
  m_game->m_light_bulb->set_color(LIGHTBULB_COLOR);
  m_game->m_light_bulb->set_enabled(true);

  // The texts.
  m_text_background->set_position(engine::Rect2D{0, 0.1f, 0.35f, 0.8f});
  m_model_name_text->set_position(0.5f, 0.05f, Origin::MID, Origin::MIN);
  m_separator_line->set_position(engine::Rect2D{0.1f, 0.15f, 0.8f, 3});
  m_model_summary_text->set_position(0.05f, 0.5f, Origin::MIN, Origin::MID);
  m_help_text->set_position(0.5f, 0.98f, Origin::MID, Origin::MAX);
  update_model_texts();
}

void ModelChooseScreen::update_model_texts() {
  auto &conf = m_game->m_model_confs[m_game->m_chosen_model];
  m_model_name_text->set_text(conf->get_name());
  m_model_summary_text->set_text(conf->get_summary());
}

void ModelChooseScreen::frame(float time_delta) {
  auto &confs = m_game->m_model_confs;

  // Update the models wheel angle.
  m_current_angle += 4 * time_delta * (m_target_angle - m_current_angle);

  UserInput user_input = m_game->m_input_receiver.update_input(time_delta);

  // Set the models positions.
  for (int i = 0; i < confs.size(); i++) {
    float angle_deg = m_current_angle + m_model_base_angles[i];
    float angle = angle_deg / 180. * PI;
    engine::vec3 pos =
        MODEL_WHEEL_POS + engine::vec3(std::sin(angle) * MODEL_WHEEL_RADIUS, 0,
                                       std::cos(angle) * MODEL_WHEEL_RADIUS);
    confs[i]->model()->set_position(pos);
    confs[i]->animate(time_delta, user_input);
    if (pos.z < 0) {
      confs[i]->reset_for_animation();
    }
  }

  // Update chosen model.
  if (engine::IsKeyPressed(KEY_SPACE)) {
    m_game->m_chosen_model = (m_game->m_chosen_model + 1) % confs.size();
    m_target_angle = m_model_base_angles[m_game->m_chosen_model];
    update_model_texts();
  }

  // Start game on "Enter"
  if (engine::IsKeyPressed(KEY_ENTER)) {
    m_game->m_current_screen =
        std::make_shared<TransitionToSimulatorScreen>(m_game);

    // For some reason, deleting the texts causes a crash. Hiding them instead.
    m_game->m_device.delete_drawable2d(m_model_name_text);
    m_game->m_device.delete_drawable2d(m_model_summary_text);
    m_game->m_device.delete_drawable2d(m_separator_line);
    m_game->m_device.delete_drawable2d(m_help_text);
    m_game->m_device.delete_drawable2d(m_text_background);
  }

  m_game->m_device.draw_frame();
}

////////////////////////////////////////////////////////////////////////////////
// Transition To Simulator Screen implementation
////////////////////////////////////////////////////////////////////////////////

TransitionToSimulatorScreen::TransitionToSimulatorScreen(Game *game)
    : GameScreen(game),
      m_model_position_from(
          game->m_model_confs[game->m_chosen_model]->model()->get_position()),
      m_model_position_to(
          game->m_model_confs[game->m_chosen_model]->INIT_POSITION),
      m_model_rotation_from(engine::mat_to_angles_zyx(
          game->m_model_confs[game->m_chosen_model]->model()->get_rotation())),
      m_model_rotation_to(engine::mat_to_angles_zyx(engine::mat4::RotateXYZ(
          game->m_model_confs[game->m_chosen_model]->INIT_ROTATION))),
      m_camera_position_from(game->m_device.get_camera().GetPosition()),
      m_camera_position_to(SimulatorScreen::CAMERA_POSITION),
      m_ambient_color_from(game->m_device.get_ambient_light()),
      m_ambient_color_to(SimulatorScreen::AMBIENT_LIGHT_COLOR),
      m_sun_light_color_from(game->m_sun_light->get_color()),
      m_sun_light_color_to(SimulatorScreen::SUN_LIGHT_COLOR),
      m_lightbulb_color_from(game->m_light_bulb->get_color()),
      m_lightbulb_color_to(SimulatorScreen::LIGHTBULB_COLOR), m_timer(0) {}

float TransitionToSimulatorScreen::get_alpha() const {
  float linear_alpha = m_timer / m_transition_time;
  return std::sin((linear_alpha - 0.5) * PI) / 2 +
         0.5; // Ease in-out with sine.
}

void TransitionToSimulatorScreen::frame(float time_delta) {
  m_timer += time_delta;
  float alpha = get_alpha();

  if (m_timer >= m_transition_time) {
    m_game->m_current_screen = std::make_shared<SimulatorScreen>(m_game);
    return;
  }

  // Model and Camera.
  engine::vec3 model_pos =
      m_model_position_from * (1 - alpha) + m_model_position_to * alpha;
  engine::vec3 camera_pos =
      m_camera_position_from * (1 - alpha) + m_camera_position_to * alpha;
  m_game->m_device.get_camera().SetPosition(camera_pos);
  m_game->m_device.get_camera().SetTarget(model_pos);
  auto &conf = m_game->m_model_confs[m_game->m_chosen_model];
  conf->model()->set_position(model_pos);
  conf->model()->set_rotation(engine::angles_zyx_to_mat(
      m_model_rotation_from * (1 - alpha) + m_model_rotation_to * alpha));
  conf->model()->update(time_delta, engine::vec3(0, 0, 0));

  // Lighting.
  engine::Color ambient_color =
      m_ambient_color_from * (1 - alpha) + m_ambient_color_to * alpha;
  m_game->m_device.set_ambient_light(ambient_color);
  engine::Color sun_light_color =
      m_sun_light_color_from * (1 - alpha) + m_sun_light_color_to * alpha;
  m_game->m_sun_light->set_color(sun_light_color);
  engine::Color lightbulb_color =
      m_lightbulb_color_from * (1 - alpha) + m_lightbulb_color_to * alpha;
  m_game->m_light_bulb->set_color(lightbulb_color);
  m_game->m_device.draw_frame();
}

////////////////////////////////////////////////////////////////////////////////
// Simulator Screen implementation.
////////////////////////////////////////////////////////////////////////////////

const engine::vec3 SimulatorScreen::CAMERA_POSITION =
    engine::vec3(0.5f, 1.6f, -5.0f);

const engine::Color SimulatorScreen::AMBIENT_LIGHT_COLOR =
    engine::Color(75, 75, 75, 255);

const engine::Color SimulatorScreen::SUN_LIGHT_COLOR =
    engine::Color(150, 150, 150, 255);

const engine::Color SimulatorScreen::LIGHTBULB_COLOR =
    engine::Color(0, 0, 0, 0);

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
    std::shared_ptr<Configuration> conf = m_game->m_model_confs[i];
    bool visible = (i == m_game->m_chosen_model);
    conf->model()->set_visible(visible);
    conf->dashboard()->set_visible(visible);
    if (visible) {
      conf->reset_for_simulation();
      conf->model()->set_position(conf->INIT_POSITION);
      conf->model()->set_velocity(conf->INIT_VELOCITY);
      conf->model()->set_rotation(engine::mat4::RotateXYZ(conf->INIT_ROTATION));
    }
  }
  m_help_text->set_position(1.0f, 20, Origin::MAX, Origin::MIN);
  m_full_help_text_background->set_position(engine::Rect2D{0, 0, 1.0f, 1.0f});
  m_full_help_text->set_position(0.5f, 0.3f, Origin::MID, Origin::MIN);

  // Set the camera
  m_game->m_device.get_camera().SetPosition(CAMERA_POSITION);

  // Set the lighting.
  m_game->m_device.set_ambient_light(AMBIENT_LIGHT_COLOR);
  m_game->m_sun_light->set_enabled(true);
  m_game->m_sun_light->set_color(SUN_LIGHT_COLOR);
  m_game->m_light_bulb->set_enabled(true);
  m_game->m_light_bulb->set_color(LIGHTBULB_COLOR);
}

SimulatorScreen::~SimulatorScreen() {
  m_game->m_device.delete_drawable2d(m_help_text);
  m_game->m_device.delete_drawable2d(m_full_help_text);
  m_game->m_device.delete_drawable2d(m_full_help_text_background);
}

void SimulatorScreen::frame(float time_delta) {

  if (engine::IsKeyPressed(KEY_C)) {
    m_game->m_current_screen = std::make_shared<ControllerConfigScreen>(
        m_game, m_game->m_current_screen);
    return;
  }

  // Update the model.
  std::shared_ptr<Configuration> conf =
      m_game->m_model_confs[m_game->m_chosen_model];
  UserInput user_input = m_game->m_input_receiver.update_input(time_delta);
  ServoData servo_data =
      conf->controls()->get_servo_data(user_input.controls_input, time_delta);
  for (size_t channel = 0; channel < servo_data.size(); channel++) {
    conf->model()->get_servo(channel).update(servo_data[channel], time_delta);
  }
  conf->model()->update(time_delta, engine::vec3(0, 0, 0));

  // Apply external force on the helicopter touch points.
  float model_mass = conf->model()->get_mass();
  conf->model()->reset_force();
  std::vector<FlyingObject::TouchPoint> touchpoints =
      conf->model()->get_touchpoints_in_world();
  for (unsigned int i = 0; i < touchpoints.size(); i++) {
    FlyingObject::TouchPoint tp = touchpoints[i];
    if (tp.pos.y < 0) {
      engine::vec3 tp_force = engine::vec3(0, -500 * tp.pos.y, 0) * model_mass;
      engine::vec3 friction_force;
      friction_force = tp.friction_coeff * tp.vel;
      friction_force.y = 10.0f * tp.vel.y;
      tp_force += -friction_force * (-tp.pos.y / 0.03) * model_mass;
      conf->model()->add_force(i, tp_force);
    }
  }

  // Help texts.
  m_full_help_text->set_visible(::IsKeyDown(KEY_H));
  m_full_help_text_background->set_visible(::IsKeyDown(KEY_H));

  // Draw.
  m_game->m_device.get_camera().SetTarget(conf->model()->get_position());
  conf->dashboard()->update_ui(conf->controls()->get_telemetry(),
                               conf->model()->get_telemetry());
  m_game->m_device.draw_frame();
}

////////////////////////////////////////////////////////////////////////////////
// ControllerConfigScreen implementation.
////////////////////////////////////////////////////////////////////////////////

ControllerConfigScreen::ControllerConfigScreen(
    Game *game, std::shared_ptr<GameScreen> return_to_screen)
    : GameScreen(game), m_background(m_game->m_device.create_square2d(
                            engine::Color(255, 255, 255, 180))),
      m_title(m_game->m_device.create_text2d(
          "Controller Configuration",
          engine::Text2D::FontOptions{60, engine::Color(0, 0, 0, 255)},
          m_background)),
      m_choose_gamepad_text_background(m_game->m_device.create_square2d(
          engine::Color(255, 255, 255, 20), m_background)),
      m_choose_gamepad_text(m_game->m_device.create_text2d(
          "",
          engine::Text2D::FontOptions{CONFIG_FONT_SIZE,
                                      engine::Color(0, 0, 0, 255)},
          m_choose_gamepad_text_background)),
      m_channel_config_background(m_game->m_device.create_square2d(
          engine::Color(255, 255, 255, 20), m_background)),
      m_help_text(m_game->m_device.create_text2d(
          "", engine::Text2D::FontOptions{20, engine::Color(0, 0, 0, 255), 1.5},
          m_background)),
      m_return_to_screen(return_to_screen) {
  // Set elements positions.
  m_background->set_position(engine::Rect2D{0, 0, 1.0f, 1.0f});
  m_title->set_position(0.5f, 0.1f, Origin::MID, Origin::MID);
  m_choose_gamepad_text_background->set_position(
      engine::Rect2D{0.05f, 0.2f, 0.9f, CONFIG_FONT_SIZE + 10});
  m_choose_gamepad_text->set_position(0.05f, 0.1f, Origin::MIN, Origin::MIN);
  m_channel_config_background->set_position(
      engine::Rect2D{0.1f, 0.35f, 0.8f, 0.4f});
  m_help_text->set_position(0.5f, 0.9f, Origin::MID, Origin::MID);

  // Create the channel configs UI elements.
  std::string channel_names[]{"Throttle:", "Pitch:", "Roll:", "Yaw:"};
  for (int i = 0; i < 4; i++) {
    ChannelConfig channel_config;
    channel_config.background = m_game->m_device.create_square2d(
        engine::Color(255, 255, 255, 20), m_channel_config_background);
    channel_config.channel_name_text = m_game->m_device.create_text2d(
        channel_names[i],
        engine::Text2D::FontOptions{CONFIG_FONT_SIZE,
                                    engine::Color(0, 0, 0, 255)},
        channel_config.background);
    channel_config.channel_number_text = m_game->m_device.create_text2d(
        "",
        engine::Text2D::FontOptions{CONFIG_FONT_SIZE,
                                    engine::Color(0, 0, 0, 255)},
        channel_config.background);
    channel_config.reverse_text = m_game->m_device.create_text2d(
        "Reverse",
        engine::Text2D::FontOptions{CONFIG_FONT_SIZE,
                                    engine::Color(0, 0, 0, 255)},
        channel_config.background);
    channel_config.channel_value_background = m_game->m_device.create_square2d(
        engine::Color(255, 255, 255, 255), channel_config.background);
    channel_config.channel_value_bar = m_game->m_device.create_square2d(
        engine::Color(0, 255, 0, 255), channel_config.channel_value_background);
    m_channel_configs.push_back(channel_config);

    // Set per-channel positions:
    channel_config.background->set_position(engine::Rect2D{
        0.05f, 1.0f / 8 + float(i) / 4, 0.9f, CONFIG_FONT_SIZE + 10});
    channel_config.channel_name_text->set_position(5, 0.5f, Origin::MIN,
                                                   Origin::MID);
    channel_config.channel_number_text->set_position(0.25f, 0.5f, Origin::MIN,
                                                     Origin::MID);
    channel_config.reverse_text->set_position(0.5f, 0.57f, Origin::MIN,
                                              Origin::MID);
    channel_config.channel_value_background->set_position(
        engine::Rect2D{0.75f, 0.05f, 0.24f, 0.9f});
    channel_config.channel_value_bar->set_position(
        engine::Rect2D{0.05f, 0.1f, 0.9f, 0.8f});
  }
}

void ControllerConfigScreen::frame(float time_delta) {
  UserInput user_input = m_game->m_input_receiver.update_input(time_delta);

  // Update from user keyboard input.
  if (engine::IsKeyPressed(KEY_UP)) {
    m_user_focus = (m_user_focus + 5 - 1) % 5;
  } else if (engine::IsKeyPressed(KEY_DOWN)) {
    m_user_focus = (m_user_focus + 1) % 5;
  }

  if (engine::IsKeyPressed(KEY_LEFT)) {
    switch (m_user_focus) {
    case 0:
      cycle_active_joystick(-1);
      break;
    case 1:
      m_game->m_input_receiver.get_config().throttle_channel.joystick_channel--;
      break;
    case 2:
      m_game->m_input_receiver.get_config().pitch_channel.joystick_channel--;
      break;
    case 3:
      m_game->m_input_receiver.get_config().roll_channel.joystick_channel--;
      break;
    case 4:
      m_game->m_input_receiver.get_config().yaw_channel.joystick_channel--;
      break;
    }
  } else if (engine::IsKeyPressed(KEY_RIGHT)) {
    switch (m_user_focus) {
    case 0:
      cycle_active_joystick(1);
      break;
    case 1:
      m_game->m_input_receiver.get_config().throttle_channel.joystick_channel++;
      break;
    case 2:
      m_game->m_input_receiver.get_config().pitch_channel.joystick_channel++;
      break;
    case 3:
      m_game->m_input_receiver.get_config().roll_channel.joystick_channel++;
      break;
    case 4:
      m_game->m_input_receiver.get_config().yaw_channel.joystick_channel++;
      break;
    }
  }

  if (IsKeyPressed(KEY_R)) {
    switch (m_user_focus) {
    case 1:
      m_game->m_input_receiver.get_config().throttle_channel.reverse =
          !m_game->m_input_receiver.get_config().throttle_channel.reverse;
      break;
    case 2:
      m_game->m_input_receiver.get_config().pitch_channel.reverse =
          !m_game->m_input_receiver.get_config().pitch_channel.reverse;
      break;
    case 3:
      m_game->m_input_receiver.get_config().roll_channel.reverse =
          !m_game->m_input_receiver.get_config().roll_channel.reverse;
      break;
    case 4:
      m_game->m_input_receiver.get_config().yaw_channel.reverse =
          !m_game->m_input_receiver.get_config().yaw_channel.reverse;
      break;
    }
  }

  if (engine::IsKeyPressed(KEY_ENTER)) {
    m_game->m_current_screen = m_return_to_screen;
    return;
  }

  if (m_game->m_input_receiver.get_config().active_joystick_name == "") {
    m_user_focus = 0;
    m_channel_config_background->set_visible(false);
  } else {
    m_channel_config_background->set_visible(true);
  }

  // Update focus colors.
  m_choose_gamepad_text_background->set_color(
      engine::Color(255, 255, 255, m_user_focus == 0 ? 120 : 0));
  for (int i = 0; i < m_channel_configs.size(); i++) {
    m_channel_configs[i].background->set_color(
        engine::Color(255, 255, 255, m_user_focus == (i + 1) ? 120 : 0));
  }

  // Update from input config.
  update_from_input_config(m_game->m_input_receiver.get_config(), user_input);

  // Update help text.
  if (m_user_focus == 0) {
    m_help_text->set_text("Use left/right to change active controller / "
                          "Keyboard\nUse Enter to confirm.");
  } else {
    m_help_text->set_text("Use left/right to change channel number.\nPress 'R' "
                          "to toggle reverse.\nUse Enter to confirm.");
  }

  m_game->m_device.draw_frame();
}

void ControllerConfigScreen::cycle_active_joystick(int amount) {
  auto available_joysticks = m_game->m_input_receiver.get_available_joysticks();
  UserInputReciever::Config &config = m_game->m_input_receiver.get_config();
  config.active_joystick_name =
      available_joysticks[(std::find(available_joysticks.begin(),
                                     available_joysticks.end(),
                                     config.active_joystick_name) -
                           available_joysticks.begin() + amount) %
                          available_joysticks.size()];
}

void ControllerConfigScreen::update_from_input_config(
    const UserInputReciever::Config &input_config,
    const UserInput &user_input) {
  // Update chosen gamepad text.
  if (input_config.active_joystick_name != "") {
    m_choose_gamepad_text->set_text("Input from gamepad: " +
                                    input_config.active_joystick_name);
  } else {
    m_choose_gamepad_text->set_text("Input from keyboard");
  }

  // Update channels config.
  update_channel_config(input_config.throttle_channel, 0,
                        user_input.controls_input.throttle_stick);
  update_channel_config(input_config.pitch_channel, 1,
                        user_input.controls_input.pitch_stick);
  update_channel_config(input_config.roll_channel, 2,
                        user_input.controls_input.roll_stick);
  update_channel_config(input_config.yaw_channel, 3,
                        user_input.controls_input.yaw_stick);
}

void ControllerConfigScreen::update_channel_config(
    const UserInputReciever::Config::Channel &channel_config,
    size_t channel_index, float value) {
  ChannelConfig &ui_config = m_channel_configs[channel_index];
  ui_config.channel_number_text->set_text(
      "Channel: " + std::to_string(channel_config.joystick_channel));
  if (channel_config.reverse) {
    ui_config.reverse_text->set_color(engine::Color(255, 0, 0, 255));
  } else {
    ui_config.reverse_text->set_color(engine::Color(128, 128, 128, 128));
  }

  engine::Rect2D bar_position = ui_config.channel_value_bar->get_position();
  bar_position.width = (value + 1) / 2 * 0.9f;
  ui_config.channel_value_bar->set_position(bar_position);
}

ControllerConfigScreen::~ControllerConfigScreen() {
  m_game->m_device.delete_drawable2d(m_background);
  m_game->m_device.delete_drawable2d(m_help_text);
  m_game->m_device.delete_drawable2d(m_title);
  m_game->m_device.delete_drawable2d(m_choose_gamepad_text);
  m_game->m_device.delete_drawable2d(m_choose_gamepad_text_background);
  m_game->m_device.delete_drawable2d(m_channel_config_background);
  for (ChannelConfig &channel_config : m_channel_configs) {
    m_game->m_device.delete_drawable2d(channel_config.background);
    m_game->m_device.delete_drawable2d(channel_config.channel_name_text);
    m_game->m_device.delete_drawable2d(channel_config.channel_number_text);
    m_game->m_device.delete_drawable2d(channel_config.reverse_text);
    m_game->m_device.delete_drawable2d(channel_config.channel_value_background);
    m_game->m_device.delete_drawable2d(channel_config.channel_value_bar);
  }
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
