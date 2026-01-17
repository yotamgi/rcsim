/** Simulator main file. */
// #include "input_event_reciever.h"
// #include "model_configurations.h"
#include "raylib_engine.h"

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

void add_banana(engine::RaylibDevice &device, const engine::vec3 &pos,
                const engine::vec3 &rotation) {
  std::shared_ptr<engine::Model> banana_model =
      device.load_model("resources/media/banana/source/banana.obj");
  ::SetMaterialTexture(
      &banana_model->GetMaterials()[0], 0,
      ::LoadTexture("resources/media/banana/textures/rgb.jpeg"));

  banana_model->SetTransform(
      engine::mat4::Scale(1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f) *
      engine::mat4::RotateX(rotation.x) * engine::mat4::RotateY(rotation.y) *
      engine::mat4::RotateZ(rotation.z) *
      engine::mat4::Translate(pos.x, pos.y, pos.z));
}

int main() {

  engine::RaylibDevice device(1440, 900, "rcsim - RC Simulator");
  device.create_light(LIGHT_DIRECTIONAL, raylib::Vector3(-200, 100, -200),
                      raylib::Vector3(0, 0, 0),
                      raylib::Color(255, 255, 255, 255));

  // Load the stadium model.
  std::shared_ptr<engine::Model> stadium_model = device.load_model(
      "resources/media/BasketballStadium/source/63BasketBallZemin.obj");
  raylib::Material stadium_material = stadium_model->GetMaterials()[0];
  raylib::Texture stadium_texture(
      "resources/media/BasketballStadium/textures/BasketZemin_Color.png");
  stadium_texture.GenMipmaps();
  stadium_texture.SetFilter(TEXTURE_FILTER_TRILINEAR);
  stadium_material.SetTexture(MATERIAL_MAP_DIFFUSE, stadium_texture);

  stadium_model->SetTransform(engine::mat4::Translate(0, 0, 0) *
                              engine::mat4::Scale(4, 4, 4));


  // Add some bananas for scale.
  add_banana(device, engine::vec3(-0.5, 0.05, 0.2), engine::vec3(90, 73, 0));
  add_banana(device, engine::vec3(0.9, 0.05, 0.3), engine::vec3(90, 40, 0));
  add_banana(device, engine::vec3(0.4, 0.05, 0.0), engine::vec3(90, 0, 0));

  // Add skybox.
  device.add_skybox_from_6_images(
      "resources/media/skybox/px.png", "resources/media/skybox/nx.png",
      "resources/media/skybox/py.png", "resources/media/skybox/ny.png",
      "resources/media/skybox/pz.png", "resources/media/skybox/nz.png");

  device.get_camera().SetPosition(raylib::Vector3(0.0f, 2.0f, 5.0f));

  int lastFPS = -1;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  double then = tv.tv_sec * 1000. + tv.tv_usec / 1000.;
  double time_delta;

  // Configuration model_conf = MODEL_CONFIGURATIONS[2].create(driver, smgr);

  // float model_mass = model_conf.model->get_mass();

  while (!WindowShouldClose()) {
    // calculate the delta time, and make sure it does not exceed 100 fps
    double now;
    gettimeofday(&tv, NULL);
    now = tv.tv_sec * 1000. + tv.tv_usec / 1000.;
    time_delta = (now - then) / 1000.;
    then = now;
    time_delta = time_delta > 0.03 ? 0.03 : time_delta;

    // // Update the model.
    // UserInput user_input = receiver.update_input(time_delta);
    // ServoData servo_data = model_conf.controls->get_servo_data(
    //     user_input.controls_input, time_delta);
    // for (size_t channel = 0; channel < servo_data.size(); channel++) {
    //   model_conf.model->get_servo(channel).update(servo_data[channel],
    //                                               time_delta);
    // }
    // model_conf.model->update(time_delta, irrvec3(0, 0, 0));

    // // Apply external force on the helicopter touch points.
    // model_conf.model->reset_force();
    // std::vector<BaseHeli::TouchPoint> touchpoints =
    //     model_conf.model->get_touchpoints_in_world();
    // for (unsigned int i = 0; i < touchpoints.size(); i++) {
    //   BaseHeli::TouchPoint tp = touchpoints[i];
    //   if (tp.pos.Y < 0) {
    //     irrvec3 tp_force = irrvec3(0, -500 * tp.pos.Y, 0) * model_mass;
    //     irrvec3 friction_force;
    //     friction_force = tp.vel;
    //     tp.friction_coeff.rotateVect(friction_force);
    //     friction_force.Y = 10.0f * tp.vel.Y;
    //     tp_force += -friction_force * (-tp.pos.Y / 0.03) * model_mass;
    //     model_conf.model->add_force(i, tp_force);
    //   }
    // }

    // Draw.
    // device.get_camera().SetTarget(model_conf.model->get_position());
    device.get_camera().SetTarget(raylib::Vector3(0, 0.5, 0));
    // model_conf.dashboard->update_ui(model_conf.controls->get_telemetry(),
    //                                 model_conf.model->get_telemetry());
    // driver->endScene();

    device.draw_frame();
  }

  return 0;
}