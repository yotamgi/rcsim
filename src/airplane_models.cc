#include "airplane_models.h"

const float SimpleGlider::TOTAL_MASS = 0.3f;      // in Kg.
const float SimpleGlider::WING_LENGTH = 1.8f;     // in meters.
const float SimpleGlider::FUSELAGE_LENGTH = 1.2f; // in meters.
const float SimpleGlider::WING_WIDTH = 0.2f;      // in meters.
const float SimpleGlider::FUESLAGE_MASS = 0.1f;   // in Kg.
const float SimpleGlider::WING_MASS =
    SimpleGlider::TOTAL_MASS - SimpleGlider::FUESLAGE_MASS; // in Kg.

SimpleGlider::SimpleGlider(engine::RaylibDevice *device)
    : Airplane(
          {
              .mass = TOTAL_MASS,
              .moi = engine::vec3(
                  (1. / 12) * FUESLAGE_MASS * FUSELAGE_LENGTH * FUSELAGE_LENGTH,
                  (1. / 12) * WING_MASS * WING_LENGTH * WING_LENGTH,
                  (1. / 12) * FUSELAGE_LENGTH * FUSELAGE_LENGTH *
                          FUESLAGE_MASS +
                      (1. / 12) * WING_MASS * WING_LENGTH * WING_LENGTH),
              .airfoils =
                  {
                      // 0: Left wing:
                      {
                          .x_length = WING_LENGTH / 2,
                          .z_width = WING_WIDTH,
                          .y_thickness = 0.0001,
                          .rotation_angles = engine::vec3(-2, 0, -5),
                          .position_in_airplane =
                              engine::vec3(-WING_LENGTH / 4, 0, 0),
                          .num_points = 5,
                          .stall_angle_min = -3,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 2 * WING_WIDTH * 0.2f,
                          .flap_point_from = 0,
                          .flap_point_to = 2,
                          .max_flap_angle = 45,
                      },
                      // 1: Right wing:
                      {
                          .x_length = WING_LENGTH / 2,
                          .z_width = WING_WIDTH,
                          .y_thickness = 0.0001,
                          .rotation_angles = engine::vec3(-2, 0, 5),
                          .position_in_airplane =
                              engine::vec3(WING_LENGTH / 4, 0, 0),
                          .num_points = 5,
                          .stall_angle_min = -3,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 2 * WING_WIDTH * 0.2f,
                          .flap_point_from = 2,
                          .flap_point_to = 4,
                          .max_flap_angle = 45,
                      },
                      // 2: Fuselage:
                      {
                          .x_length = FUSELAGE_LENGTH,
                          .z_width = 0.01,
                          .y_thickness = 0.001,
                          .rotation_angles = engine::vec3(0, 90, 90),
                          .position_in_airplane =
                              engine::vec3(0, -0.05, -FUSELAGE_LENGTH * 0.2f),
                          .num_points = 5,
                          .stall_angle_min = 0,
                          .stall_angle_max = 0,
                          .has_flap = false,
                      },
                      // 3: Elevator:
                      {
                          .x_length = WING_LENGTH / 4,
                          .z_width = WING_WIDTH / 2,
                          .y_thickness = 0.0005,
                          .rotation_angles = engine::vec3(0, 0, 0),
                          .position_in_airplane =
                              engine::vec3(0, 0, -FUSELAGE_LENGTH * 0.6f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 2 * 0.2f,
                          .flap_point_from = 0,
                          .flap_point_to = 1,
                          .max_flap_angle = 45,
                      },
                      // 4: Rudder:
                      {
                          .x_length = WING_LENGTH / 6,
                          .z_width = WING_WIDTH / 2,
                          .y_thickness = 0.0005,
                          .rotation_angles = engine::vec3(0, 0, 90),
                          .position_in_airplane = engine::vec3(
                              0, WING_LENGTH / 8 / 2, -FUSELAGE_LENGTH * 0.6f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 2.0f * 0.4f,
                          .flap_point_from = 0,
                          .flap_point_to = 0,
                          .max_flap_angle = 45,
                      },
                  },
              .channel_flap_mapping =
                  {
                      {AIRPLANE_CHANNEL_PITCH, 3},
                      {AIRPLANE_CHANNEL_ROLL, 0},
                      {AIRPLANE_CHANNEL_YAW, 4},
                      {AIRPLANE_CHANNEL_FLAPRON, 1},
                  },
              .servo_max_rps = {1, 1, 1, 1, 1, 1},
              .servo_init_values = {-1, 0, 0, 0, 0, 0},
              .init_position = engine::vec3(1, 10, -1),
              .init_velocity = engine::vec3(0, 0, 10),
              .init_rotation = engine::vec3(0, 0, 0),
          },
          device) {}

const float Trainer::TOTAL_MASS = 2.0f;      // in Kg.
const float Trainer::WING_LENGTH = 2.0f;     // in meters.
const float Trainer::FUSELAGE_LENGTH = 1.4f; // in meters.
const float Trainer::WING_WIDTH = 0.35f;     // in meters.
const float Trainer::FUESLAGE_MASS = 1.5f;   // in Kg.
const float Trainer::WING_MASS =
    Trainer::TOTAL_MASS - Trainer::FUESLAGE_MASS; // in Kg.

Trainer::Trainer(engine::RaylibDevice *device)
    : Airplane(
          {
              .mass = TOTAL_MASS,
              .moi = engine::vec3(
                  (1. / 12) * FUESLAGE_MASS * FUSELAGE_LENGTH * FUSELAGE_LENGTH,
                  (1. / 12) * WING_MASS * WING_LENGTH * WING_LENGTH,
                  (1. / 12) * FUSELAGE_LENGTH * FUSELAGE_LENGTH *
                          FUESLAGE_MASS +
                      (1. / 12) * WING_MASS * WING_LENGTH * WING_LENGTH),
              .airfoils =
                  {
                      // 0: Left wing:
                      {
                          .x_length = WING_LENGTH / 2,
                          .z_width = WING_WIDTH,
                          .y_thickness = 0.01,
                          .rotation_angles = engine::vec3(-3, 0, -1),
                          .position_in_airplane =
                              engine::vec3(-WING_LENGTH / 4, 0.12, 0),
                          .num_points = 5,
                          .stall_angle_min = -10,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 3.0f * WING_WIDTH * 0.3f,
                          .flap_point_from = 0,
                          .flap_point_to = 3,
                          .max_flap_angle = 45,
                          .min_flap_angle = -40,
                      },
                      // 1: Right wing:
                      {
                          .x_length = WING_LENGTH / 2,
                          .z_width = WING_WIDTH,
                          .y_thickness = 0.01,
                          .rotation_angles = engine::vec3(-3, 0, 1),
                          .position_in_airplane =
                              engine::vec3(WING_LENGTH / 4, 0.12, 0),
                          .num_points = 5,
                          .stall_angle_min = -10,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 3.0f * WING_WIDTH * 0.3f,
                          .flap_point_from = 1,
                          .flap_point_to = 4,
                          .max_flap_angle = 45,
                          .min_flap_angle = -40,
                      },
                      // 2: Fuselage 1:
                      {
                          .x_length = FUSELAGE_LENGTH,
                          .z_width = 0.10,
                          .y_thickness = 1e-4,
                          .rotation_angles = engine::vec3(90, 0, 90),
                          .position_in_airplane =
                              engine::vec3(0, -0.05, -FUSELAGE_LENGTH * 0.2f),
                          .num_points = 5,
                          .stall_angle_min = 0,
                          .stall_angle_max = 0,
                          .has_flap = false,
                      },
                      // 3: Elevator:
                      {
                          .x_length = WING_LENGTH / 4,
                          .z_width = WING_WIDTH / 2,
                          .y_thickness = 0.005,
                          .rotation_angles = engine::vec3(0, 0, 0),
                          .position_in_airplane =
                              engine::vec3(0, -0.03, -FUSELAGE_LENGTH * 0.65f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 1.2f * 0.2f,
                          .flap_point_from = 0,
                          .flap_point_to = 1,
                          .max_flap_angle = 45,
                          .flap_mid_angle = -2,
                      },
                      // 4: Rudder:
                      {
                          .x_length = WING_LENGTH / 6,
                          .z_width = WING_WIDTH / 2,
                          .y_thickness = 0.005,
                          .rotation_angles = engine::vec3(0, 0, 90),
                          .position_in_airplane = engine::vec3(
                              0, WING_LENGTH / 8 / 2, -FUSELAGE_LENGTH * 0.7f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 2.0f * 0.4f,
                          .flap_point_from = 0,
                          .flap_point_to = 0,
                          .max_flap_angle = 45,
                      },
                      // 5: Fuselage 2:
                      {
                          .x_length = FUSELAGE_LENGTH,
                          .z_width = 0.1,
                          .y_thickness = 1e-4,
                          .rotation_angles = engine::vec3(0, 90, 0),
                          .position_in_airplane =
                              engine::vec3(0, -0.05, -FUSELAGE_LENGTH * 0.2f),
                          .num_points = 5,
                          .stall_angle_min = 0,
                          .stall_angle_max = 0,
                          .has_flap = false,
                      },
                  },
              .channel_flap_mapping =
                  {
                      {AIRPLANE_CHANNEL_PITCH, 3},
                      {AIRPLANE_CHANNEL_ROLL, 0},
                      {AIRPLANE_CHANNEL_YAW, 4},
                      {AIRPLANE_CHANNEL_FLAPRON, 1},
                  },
              .propellants =
                  {
                      {
                          .direction_in_airplane = engine::vec3(0, 0, -1),
                          .position_in_airplane =
                              engine::vec3(0.0f, 0.0f, FUSELAGE_LENGTH * 0.3f),
                          .thrust_airspeed = 50.0f,
                          .max_thrust = 18.0f,
                      },
                  },
              .channel_prop_mapping =
                  {
                      {AIRPLANE_CHANNEL_THROTTLE, 0},
                  },
              .touchpoints_in_airplane =
                  {
                      // Wheels:
                      {.pos = engine::vec3(0.0, -0.26, 0.3),
                       .friction_coeff = diag2(15.0f, 0.3f)},
                      {.pos = engine::vec3(0.23, -0.26, -0.07),
                       .friction_coeff = diag2(15.0f, 0.3f)},
                      {.pos = engine::vec3(-0.23, -0.26, -0.07),
                       .friction_coeff = diag2(15.0f, 0.3f)},
                      // Wing touchpoints:
                      {.pos = engine::vec3(1.0, 0.12, 0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = engine::vec3(-1.0, 0.12, 0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = engine::vec3(1.0, 0.12, -0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = engine::vec3(-1.0, 0.12, -0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      // Back touchpoint:
                      {.pos = engine::vec3(0.0, 0.0, -1.),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      // Engine touchpoint:
                      {.pos = engine::vec3(0, 0, FUSELAGE_LENGTH * 0.2),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                  },
              .touchpoint_to_channel_mapping =
                  {
                      {0,
                       {.servo_index = AIRPLANE_CHANNEL_YAW, .max_angle = 45}},
                  },
              .servo_max_rps = {1, 3, 3, 3, 3, 3},
              .servo_init_values = {-1, 0, 0, 0, 0, 0},
              .init_position = engine::vec3(0, 0.3, 0),
              .init_velocity = engine::vec3(0, 0, 0),
              .init_rotation = engine::vec3(0, 0, 0),
              .show_skeleton = false,
          },
          device) {

  m_body_node = device->load_model(
      "resources/media/cessna/CessnaBodyFixed2.obj", m_ui_node);

  m_body_node->set_transform(engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
                             engine::mat4::Translate(0, -0.07, 0));

  m_left_ailron_node =
      device->load_model("resources/media/cessna/CessnaAilron.obj", m_ui_node);
  m_left_ailron_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::Translate(-0.33, 0.105, -0.12));

  m_right_ailron_node =
      device->load_model("resources/media/cessna/CessnaAilron.obj", m_ui_node);
  m_right_ailron_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::Translate(0.33, 0.105, -0.12));

  m_elevator_node = device->load_model(
      "resources/media/cessna/CessnaElevator.obj", m_ui_node);
  m_elevator_node->set_transform(engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
                                 engine::mat4::Translate(0, -0.04, -0.9));

  std::shared_ptr<engine::Model> rudder_base = device->create_empty(m_ui_node);
  rudder_base->set_transform(engine::mat4::RotateX(-30 / 180. * PI) *
                             engine::mat4::Translate(0, 0, -0.86));
  m_rudder_node = device->load_model("resources/media/cessna/CessnaRudder.obj",
                                     rudder_base);
  m_rudder_node->set_transform(engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4));

  m_prop_node =
      device->load_model("resources/media/cessna/CessnaProp.obj", m_ui_node);
  m_prop_node->set_transform(engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
                             engine::mat4::Translate(0, 0, 0.4));

  m_prop_angle = 0.0f;
}

void Trainer::update_ui() {
  Airplane::update_ui();
  m_prop_angle +=
      (m_servos[AIRPLANE_CHANNEL_THROTTLE].get() + 1.0) * 3412.0f * 0.1f;
  m_prop_node->set_transform(engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
                             engine::mat4::RotateZ(m_prop_angle / 180. * PI) *
                             engine::mat4::Translate(0, 0, 0.4));

  float roll_angle =
      m_servos[AIRPLANE_CHANNEL_ROLL].get() * 45.0f; // in degrees.
  m_left_ailron_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::RotateXYZ(engine::vec3(roll_angle, 0, -4) / 180. * PI) *
      engine::mat4::Translate(-0.33, 0.105, -0.12));
  m_right_ailron_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::RotateXYZ(engine::vec3(-roll_angle - 7, 0, 183) / 180. * PI) *
      engine::mat4::Translate(0.33, 0.105, -0.12));
  m_elevator_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::RotateX(m_servos[AIRPLANE_CHANNEL_PITCH].get() * 45. / 180.0 * PI) *
      engine::mat4::Translate(0, -0.04, -0.9));
  m_rudder_node->set_transform(
      engine::mat4::Scale(1. / 4, 1. / 4, 1. / 4) *
      engine::mat4::RotateY(m_servos[AIRPLANE_CHANNEL_YAW].get() * 45. / 180.0 * PI));
}