
#include "airplane_models.h"

const float SimpleGlider::TOTAL_MASS = 0.3f;      // in Kg.
const float SimpleGlider::WING_LENGTH = 1.8f;     // in meters.
const float SimpleGlider::FUSELAGE_LENGTH = 1.2f; // in meters.
const float SimpleGlider::WING_WIDTH = 0.2f;      // in meters.
const float SimpleGlider::FUESLAGE_MASS = 0.1f;   // in Kg.
const float SimpleGlider::WING_MASS =
    SimpleGlider::TOTAL_MASS - SimpleGlider::FUESLAGE_MASS; // in Kg.

SimpleGlider::SimpleGlider(irr::scene::ISceneManager *smgr,
                           irr::video::IVideoDriver *driver)
    : Airplane(
          {
              .mass = TOTAL_MASS,
              .moi = irrvec3(
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
                          .rotation_angles = irrvec3(-2, 0, -5),
                          .position_in_airplane =
                              irrvec3(-WING_LENGTH / 4, 0, 0),
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
                          .rotation_angles = irrvec3(-2, 0, 5),
                          .position_in_airplane =
                              irrvec3(WING_LENGTH / 4, 0, 0),
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
                          .rotation_angles = irrvec3(0, 90, 90),
                          .position_in_airplane =
                              irrvec3(0, -0.05, -FUSELAGE_LENGTH * 0.2f),
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
                          .rotation_angles = irrvec3(0, 0, 0),
                          .position_in_airplane =
                              irrvec3(0, 0, -FUSELAGE_LENGTH * 0.6f),
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
                          .rotation_angles = irrvec3(0, 0, 90),
                          .position_in_airplane = irrvec3(
                              0, WING_LENGTH / 8 / 2, -FUSELAGE_LENGTH * 0.6f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 2 * 0.4,
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
              .init_position = irrvec3(1, 10, -1),
              .init_velocity = irrvec3(0, 0, 10),
              .init_rotation = irrvec3(0, 0, 0),
          },
          smgr, driver) {}

const float Trainer::TOTAL_MASS = 2.0f;      // in Kg.
const float Trainer::WING_LENGTH = 2.0f;     // in meters.
const float Trainer::FUSELAGE_LENGTH = 1.4f; // in meters.
const float Trainer::WING_WIDTH = 0.3f;      // in meters.
const float Trainer::FUESLAGE_MASS = 1.5f;   // in Kg.
const float Trainer::WING_MASS =
    Trainer::TOTAL_MASS - Trainer::FUESLAGE_MASS; // in Kg.

Trainer::Trainer(irr::scene::ISceneManager *smgr,
                 irr::video::IVideoDriver *driver)
    : Airplane(
          {
              .mass = TOTAL_MASS,
              .moi = irrvec3(
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
                          .rotation_angles = irrvec3(-3, 0, -5),
                          .position_in_airplane =
                              irrvec3(-WING_LENGTH / 4, 0, 0),
                          .num_points = 5,
                          .stall_angle_min = -5,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 1.2 * WING_WIDTH * 0.2f,
                          .flap_point_from = 0,
                          .flap_point_to = 2,
                          .max_flap_angle = 45,
                      },
                      // 1: Right wing:
                      {
                          .x_length = WING_LENGTH / 2,
                          .z_width = WING_WIDTH,
                          .y_thickness = 0.01,
                          .rotation_angles = irrvec3(-3, 0, 5),
                          .position_in_airplane =
                              irrvec3(WING_LENGTH / 4, 0, 0),
                          .num_points = 5,
                          .stall_angle_min = -5,
                          .stall_angle_max = 15,
                          .has_flap = true,
                          .flap_area = WING_LENGTH / 1.2 * WING_WIDTH * 0.2f,
                          .flap_point_from = 2,
                          .flap_point_to = 4,
                          .max_flap_angle = 45,
                      },
                      // 2: Fuselage:
                      {
                          .x_length = FUSELAGE_LENGTH,
                          .z_width = 0.01,
                          .y_thickness = 0.1,
                          .rotation_angles = irrvec3(0, 90, 90),
                          .position_in_airplane =
                              irrvec3(0, -0.05, -FUSELAGE_LENGTH * 0.2f),
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
                          .rotation_angles = irrvec3(0, 0, 0),
                          .position_in_airplane =
                              irrvec3(0, 0, -FUSELAGE_LENGTH * 0.6f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 1.2 * 0.2f,
                          .flap_point_from = 0,
                          .flap_point_to = 1,
                          .max_flap_angle = 45,
                      },
                      // 4: Rudder:
                      {
                          .x_length = WING_LENGTH / 6,
                          .z_width = WING_WIDTH / 2,
                          .y_thickness = 0.005,
                          .rotation_angles = irrvec3(0, 0, 90),
                          .position_in_airplane = irrvec3(
                              0, WING_LENGTH / 8 / 2, -FUSELAGE_LENGTH * 0.6f),
                          .num_points = 2,
                          .stall_angle_min = -3,
                          .stall_angle_max = 3,
                          .has_flap = true,
                          .flap_area = WING_WIDTH / 2 * 0.4,
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
              .propellants =
                  {
                      {
                          .direction_in_airplane = irrvec3(0, 0, -1),
                          .position_in_airplane =
                              irrvec3(0, 0, FUSELAGE_LENGTH * 0.2),
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
                      {.pos = irrvec3(0.0, -0.3, 0.3),
                       .friction_coeff = diag2(15.0f, 0.0f)},
                      {.pos = irrvec3(0.3, -0.3, -0.1),
                       .friction_coeff = diag2(15.0f, 0.0f)},
                      {.pos = irrvec3(-0.3, -0.3, -0.1),
                       .friction_coeff = diag2(15.0f, 0.0f)},
                      // Wing touchpoints:
                      {.pos = irrvec3(1.0, 0.05, 0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = irrvec3(-1.0, 0.05, 0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = irrvec3(1.0, 0.05, -0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      {.pos = irrvec3(-1.0, 0.05, -0.14),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      // Back touchpoint:
                      {.pos = irrvec3(0.0, 0.0, -1.),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                      // Engine touchpoint:
                      {.pos = irrvec3(0, 0, FUSELAGE_LENGTH * 0.2),
                       .friction_coeff = diag2(3.0f, 3.0f)},
                  },
              .servo_max_rps = {1, 1, 1, 1, 1, 1},
              .servo_init_values = {-1, 0, 0, 0, 0, 0},
              .init_position = irrvec3(1, 0.3, -1),
              .init_velocity = irrvec3(0, 0, 0),
              .init_rotation = irrvec3(0, 0, 0),
          },
          smgr, driver) {}
