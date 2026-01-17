#include "airplane.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

const double GRAVITY_CONSTANT = 9.8;
const float AIR_DENSITY = 1.2;
const float LIFT_COEFFICIENT = 3;

typedef irr::core::vector3df irrvec3;

static irrvec3 advection_force(const irrvec3 &airflow,
                               const irrvec3 &surface_normal,
                               float surface_area) {
  irrvec3 airflow_direction = irrvec3(airflow).normalize();
  float airflow_dot_normal = airflow.dotProduct(surface_normal);
  float effective_area =
      std::sqrt(std::abs(airflow_direction.dotProduct(surface_normal))) *
      surface_area;

  // The advection force is the change of momentum of the air.
  float advection_magnitude =
      // Total mass of air that hits the airfoil per second [Kg / SEC]
      AIR_DENSITY * std::abs(effective_area) *
      airflow.getLength()
      // Times the change of velocity [M / SEC]
      * airflow_dot_normal;
  return surface_normal * advection_magnitude;
}

static irrvec3 rotate(irr::core::matrix4 matrix, irrvec3 vec) {
  matrix.rotateVect(vec);
  return vec;
}

////////////////////////////////////////////////////////////////////////////////
// PointAirFoil class implementation
////////////////////////////////////////////////////////////////////////////////

AppliedForce PointAirFoil::get_force(const irrvec3 &airflow) {
  float airflow_dot_normal = airflow.dotProduct(m_params.normal);
  float airflow_dot_wing = airflow.dotProduct(m_params.wing_airflow_direction);
  float angle_of_attack =
      std::atan2(airflow_dot_normal, -airflow_dot_wing) / M_PI * 180;

  // The Advection lift is the force that originates in the fact that air
  // collides in the wing.
  irrvec3 lift = advection_force(airflow, m_params.normal, m_params.area);

  // The drag;
  irrvec3 drag = advection_force(airflow, m_params.wing_airflow_direction,
                                 m_params.drag_area);

  // The flaps
  irrvec3 flap = advection_force(airflow, m_flap_normal, m_params.flap_area);

  // The wing effect, which depends on the airflow on the (normal, wing
  // direction) plane.
  irrvec3 wing_effect;
  if ((angle_of_attack > m_params.stall_angle_min) &&
      (angle_of_attack < m_params.stall_angle_max)) {
    wing_effect = (lift + flap) * LIFT_COEFFICIENT;
  }

  // The total force.
  irrvec3 force = drag + lift + flap + wing_effect;

  // If UI initialized, update it.
  if (m_force_arrow) {
    m_force_arrow->point(force);
  }

  return AppliedForce{.force = force,
                      .position_in_airplane = m_params.position_in_airplane};
}

void PointAirFoil::init_ui(irr::scene::ISceneManager *smgr,
                           irr::video::IVideoDriver *driver,
                           irr::scene::ISceneNode *parent) {
  irr::scene::ISceneNode *point_node = smgr->addSphereSceneNode(
      0.03f, 16, parent, -1, m_params.position_in_airplane);
  m_force_arrow = std::make_shared<Arrow>(smgr, point_node);

  point_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
  point_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
  point_node->setDebugDataVisible(irr::scene::EDS_OFF);
}

////////////////////////////////////////////////////////////////////////////////
// RectangularAirFoil class implementation
////////////////////////////////////////////////////////////////////////////////

RectangularAirFoil::RectangularAirFoil(const RectangularAirFoil::Params &params)
    : m_params(params) {
  irrmat4 rotoation_mat;
  rotoation_mat.setRotationDegrees(m_params.rotation_angles);
  irrvec3 normal(0, 1, 0), wing_direction(0, 0, 1), along_wing(1, 0, 0);
  rotoation_mat.rotateVect(normal);
  rotoation_mat.rotateVect(wing_direction);
  rotoation_mat.rotateVect(along_wing);

  irrvec3 wing_start =
      m_params.position_in_airplane - along_wing * m_params.x_length / 2;
  float points_interval = m_params.x_length / (m_params.num_points - 1);
  float wing_area = m_params.z_width * m_params.x_length;
  float point_area = wing_area / (m_params.num_points - 1);
  float num_flap_points = m_params.flap_point_to - m_params.flap_point_from + 1;
  float drag_area = m_params.y_thickness * m_params.x_length;
  float point_drag_area = drag_area / (m_params.num_points + 1);

  for (int i = 0; i < m_params.num_points; i++) {
    irrvec3 position = wing_start + i * points_interval * along_wing;
    float point_specific_area = point_area;
    float point_specific_drag_area = point_drag_area;
    if ((i == 0) || (i == m_params.num_points - 1)) {
      point_specific_area /=
          2.0f; // The first and last points are half the area.
      point_specific_drag_area /=
          2.0f; // The first and last points are half the drag area.
    }
    float point_flap_area = 0.0f;
    if (m_params.has_flap && (i >= m_params.flap_point_from) &&
        (i <= m_params.flap_point_to)) {
      point_flap_area = m_params.flap_area / num_flap_points;
    }
    PointAirFoil::Params point_params{
        .area = point_specific_area,
        .flap_area = point_flap_area,
        .drag_area = point_specific_drag_area,
        .position_in_airplane = position,
        .normal = normal,
        .wing_airflow_direction = wing_direction,
        .stall_angle_min = m_params.stall_angle_min,
        .stall_angle_max = m_params.stall_angle_max,
    };
    m_point_airfoils.push_back(std::make_shared<PointAirFoil>(point_params));
  }
  m_along_wing_direction = wing_direction.crossProduct(normal);
  m_wing_normal = normal;
}

std::vector<AppliedForce>
RectangularAirFoil::calc_force(const irrvec3 &wind_in_airplane,
                               const irrvec3 &velocity_in_airplane,
                               const irrvec3 &omega_in_airplane) {
  std::vector<AppliedForce> forces;
  for (auto point_airfoil : m_point_airfoils) {
    irrvec3 point_airflow =
        wind_in_airplane - velocity_in_airplane -
        omega_in_airplane.crossProduct(point_airfoil->get_position());
    AppliedForce force = point_airfoil->get_force(point_airflow);
    forces.push_back(force);
  }
  return forces;
}

void RectangularAirFoil::set_flap(float value) {
  float max_angle = m_params.max_flap_angle;
  float min_angle = (m_params.min_flap_angle == 0.)
                        ? max_angle
                        : std::abs(m_params.min_flap_angle);
  float angle =
      value * (value > 0 ? max_angle : min_angle) + m_params.flap_mid_angle;
  float angle_radians = angle / 180 * M_PI;
  irrmat4 rotation;
  rotation.setRotationAxisRadians(-angle_radians, m_along_wing_direction);
  irrvec3 flap_normal = m_wing_normal;
  rotation.rotateVect(flap_normal);
  for (auto point_airfoil : m_point_airfoils) {
    point_airfoil->set_flap_normal(flap_normal);
  }
}

void RectangularAirFoil::init_ui(irr::scene::ISceneManager *smgr,
                                 irr::video::IVideoDriver *driver,
                                 irr::scene::ISceneNode *parent) {
  irr::scene::IMeshSceneNode *airfoil_node = smgr->addCubeSceneNode(
      1.0f, parent, -1, m_params.position_in_airplane, m_params.rotation_angles,
      irr::core::vector3df(m_params.x_length, m_params.y_thickness,
                           m_params.z_width));

  airfoil_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
  airfoil_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
  airfoil_node->setDebugDataVisible(irr::scene::EDS_OFF);
  airfoil_node->addShadowVolumeSceneNode();

  for (auto point_airfoil : m_point_airfoils) {
    point_airfoil->init_ui(smgr, driver, parent);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Propellant class implementation
////////////////////////////////////////////////////////////////////////////////

void Propellant::init_ui(irr::scene::ISceneManager *smgr,
                         irr::video::IVideoDriver *driver,
                         irr::scene::ISceneNode *parent) {
  irr::scene::ISceneNode *point_node = smgr->addSphereSceneNode(
      0.1f, 16, parent, -1, m_params.position_in_airplane);
  m_force_arrow = std::make_shared<Arrow>(smgr, point_node);

  point_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
  point_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
  point_node->setDebugDataVisible(irr::scene::EDS_OFF);
}

AppliedForce Propellant::calc_force(const irrvec3 &wind_in_airplane,
                                    const irrvec3 &velocity_in_airplane) {
  irrvec3 airflow = wind_in_airplane - velocity_in_airplane;
  float airflow_dot_direction =
      m_params.direction_in_airplane.dotProduct(airflow);
  float airflow_diff = m_params.thrust_airspeed - airflow_dot_direction;
  float airflow_ratio = airflow_diff / m_params.thrust_airspeed;
  float force_magnitude = airflow_ratio * m_params.max_thrust * m_throttle;
  irrvec3 force = -force_magnitude * m_params.direction_in_airplane;

  // If UI initialized, update it.
  if (m_force_arrow) {
    m_force_arrow->point(force);
  }

  return AppliedForce{
      .force = force,
      .position_in_airplane = m_params.position_in_airplane // noindent
  };
}

////////////////////////////////////////////////////////////////////////////////
// Airplane class implementation.
////////////////////////////////////////////////////////////////////////////////

Airplane::Airplane(const Airplane::Params &params,
                   irr::scene::ISceneManager *smgr,
                   irr::video::IVideoDriver *driver)
    : m_ui_node(smgr->addEmptySceneNode()), m_params(params) {
  for (size_t i = 0; i < m_params.servo_max_rps.size(); i++) {
    m_servos.push_back(
        ServoFilter(m_params.servo_max_rps[i], m_params.servo_init_values[i]));
  }

  for (const auto &airfoil_params : m_params.airfoils) {
    std::shared_ptr<RectangularAirFoil> airfoil =
        std::make_shared<RectangularAirFoil>(airfoil_params);
    m_airfoils.push_back(airfoil);
  }

  for (const auto &prop_params : m_params.propellants) {
    std::shared_ptr<Propellant> prop =
        std::make_shared<Propellant>(prop_params);
    m_propellants.push_back(prop);
  }

  m_position_in_world = m_params.init_position;
  m_velocity_in_world = m_params.init_velocity;
  m_rotation_in_world.setRotationDegrees(m_params.init_rotation);

  init_ui(smgr, driver);
}

void Airplane::init_ui(irr::scene::ISceneManager *smgr,
                       irr::video::IVideoDriver *driver) {
  if (!m_params.show_skeleton)
    return;

  for (const auto &airfoil : m_airfoils) {
    airfoil->init_ui(smgr, driver, m_ui_node);
  }

  for (const auto &prop : m_propellants) {
    prop->init_ui(smgr, driver, m_ui_node);
  }

  for (const auto &tp : m_params.touchpoints_in_airplane) {
    irr::scene::ISceneNode *point_node =
        smgr->addSphereSceneNode(0.05f, 16, m_ui_node, -1, tp.pos);
    point_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    point_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    point_node->setDebugDataVisible(irr::scene::EDS_OFF);
  }
}

void Airplane::update(double time_delta, const irrvec3 &wind_speed) {
  // Update the servos.
  for (size_t i = 0; i < m_servos.size(); i++) {
    if (m_params.channel_flap_mapping.find(i) !=
        m_params.channel_flap_mapping.end()) {
      m_airfoils[m_params.channel_flap_mapping[i]]->set_flap(m_servos[i].get());
    }
    if (m_params.channel_prop_mapping.find(i) !=
        m_params.channel_prop_mapping.end()) {
      m_propellants[m_params.channel_prop_mapping[i]]->set_throttle(
          m_servos[i].get());
    }
  }

  // Calculate the forces from the airfoils.
  irrvec3 airfoil_forces_in_airplane;
  irrvec3 airfoil_torque_in_airplane;
  irrvec3 wind_in_airplane =
      rotate(m_rotation_in_world.getTransposed(), wind_speed);
  irrvec3 velocity_in_ariplane =
      rotate(m_rotation_in_world.getTransposed(), m_velocity_in_world);
  for (const auto &airfoil : m_airfoils) {
    std::vector<AppliedForce> forces_in_airplane = airfoil->calc_force(
        wind_in_airplane, velocity_in_ariplane, m_angular_velocity_in_airplane);
    for (const auto &force : forces_in_airplane) {
      airfoil_forces_in_airplane += force.force;
      airfoil_torque_in_airplane +=
          force.position_in_airplane.crossProduct(force.force);
    }
  }
  irrvec3 airfoil_force_in_world =
      rotate(m_rotation_in_world, airfoil_forces_in_airplane);

  // Calculate the force from the propellants.
  irrvec3 prop_forces_in_airplane;
  irrvec3 prop_torque_in_airplane;
  for (const auto &prop : m_propellants) {
    AppliedForce force_in_airplane =
        prop->calc_force(wind_in_airplane, velocity_in_ariplane);
    prop_forces_in_airplane += force_in_airplane.force;
    prop_torque_in_airplane +=
        force_in_airplane.position_in_airplane.crossProduct(
            force_in_airplane.force);
  }
  irrvec3 propellant_force_in_world =
      rotate(m_rotation_in_world, prop_forces_in_airplane);

  // Gravity force.
  irrvec3 gravity_in_world = irrvec3(0, -m_params.mass * GRAVITY_CONSTANT, 0);

  // Update position and velocity according to the forces.
  irrvec3 total_force = airfoil_force_in_world + propellant_force_in_world +
                        gravity_in_world + m_external_force_in_world;
  m_velocity_in_world += total_force / m_params.mass * time_delta;
  m_position_in_world += m_velocity_in_world * time_delta;

  // Update the rotation according to the moments using Euler's equation.
  irrvec3 total_torque_in_airplane = airfoil_torque_in_airplane +
                                     prop_torque_in_airplane +
                                     m_external_torque_in_airplane;
  m_angular_velocity_in_airplane +=
      time_delta *
      (total_torque_in_airplane -
       m_angular_velocity_in_airplane.crossProduct(
           m_params.moi * m_angular_velocity_in_airplane)) /
      m_params.moi;
  irrvec3 angulal_velocity_in_world =
      rotate(m_rotation_in_world, m_angular_velocity_in_airplane);
  update_rotation_matrix(m_rotation_in_world,
                         angulal_velocity_in_world * time_delta);

  // Update the UI node position and rotation.
  update_ui();

  m_airspeed = (m_velocity_in_world - wind_speed).getLength();
}

void Airplane::add_force(unsigned int touchpoint_index, const irrvec3 &force) {
  m_external_force_in_world += force;
  m_external_torque_in_airplane +=
      m_params.touchpoints_in_airplane[touchpoint_index].pos.crossProduct(
          rotate(m_rotation_in_world.getTransposed(), force));
}

void Airplane::reset_force() {
  m_external_force_in_world = irrvec3();
  m_external_torque_in_airplane = irrvec3();
}

std::vector<Airplane::TouchPoint> Airplane::get_touchpoints_in_world() const {
  std::vector<FlyingObject::TouchPoint> touchpoints_in_world;
  for (size_t tp_index = 0; tp_index < m_params.touchpoints_in_airplane.size();
       tp_index++) {
    auto touchpoint_in_body = m_params.touchpoints_in_airplane[tp_index];
    FlyingObject::TouchPoint tp;
    tp.pos = m_position_in_world +
             rotate(m_rotation_in_world, touchpoint_in_body.pos);
    tp.vel =
        m_velocity_in_world +
        rotate(m_rotation_in_world, m_angular_velocity_in_airplane.crossProduct(
                                        touchpoint_in_body.pos));
    irrmat4 friction_in_body = touchpoint_in_body.friction_coeff;
    if (m_params.touchpoint_to_channel_mapping.find(tp_index) !=
        m_params.touchpoint_to_channel_mapping.end()) {
      auto wheel_conf = m_params.touchpoint_to_channel_mapping.at(tp_index);
      float angle = m_servos[wheel_conf.servo_index].get() * wheel_conf.max_angle;
      irrmat4 wheel_rot;
      wheel_rot.setRotationDegrees(irrvec3(0, -angle, 0));
      friction_in_body =
          wheel_rot * friction_in_body * wheel_rot.getTransposed();
    }
    tp.friction_coeff = m_rotation_in_world * friction_in_body *
                        m_rotation_in_world.getTransposed();
    touchpoints_in_world.push_back(tp);
  }
  return touchpoints_in_world;
}

void Airplane::update_ui() {
  m_ui_node->setPosition(m_position_in_world);
  m_ui_node->setRotation(m_rotation_in_world.getRotationDegrees());
}

Airplane::Telemetry Airplane::get_telemetry() const {
  Telemetry telemetry;
  telemetry.rps = 0;
  telemetry.target_rps = 0;
  telemetry.velocity_magnitude = m_velocity_in_world.getLength();
  telemetry.airspeed = m_airspeed;
  return telemetry;
}