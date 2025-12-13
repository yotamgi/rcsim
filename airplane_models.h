#ifndef __AIRPLANE_MODELS_H__
#define __AIRPLANE_MODELS_H__

#include "airplane.h"

/***
 * A 1Kg glider, with wing span of 1.7m. and length of 1.2m.
 *
 * The wing is assumed to weigh 0.3Kg and the fuselage 0.7Kg.
 */
class SimpleGlider : public Airplane {
public:
  SimpleGlider(irr::scene::ISceneManager *smgr,
               irr::video::IVideoDriver *driver);
  static const float TOTAL_MASS;
  static const float WING_LENGTH;
  static const float FUSELAGE_LENGTH;
  static const float WING_WIDTH;
  static const float FUESLAGE_MASS;
  static const float WING_MASS;
};

/***
 * A 2Kg RC airplane, with wing span of 2m. and length of 1.4m.
 *
 * The wing is assumed to weigh 0.5Kg and the fuselage 1.5Kg.
 */
class Trainer : public Airplane {
public:
  Trainer(irr::scene::ISceneManager *smgr, irr::video::IVideoDriver *driver);
  static const float TOTAL_MASS;
  static const float WING_LENGTH;
  static const float FUSELAGE_LENGTH;
  static const float WING_WIDTH;
  static const float FUESLAGE_MASS;
  static const float WING_MASS;

protected:
  virtual void update_ui() override;

  irr::scene::IMeshSceneNode *m_body_node;
  irr::scene::IMeshSceneNode *m_left_ailron_node;
  irr::scene::IMeshSceneNode *m_right_ailron_node;
  irr::scene::IMeshSceneNode *m_elevator_node;
  irr::scene::IMeshSceneNode *m_rudder_node;
  irr::scene::IMeshSceneNode *m_prop_node;

  float m_prop_angle;
};

#endif // __AIRPLANE_MODELS_H__