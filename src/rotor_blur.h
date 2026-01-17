#ifndef __ROTOR_BLUR_H__
#define __ROTOR_BLUR_H__

#include <irrlicht/irrlicht.h>
typedef irr::core::vector3df irrvec3;

class RotorBlur {
public:
  void init_ui(irr::scene::ISceneManager *smgr, float radius, irrvec3 position,
               irrvec3 rotation, float thickness,
               irr::scene::IMeshSceneNode *parent_node);

private:
  // Methods to be overriden by decendents.
  virtual float get_width(float along_radius) const = 0;
  virtual irr::video::SColor get_color(float along_radius) const = 0;

  class Manipulator : public irr::scene::IVertexManipulator {
  public:
    Manipulator(const RotorBlur *parent) : m_parent(parent) {}
    void operator()(irr::video::S3DVertex &vertex) const;

  private:
    const RotorBlur *const m_parent;
  };
  irr::scene::IMeshSceneNode *m_blur_node;
};

#endif // __ROTOR_BLUR_H__
