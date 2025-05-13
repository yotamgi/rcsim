#ifndef __ARROW_H__
#define __ARROW_H__

#include <irrlicht/irrlicht.h>

typedef irr::core::vector3df irrvec3;

class Arrow {
public:
  Arrow(irr::scene::ISceneManager *smgr);

  void point(const irrvec3 &at);

private:
  irr::scene::ISceneNode *m_node;
};

#endif // __ARROW_H__
