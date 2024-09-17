
#include "arrow.h"
#include <iostream>


Arrow::Arrow(irr::scene::ISceneManager *smgr) {
    m_node = smgr->addEmptySceneNode();

	irr::scene::ISceneNode *arrow_node = smgr->addAnimatedMeshSceneNode(
        smgr->addArrowMesh("arrow", 
				irr::video::SColor(255, 255, 0, 0), 
                irr::video::SColor(255, 0, 255, 0)), 
                m_node);

    arrow_node->setMaterialFlag(irr::video::EMF_LIGHTING, true);
    arrow_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    arrow_node->setDebugDataVisible(irr::scene::EDS_OFF);
    arrow_node->setRotation(irrvec3(90, 0, 0));
}

void Arrow::point(const irrvec3 &at) {
    m_node->setScale(irrvec3(1, 1, at.getLength()));
    m_node->setRotation(at.getHorizontalAngle());
}
