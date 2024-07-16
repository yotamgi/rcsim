#include "rotor_blur.h"

const float PI = 3.14159265;


void RotorBlur::init_ui(
        irr::scene::ISceneManager *smgr,
        float radius, 
        irrvec3 position,
        irrvec3 rotation,
        float thickness,
        irr::scene::IMeshSceneNode* parent_node)
{
    m_blur_node = smgr->addSphereSceneNode(
            radius,
            256,  // num vertices.
            parent_node,
            -1,
            position,
            rotation,
            irrvec3(1, thickness, 1)  // scale.
    );
    m_blur_node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
    m_blur_node->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
    m_blur_node->getMaterial(0).MaterialType = irr::video::EMT_TRANSPARENT_VERTEX_ALPHA;
    smgr->getMeshManipulator()->apply(Manipulator(this), m_blur_node->getMesh());
}

void RotorBlur::Manipulator::operator()(irr::video::S3DVertex& vertex) const {
    float distance = std::sqrt(vertex.Pos.X*vertex.Pos.X + vertex.Pos.Z*vertex.Pos.Z);
    float alpha = m_parent->get_width(distance) / (2 * distance * PI);
    alpha = alpha > 1 ? 1 : alpha;
    irr::video::SColor color = m_parent->get_color(distance);
    vertex.Color.set(255 * alpha, color.getRed(), color.getGreen(), color.getBlue());
}

