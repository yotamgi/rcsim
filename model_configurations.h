#ifndef __MODEL_CONFIGURATION_H__
#define __MODEL_CONFIGURATION_H__

#include "flying_object.h"
#include "controls.h"
#include "dashboard.h"
#include <memory>
#include <irrlicht/irrlicht.h>

struct Configuration {
    std::shared_ptr<FlyingObject> model;
    std::shared_ptr<Controls> controls;
    std::shared_ptr<Dashboard> dashboard;
};

typedef Configuration (*CreateFunction)(irr::video::IVideoDriver*, irr::scene::ISceneManager*);

struct ModelConfiguration {
    std::string name;
    CreateFunction create;
};

extern std::vector<ModelConfiguration> MODEL_CONFIGURATIONS;

#endif // __MODEL_CONFIGURATION_H__
