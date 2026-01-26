#ifndef __MODEL_CONFIGURATION_H__
#define __MODEL_CONFIGURATION_H__

#include "controls.h"
// #include "dashboard.h"
#include "flying_object.h"
#include "raylib_engine.h"
#include <memory>

struct Configuration {
  std::shared_ptr<FlyingObject> model;
  std::shared_ptr<Controls> controls;
  // std::shared_ptr<Dashboard> dashboard;
};

typedef Configuration (*CreateFunction)(engine::RaylibDevice *device);

struct ModelConfiguration {
  std::string name;
  CreateFunction create;
};

extern std::vector<ModelConfiguration> MODEL_CONFIGURATIONS;

#endif // __MODEL_CONFIGURATION_H__
