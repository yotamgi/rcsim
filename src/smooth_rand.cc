
#include "smooth_rand.h"
#include <cstdlib>

float rand_float() { return (float)std::rand() / (float)RAND_MAX; }
engine::vec3 rand_vec3() {
  return engine::vec3(rand_float(), rand_float(), rand_float());
}
