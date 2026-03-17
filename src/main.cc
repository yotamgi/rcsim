/** Simulator main file. */
#include "game.h"

#include <iostream>
#include <sys/time.h>
#include <unistd.h>

#ifdef PLATFORM_WEB
#include <emscripten.h>
#endif

int main() {
  Game game;

#ifdef PLATFORM_WEB
  emscripten_set_main_loop_arg(
      [](void *arg) { static_cast<Game *>(arg)->frame(); }, &game, 0, true);
#else
  while (!WindowShouldClose()) {
    // calculate the delta time, and make sure it does not exceed 100 fps
    game.frame();
  }
#endif

  return 0;
}