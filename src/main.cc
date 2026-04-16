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
  while (game.frame());
#endif

  return 0;
}