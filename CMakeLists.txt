cmake_minimum_required(VERSION 3.11)
project(rcsim LANGUAGES CXX)

# 1. Include FetchContent module
include(FetchContent)

# 2. Declare raylib dependency
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.5 # Or "master" for the latest
)

# 3. Disable raylib's internal examples/tests to save time
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# 4. Make raylib available to the project
FetchContent_MakeAvailable(raylib)

# 5. Fetch the C++ Wrapper
FetchContent_Declare(
  raylib_cpp
  GIT_REPOSITORY https://github.com/RobLoach/raylib-cpp.git
  GIT_TAG v5.5.0
)
FetchContent_MakeAvailable(raylib_cpp)

# 6. Define your executable
add_executable(${PROJECT_NAME} 
    src/main.cc
    src/raylib_engine.cc
    src/rlights.cc
    src/input_event_reciever.cc
    src/airplane.cc
    src/heli.cc
    src/smooth_rand.cc
    src/flight_controller.cc
    src/airplane_models.cc
    src/flying_object.cc
    src/controls.cc
    src/model_configurations.cc
)

# 7. Link against raylib
# On Linux, you also need to link standard system math/threading libs
target_link_libraries(${PROJECT_NAME} raylib_cpp raylib)

if(EMSCRIPTEN)
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s USE_GLFW=3 -s ASSERTIONS=1 -s WASM=1 -s ASYNCIFY -s GL_ENABLE_GET_PROC_ADDRESS=1 --preload-file resources -s ALLOW_MEMORY_GROWTH=1")
  set(CMAKE_EXECUTABLE_SUFFIX ".html") # This line is used to set your executable to build with the emscripten html template so that you can directly open it.
  target_compile_options(${PROJECT_NAME} PRIVATE -DGRAPHICS_API_OPENGL_ES2 -DPLATFORM_WEB)
else()
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lGL")
  target_compile_options(${PROJECT_NAME} PRIVATE -DGRAPHICS_API_OPENGL_33 -DPLATFORM_DESKTOP)
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(${PROJECT_NAME} m pthread dl rt X11)
endif()
