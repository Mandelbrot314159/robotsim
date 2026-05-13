cmake_minimum_required(VERSION 3.16)
project(robotsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
find_package(OpenGL REQUIRED)

# glm is header-only; libglm-dev ships a cmake config that requires very old CMake,
# so we just locate the headers directly to keep things simple.
find_path(GLM_INCLUDE_DIR glm/glm.hpp PATHS /usr/include /usr/local/include REQUIRED)

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)
add_library(imgui STATIC
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_link_libraries(imgui PUBLIC PkgConfig::SDL2)

add_executable(robotsim
  src/main.cpp
  src/arm.cpp
  src/camera.cpp
  src/keyframes.cpp
  src/renderer.cpp
  src/ui.cpp
)
target_include_directories(robotsim PRIVATE src)
target_compile_options(robotsim PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robotsim PRIVATE
  imgui
  PkgConfig::SDL2
  OpenGL::GL
)
target_include_directories(robotsim PRIVATE ${GLM_INCLUDE_DIR})

target_compile_definitions(robotsim PRIVATE
  SHADER_DIR="${CMAKE_SOURCE_DIR}/shaders"
)
