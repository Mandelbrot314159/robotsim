# robotsim

A 7-DOF robotic arm simulator written in C++. Drag sliders to pose the arm, snapshot poses as keyframes, hit play to interpolate between them.

Built with SDL2 + OpenGL 3.3 core + [Dear ImGui](https://github.com/ocornut/imgui).

## Requirements

- Linux (tested on Ubuntu 22.04). Should be straightforward to port to macOS / Windows; only the dependency install steps would change.
- A C++17 compiler (g++ 11+ or clang++ 14+).
- CMake 3.16+.
- An OpenGL 3.3-capable GPU.

## Setup

### 1. Install system dependencies

```bash
sudo apt install -y build-essential cmake libsdl2-dev libglm-dev libgl1-mesa-dev git
```

### 2. Clone the repo

```bash
git clone https://github.com/Mandelbrot314159/robotsim.git
cd robotsim
```

### 3. Fetch Dear ImGui

ImGui is not vendored in this repo; clone it into `external/imgui/`:

```bash
git clone --depth 1 -b docking https://github.com/ocornut/imgui.git external/imgui
```

### 4. Configure and build

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

The resulting `robotsim` binary is placed in the project root.

## Run

```bash
./robotsim
```

## Controls

**Viewport**
- Left mouse drag — orbit the camera
- Right / middle mouse drag — pan
- Scroll wheel — zoom
- `Esc` — quit

**Arm Control panel**
- **Mode**: Manual (sliders drive the arm) or Playback (keyframes drive the arm).
- **Joint sliders (J1–J7)**: drag in degrees. Joint limits are enforced.
- **Add (snapshot current pose)**: appends a keyframe with the current joint angles.
- **Frame list**: click a frame to select it. The selected frame can be loaded back into the sliders, overwritten from the current pose, deleted, or have its "duration to next" tweaked.
- **Play / Pause / Stop / Loop**: standard playback controls. The time slider scrubs through the sequence; segments are eased with smoothstep.

## Project layout

```
CMakeLists.txt          Build configuration
shaders/                GLSL shaders (lit cube + flat-color grid)
src/
  main.cpp              SDL2 window, GL context, main loop
  arm.{h,cpp}           7-DOF forward kinematics
  camera.{h,cpp}        Orbit camera
  renderer.{h,cpp}      Minimal GL renderer (cube mesh, grid)
  keyframes.{h,cpp}     Keyframe storage + smoothstep interpolation
  ui.{h,cpp}            ImGui panels (sliders, keyframe list)
external/imgui/         Dear ImGui (not checked in — see Setup step 3)
```

The arm definition (joint axes, link lengths, and joint limits) lives in `src/arm.cpp`. Tweak the `joints_[i] = { axis, length, lower, upper }` lines to change the geometry.

## Notes

- This project uses Mesa's `libGL.so` directly with `GL_GLEXT_PROTOTYPES` — no GL loader (glad / glew / epoxy) is required on Linux. If you port to a system where `libGL` doesn't export GL 3.3+ functions, add a loader.
