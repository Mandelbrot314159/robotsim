# robotsim

A 7-DOF robotic arm simulator written in C++. Drag sliders to pose the arm, snapshot poses as keyframes, hit play to interpolate between them.

Built with SDL2 + OpenGL 3.3 core + [Dear ImGui](https://github.com/ocornut/imgui).

## Requirements

- A C++17 compiler (g++ 11+, clang++ 14+, or MSVC 2019+).
- CMake 3.16+.
- An OpenGL 3.3-capable GPU.

Currently tested only on Linux (Ubuntu 22.04). macOS instructions below should work; Windows instructions are guidance — a GL loader needs to be wired in first, see [Windows notes](#windows).

## Setup

The first two steps are the same on every platform; only the dependency install differs.

### 1. Clone the repo

```bash
git clone https://github.com/Mandelbrot314159/robotsim.git
cd robotsim
```

### 2. Fetch Dear ImGui

```bash
git clone --depth 1 -b docking https://github.com/ocornut/imgui.git external/imgui
```

### 3. Install dependencies (pick your platform)

#### Linux (Ubuntu / Debian)

```bash
sudo apt install -y build-essential cmake libsdl2-dev libglm-dev libgl1-mesa-dev git
```

For Fedora / Arch, install the equivalent packages: a C++ compiler, `cmake`, SDL2 dev headers, GLM, and OpenGL/Mesa dev headers.

#### macOS

You need the Xcode Command Line Tools and Homebrew:

```bash
xcode-select --install
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
brew install cmake sdl2 glm
```

OpenGL is part of the macOS system framework — nothing else to install. (Note: Apple has deprecated OpenGL in favor of Metal; it still works through at least macOS 14, and the code includes `GL_SILENCE_DEPRECATION` to quiet the warnings.)

#### Windows

**Status: not yet supported out of the box.** Windows' `opengl32.dll` only exports OpenGL 1.1 functions, so the project as-written won't link. To get it working you'd need to add a GL loader (e.g. [glad](https://gen.glad.sh/), [glew](https://glew.sourceforge.net/)) and swap the platform conditional in [src/renderer.h](src/renderer.h) and [src/main.cpp](src/main.cpp) to include the loader's header instead. PRs welcome.

If you want to give it a shot anyway:

- Install [Visual Studio 2022](https://visualstudio.microsoft.com/) with the "Desktop development with C++" workload (includes MSVC and CMake), **or** install [MSYS2](https://www.msys2.org/) and use the MinGW-w64 toolchain.
- Install [vcpkg](https://vcpkg.io/en/) and run `vcpkg install sdl2 glm glad` (note: `glad` for the loader you'd add).
- Wire up glad in the renderer (see above), then configure with `cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg root>/scripts/buildsystems/vcpkg.cmake`.

### 4. Configure and build (all platforms)

```bash
cmake -S . -B build
cmake --build build -j
```

The resulting binary is placed at the project root: `robotsim` on Linux/macOS, `robotsim.exe` on Windows.

## Run

```bash
./robotsim
```

(Or `.\robotsim.exe` on Windows.)

## Controls

**Viewport**
- Left mouse drag — orbit the camera
- Right / middle mouse drag — pan
- Scroll wheel — zoom
- `Esc` — quit

**Arm Control panel**
- **Mode**: Manual (sliders drive the arm), IK (drag the tip and the joints solve to follow), or Playback (keyframes drive the arm).
- **IK mode**: a world-aligned transform gizmo appears on the end-effector, and the arm solves to follow its target pose via damped-least-squares inverse kinematics (full 6-DOF: position *and* orientation), with the other joints moving as a consequence and joint limits respected. Grab a handle with the left mouse button:
  - **Center cube** — free move on a camera-facing plane.
  - **Axis bars (X/Y/Z)** — move constrained to one world axis.
  - **Plane quads (XY/YZ/XZ)** — move constrained to a world plane.
  - **Rotation rings (X/Y/Z)** — rotate the tip's orientation about a world axis.

  Clicking empty space still orbits the camera, so you can reposition and work from any angle. You can also type exact target coordinates in the panel, reset the target/orientation to the current tip, and watch the live position/orientation error. Dragging a joint slider drops back to Manual.

  The solver also applies a gentle null-space bias toward each joint's mid-range, which keeps joints off their limits and prevents the arm from locking up in a fully-extended (singular) pose it can't bend back out of. Tune or disable it via `postureGain` in [src/ik.h](src/ik.h).
- **Joint rows**: each joint shows its name and an angle slider (degrees); limits are enforced. **Click a joint's name** to rename it in place. Expand its **"range of motion"** dropdown to edit that joint's min/max travel — widening a joint's range gives the solver more room to work.
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
external/imgui/         Dear ImGui (not checked in — see Setup step 2)
```

The arm definition (joint axes, link lengths, and joint limits) lives in `src/arm.cpp`. Tweak the `joints_[i] = { axis, length, lower, upper }` lines to change the geometry.

## Notes

- **OpenGL loading.** On Linux this project links Mesa's `libGL.so` directly with `GL_GLEXT_PROTOTYPES`, which exports all GL 3.3+ functions natively — no loader needed. On macOS it includes `<OpenGL/gl3.h>` from Apple's OpenGL framework. On Windows you'd need a loader (see [Windows notes](#windows)).
- **No pre-built binaries.** Source builds only. If you want to share a binary with someone, build it on a machine matching their OS.
