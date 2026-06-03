#include <SDL.h>
#if defined(__APPLE__)
    #define GL_SILENCE_DEPRECATION
    #include <OpenGL/gl3.h>
    #include <OpenGL/gl3ext.h>
#else
    #define GL_GLEXT_PROTOTYPES
    #include <GL/gl.h>
    #include <GL/glext.h>
#endif
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <chrono>
#include <iostream>

#include "arm.h"
#include "camera.h"
#include "ik.h"
#include "keyframes.h"
#include "picking.h"
#include "renderer.h"
#include "ui.h"

using namespace robotsim;

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    SDL_Window* win = SDL_CreateWindow(
        "robotsim",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 800,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!win) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_GLContext ctx = SDL_GL_CreateContext(win);
    if (!ctx) {
        std::cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
        return 1;
    }
    if (SDL_GL_MakeCurrent(win, ctx) != 0) {
        std::cerr << "SDL_GL_MakeCurrent failed: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_GL_SetSwapInterval(1);

    Renderer renderer;
    if (!renderer.init(SHADER_DIR)) {
        std::cerr << "Renderer init failed\n";
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(win, ctx);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    Arm arm;
    arm.setAngle(1, glm::radians(-30.0f));
    arm.setAngle(3, glm::radians(-70.0f));
    arm.setAngle(5, glm::radians(40.0f));

    OrbitCamera cam;
    KeyframeSequence seq;
    UIState ui;

    static const glm::vec3 link_colors[NUM_JOINTS] = {
        {0.85f, 0.30f, 0.30f}, {0.95f, 0.55f, 0.20f}, {0.95f, 0.85f, 0.25f},
        {0.40f, 0.80f, 0.30f}, {0.30f, 0.75f, 0.85f}, {0.40f, 0.50f, 0.90f},
        {0.70f, 0.40f, 0.85f},
    };

    IKConfig ik_cfg;
    const float kTargetPickRadius = 0.10f;  // generous grab radius for the IK target handle

    auto t_prev = std::chrono::steady_clock::now();
    bool running = true;
    bool dragging_orbit  = false;
    bool dragging_pan    = false;
    bool dragging_target = false;
    Mode prev_mode = ui.mode;

    while (running) {
        int w, h;
        SDL_GL_GetDrawableSize(win, &w, &h);
        float aspect = h > 0 ? (float)w / (float)h : 1.0f;
        int win_w, win_h;
        SDL_GetWindowSize(win, &win_w, &win_h);  // logical size: mouse coords live here
        glm::mat4 view = cam.view();
        glm::mat4 proj = cam.projection(aspect);

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL2_ProcessEvent(&ev);

            if (ev.type == SDL_QUIT) running = false;
            if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) running = false;

            if (io.WantCaptureMouse) {
                dragging_orbit  = false;
                dragging_pan    = false;
                dragging_target = false;
                continue;
            }

            if (ev.type == SDL_MOUSEBUTTONDOWN) {
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    // In IK mode, a left click on the target handle grabs it;
                    // otherwise the left button orbits the camera as usual.
                    bool grabbed = false;
                    if (ui.mode == Mode::IK) {
                        Ray r = screenRay((float)ev.button.x, (float)ev.button.y,
                                          (float)win_w, (float)win_h, view, proj);
                        float t;
                        if (raySphere(r, ui.ikTarget, kTargetPickRadius, t)) {
                            dragging_target = true;
                            grabbed = true;
                        }
                    }
                    if (!grabbed) dragging_orbit = true;
                }
                if (ev.button.button == SDL_BUTTON_RIGHT)  dragging_pan = true;
                if (ev.button.button == SDL_BUTTON_MIDDLE) dragging_pan = true;
            }
            if (ev.type == SDL_MOUSEBUTTONUP) {
                if (ev.button.button == SDL_BUTTON_LEFT) {
                    dragging_orbit  = false;
                    dragging_target = false;
                }
                if (ev.button.button == SDL_BUTTON_RIGHT)  dragging_pan = false;
                if (ev.button.button == SDL_BUTTON_MIDDLE) dragging_pan = false;
            }
            if (ev.type == SDL_MOUSEMOTION) {
                if (dragging_target && ui.mode == Mode::IK) {
                    // Slide the target on a camera-facing plane through its current
                    // position, so mouse motion maps to intuitive screen-space motion.
                    Ray r = screenRay((float)ev.motion.x, (float)ev.motion.y,
                                      (float)win_w, (float)win_h, view, proj);
                    glm::vec3 n = glm::normalize(cam.target - cam.position());
                    glm::vec3 hit;
                    if (rayPlane(r, ui.ikTarget, n, hit)) ui.ikTarget = hit;
                } else {
                    if (dragging_orbit) cam.orbit((float)ev.motion.xrel, (float)ev.motion.yrel);
                    if (dragging_pan)   cam.pan  ((float)ev.motion.xrel, (float)ev.motion.yrel);
                }
            }
            if (ev.type == SDL_MOUSEWHEEL) cam.zoom((float)ev.wheel.y);
        }

        auto t_now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(t_now - t_prev).count();
        t_prev = t_now;

        // Snap the IK target to the current tip whenever IK mode is entered,
        // so the arm doesn't jump on the first solve.
        if (ui.mode == Mode::IK && prev_mode != Mode::IK) {
            ui.ikTarget = glm::vec3(arm.endEffector()[3]);
        }
        prev_mode = ui.mode;

        if (ui.mode == Mode::Playback && seq.playing) {
            seq.advance(dt);
            arm.setAngles(seq.currentAngles());
        } else if (ui.mode == Mode::Playback && !seq.frames.empty()) {
            // Allow scrubbing via time slider when not playing
            arm.setAngles(seq.currentAngles());
        } else if (ui.mode == Mode::IK) {
            ui.ikError = solveIKPosition(arm, ui.ikTarget, ik_cfg).error;
        }

        glEnable(GL_MULTISAMPLE);
        renderer.beginFrame(w, h, cam.view(), cam.projection(aspect), cam.position());
        renderer.drawGround();

        // Base pedestal
        renderer.drawBox(
            glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0.04f)) *
                glm::scale(glm::mat4(1.0f), glm::vec3(0.22f, 0.22f, 0.08f)),
            glm::vec3(0.32f));

        auto Ts = arm.linkTransforms();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            float len = arm.config(i).length;
            glm::mat4 joint = Ts[i] * glm::scale(glm::mat4(1.0f), glm::vec3(0.07f));
            renderer.drawBox(joint, glm::vec3(0.18f, 0.18f, 0.20f));
            glm::mat4 link = Ts[i] *
                             glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, len * 0.5f)) *
                             glm::scale(glm::mat4(1.0f), glm::vec3(0.05f, 0.05f, len));
            renderer.drawBox(link, link_colors[i]);
        }

        // End-effector
        glm::mat4 ee = arm.endEffector() * glm::scale(glm::mat4(1.0f), glm::vec3(0.055f));
        renderer.drawBox(ee, glm::vec3(0.95f, 0.95f, 0.95f));

        // IK target handle: green once reached, orange while chasing.
        if (ui.mode == Mode::IK) {
            glm::vec3 color = (ui.ikError < 0.01f) ? glm::vec3(0.30f, 0.90f, 0.40f)
                                                   : glm::vec3(0.95f, 0.60f, 0.20f);
            glm::mat4 marker = glm::translate(glm::mat4(1.0f), ui.ikTarget) *
                               glm::scale(glm::mat4(1.0f), glm::vec3(0.06f));
            renderer.drawBox(marker, color);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI(ui, arm, seq);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(win);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(ctx);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
