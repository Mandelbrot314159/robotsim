#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "arm.h"
#include "keyframes.h"

namespace robotsim {

enum class Mode { Manual, IK, Playback };

struct UIState {
    Mode mode          = Mode::Manual;
    int  selectedFrame = -1;

    // Joint rename-in-place state.
    int  editingName       = -1;     // joint index currently being renamed, or -1
    bool nameFocusPending  = false;  // request keyboard focus on the rename field
    char nameBuf[64]       = {0};

    // Inverse-kinematics goal pose in world space; valid while mode == Mode::IK.
    glm::vec3 ikTarget{0.0f};
    glm::quat ikRot{1.0f, 0.0f, 0.0f, 0.0f};
    float     ikError    = 0.0f;
    float     ikAngError = 0.0f;
};

void drawUI(UIState& ui, Arm& arm, KeyframeSequence& seq);

}  // namespace robotsim
