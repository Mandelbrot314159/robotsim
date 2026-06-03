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

    // Inverse-kinematics goal pose in world space; valid while mode == Mode::IK.
    glm::vec3 ikTarget{0.0f};
    glm::quat ikRot{1.0f, 0.0f, 0.0f, 0.0f};
    float     ikError    = 0.0f;
    float     ikAngError = 0.0f;
};

void drawUI(UIState& ui, Arm& arm, KeyframeSequence& seq);

}  // namespace robotsim
