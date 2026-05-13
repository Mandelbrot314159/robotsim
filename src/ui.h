#pragma once
#include "arm.h"
#include "keyframes.h"

namespace robotsim {

struct UIState {
    bool useKeyframes  = false;
    int  selectedFrame = -1;
};

void drawUI(UIState& ui, Arm& arm, KeyframeSequence& seq);

}  // namespace robotsim
