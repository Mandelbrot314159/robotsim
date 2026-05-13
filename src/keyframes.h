#pragma once
#include <array>
#include <vector>
#include "arm.h"

namespace robotsim {

struct Keyframe {
    std::array<float, NUM_JOINTS> angles{};
    float duration_to_next = 1.0f;  // seconds to interpolate from this frame to the next
};

class KeyframeSequence {
public:
    std::vector<Keyframe> frames;
    bool  playing = false;
    bool  looping = true;
    float time    = 0.0f;

    float totalDuration() const;
    void  advance(float dt);
    std::array<float, NUM_JOINTS> sample(float t) const;
    std::array<float, NUM_JOINTS> currentAngles() const { return sample(time); }
};

}  // namespace robotsim
