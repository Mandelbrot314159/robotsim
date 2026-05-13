#include "keyframes.h"
#include <glm/glm.hpp>
#include <cmath>

namespace robotsim {

float KeyframeSequence::totalDuration() const {
    if (frames.size() < 2) return 0.0f;
    float t = 0.0f;
    for (size_t i = 0; i + 1 < frames.size(); ++i) t += frames[i].duration_to_next;
    return t;
}

void KeyframeSequence::advance(float dt) {
    if (!playing) return;
    float dur = totalDuration();
    if (dur <= 0.0f) { time = 0.0f; return; }
    time += dt;
    if (time > dur) {
        if (looping) {
            time = std::fmod(time, dur);
        } else {
            time = dur;
            playing = false;
        }
    }
}

static float smoothstep(float t) {
    t = glm::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

std::array<float, NUM_JOINTS> KeyframeSequence::sample(float t) const {
    if (frames.empty()) return {};
    if (frames.size() == 1) return frames[0].angles;

    float acc = 0.0f;
    for (size_t i = 0; i + 1 < frames.size(); ++i) {
        float d = frames[i].duration_to_next;
        if (t <= acc + d || i + 2 == frames.size()) {
            float u = d > 0.0f ? (t - acc) / d : 0.0f;
            float s = smoothstep(u);
            std::array<float, NUM_JOINTS> out{};
            for (int j = 0; j < NUM_JOINTS; ++j) {
                out[j] = glm::mix(frames[i].angles[j], frames[i + 1].angles[j], s);
            }
            return out;
        }
        acc += d;
    }
    return frames.back().angles;
}

}  // namespace robotsim
