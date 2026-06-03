#include "keyframes.h"
#include <glm/glm.hpp>
#include <cmath>

namespace robotsim {

static float smoothstep(float t) {
    t = glm::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static std::array<float, NUM_JOINTS> lerpAngles(
    const std::array<float, NUM_JOINTS>& a,
    const std::array<float, NUM_JOINTS>& b,
    float t, float dur)
{
    float u = dur > 0.0f ? t / dur : 0.0f;
    float s = smoothstep(u);
    std::array<float, NUM_JOINTS> out{};
    for (int j = 0; j < NUM_JOINTS; ++j) {
        out[j] = glm::mix(a[j], b[j], s);
    }
    return out;
}

float KeyframeSequence::totalDuration() const {
    if (frames.size() < 2) return 0.0f;
    float t = 0.0f;
    for (size_t i = 0; i + 1 < frames.size(); ++i) t += frames[i].duration_to_next;
    if (looping) t += frames.back().duration_to_next;  // wrap segment: last -> first
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

std::array<float, NUM_JOINTS> KeyframeSequence::sample(float t) const {
    if (frames.empty()) return {};
    if (frames.size() == 1) return frames[0].angles;

    float acc = 0.0f;
    for (size_t i = 0; i + 1 < frames.size(); ++i) {
        float d = frames[i].duration_to_next;
        if (t <= acc + d) {
            return lerpAngles(frames[i].angles, frames[i + 1].angles, t - acc, d);
        }
        acc += d;
    }
    if (looping) {
        float d = frames.back().duration_to_next;
        if (t <= acc + d) {
            return lerpAngles(frames.back().angles, frames.front().angles, t - acc, d);
        }
    }
    return frames.back().angles;
}

}  // namespace robotsim
