#pragma once
#include <array>
#include <string>
#include <glm/glm.hpp>

namespace robotsim {

constexpr int NUM_JOINTS = 7;

struct JointConfig {
    glm::vec3 axis;   // local rotation axis (unit vector)
    float length;     // length of the link extending after this joint, along local +Z
    float lower;      // joint angle lower limit (radians)
    float upper;      // joint angle upper limit (radians)
};

class Arm {
public:
    Arm();

    void  setAngle(int joint, float radians);
    void  setAngles(const std::array<float, NUM_JOINTS>& angles);
    float getAngle(int joint) const { return angles_[joint]; }
    const std::array<float, NUM_JOINTS>& getAngles() const { return angles_; }

    const JointConfig& config(int joint) const { return joints_[joint]; }

    // Editable joint range of motion (radians). Re-clamps the current angle.
    void setLimits(int joint, float lower, float upper);

    // Editable joint display name.
    const std::string& name(int joint) const { return names_[joint]; }
    void setName(int joint, const std::string& n) { names_[joint] = n; }

    // World transform of the frame AT joint i (before its link is drawn).
    // Link i is drawn extending along local +Z from this frame.
    std::array<glm::mat4, NUM_JOINTS> linkTransforms() const;

    // Transform of the end-effector (tip of the last link).
    glm::mat4 endEffector() const;

private:
    std::array<JointConfig, NUM_JOINTS> joints_;
    std::array<std::string, NUM_JOINTS> names_;
    std::array<float, NUM_JOINTS> angles_{};
};

}  // namespace robotsim
